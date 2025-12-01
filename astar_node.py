#!/usr/bin/env python3
import threading
import math
import time
import heapq
import random

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Empty

import tkinter as tk

# -------------------------------
# GRID / MAP CONSTANTS
# -------------------------------
CELL_SIZE = 24
GRID_W = 28
GRID_H = 18

W = GRID_W * CELL_SIZE
H = GRID_H * CELL_SIZE

EMPTY = 0
WALL = 1

# Map parameters for RViz
MAP_FRAME = "map"
MAP_RESOLUTION = 1.0  # meters per cell
MAP_ORIGIN_X = 0.0
MAP_ORIGIN_Y = 0.0

# Colors
COLOR_BG = "#1E1E1E"
COLOR_EMPTY = "#2D2D2D"
COLOR_WALL = "#141470"
COLOR_START = "#1BE81B"
COLOR_END = "#1BBCE8"
COLOR_VISITED = "#C83232"
COLOR_FRONTIER = "#FF9900"
COLOR_PATH = "#25E825"
GRID_LINE = "#333333"

# -------------------------------
# GLOBAL STATE (GUI + A*)
# -------------------------------
grid = [[EMPTY for _ in range(GRID_W)] for _ in range(GRID_H)]
start = None              # (r, c)
end = None                # (r, c)

frontier_set = set()
visited_set = set()
parent = {}
g_score = {}
f_score = {}

paused = False
algo_gen = None
algorithm_running = False
mode = None
mouse_down = False
placing_wall = True

# -------------------------------
# TKINTER GUI SETUP
# -------------------------------
root = tk.Tk()
root.title("A* with ROS2 + RViz + Services")
canvas = tk.Canvas(root, width=W, height=H, bg=COLOR_BG)
canvas.pack()


# -------------------------------
# HELPER: WORLD <-> GRID
# -------------------------------
def world_to_grid(x: float, y: float):
    """Convert world coordinates (map frame) to grid cell (r, c)."""
    c = int(round((x - MAP_ORIGIN_X) / MAP_RESOLUTION))
    r = int(round((y - MAP_ORIGIN_Y) / MAP_RESOLUTION))
    if 0 <= r < GRID_H and 0 <= c < GRID_W:
        return (r, c)
    return None


def grid_to_world(r: int, c: int):
    """Convert grid cell (r, c) to world coordinates (map frame) at cell center."""
    x = MAP_ORIGIN_X + (c + 0.5) * MAP_RESOLUTION
    y = MAP_ORIGIN_Y + (r + 0.5) * MAP_RESOLUTION
    return x, y


# -------------------------------
# DRAWING
# -------------------------------
def draw():
    canvas.delete("all")

    for r in range(GRID_H):
        for c in range(GRID_W):
            x1, y1 = c * CELL_SIZE, r * CELL_SIZE
            x2, y2 = x1 + CELL_SIZE, y1 + CELL_SIZE

            color = COLOR_WALL if grid[r][c] == WALL else COLOR_EMPTY
            canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline=GRID_LINE)

    for r, c in visited_set:
        x1, y1 = c * CELL_SIZE + 3, r * CELL_SIZE + 3
        x2, y2 = x1 + CELL_SIZE - 6, y1 + CELL_SIZE - 6
        canvas.create_rectangle(x1, y1, x2, y2, fill=COLOR_VISITED, width=0)

    for r, c in frontier_set:
        x1, y1 = c * CELL_SIZE + 4, r * CELL_SIZE + 4
        x2, y2 = x1 + CELL_SIZE - 8, y1 + CELL_SIZE - 8
        canvas.create_rectangle(x1, y1, x2, y2, fill=COLOR_FRONTIER, width=0)

    # Path from parent[]
    if end in parent:
        cur = end
        while cur in parent:
            r, c = cur
            x1, y1 = c * CELL_SIZE + 6, r * CELL_SIZE + 6
            x2, y2 = x1 + CELL_SIZE - 12, y1 + CELL_SIZE - 12
            canvas.create_rectangle(x1, y1, x2, y2, fill=COLOR_PATH, width=0)
            cur = parent[cur]

    if start:
        r, c = start
        canvas.create_rectangle(
            c * CELL_SIZE, r * CELL_SIZE,
            c * CELL_SIZE + CELL_SIZE, r * CELL_SIZE + CELL_SIZE,
            fill=COLOR_START
        )

    if end:
        r, c = end
        canvas.create_rectangle(
            c * CELL_SIZE, r * CELL_SIZE,
            c * CELL_SIZE + CELL_SIZE, r * CELL_SIZE + CELL_SIZE,
            fill=COLOR_END
        )


# -------------------------------
# RANDOM MAP GENERATOR
# -------------------------------
def generate_random_map(density: float = 0.25):
    """
    Generate a random map with given wall density (0.0–1.0).
    Does not overwrite start or end positions.
    """
    global grid

    for r in range(GRID_H):
        for c in range(GRID_W):
            # keep start and end clear
            if (start and (r, c) == start) or (end and (r, c) == end):
                grid[r][c] = EMPTY
                continue

            grid[r][c] = WALL if random.random() < density else EMPTY

    draw()


# -------------------------------
# A* WITH DIAGONALS (NO CORNER CUTTING)
# -------------------------------
def neighbors(cell):
    r, c = cell

    # 4-directional orthogonal movement
    orthogonal = [
        (-1, 0),  # up
        (1, 0),   # down
        (0, -1),  # left
        (0, 1)    # right
    ]

    # First add safe orthogonal moves
    for dr, dc in orthogonal:
        nr, nc = r + dr, c + dc
        if 0 <= nr < GRID_H and 0 <= nc < GRID_W:
            if grid[nr][nc] != WALL:
                yield (nr, nc), 1.0

    # Safe diagonal movement (NO corner cutting)
    diagonals = [
        (-1, -1),
        (-1,  1),
        ( 1, -1),
        ( 1,  1)
    ]

    for dr, dc in diagonals:
        nr, nc = r + dr, c + dc

        # Skip if out of bounds
        if not (0 <= nr < GRID_H and 0 <= nc < GRID_W):
            continue

        # Skip if diagonal target is a wall
        if grid[nr][nc] == WALL:
            continue

        # Check the two adjacent orthogonal cells
        adj1 = (r + dr, c)  # vertical neighbor
        adj2 = (r, c + dc)  # horizontal neighbor

        # Both must be free to avoid corner cutting
        if (0 <= adj1[0] < GRID_H and 0 <= adj1[1] < GRID_W and
            0 <= adj2[0] < GRID_H and 0 <= adj2[1] < GRID_W):

            if grid[adj1[0]][adj1[1]] != WALL and grid[adj2[0]][adj2[1]] != WALL:
                yield (nr, nc), math.sqrt(2)


def heuristic(a, b):
    """Octile heuristic."""
    (r1, c1) = a
    (r2, c2) = b
    dx = abs(r1 - r2)
    dy = abs(c1 - c2)
    D = 1.0
    D2 = math.sqrt(2)
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)


def astar(start_cell, end_cell):
    """Generator version of A* for visualization."""
    global frontier_set, visited_set, parent, g_score, f_score

    frontier_set = set()
    visited_set = set()
    parent = {}
    g_score = {}
    f_score = {}

    pq = []
    g_score[start_cell] = 0.0
    f_score[start_cell] = heuristic(start_cell, end_cell)
    heapq.heappush(pq, (f_score[start_cell], 0.0, start_cell))
    frontier_set.add(start_cell)

    while pq:
        fcur, _, cur = heapq.heappop(pq)
        if fcur > f_score.get(cur, math.inf):
            continue

        frontier_set.discard(cur)
        visited_set.add(cur)

        # yield for visualization
        yield False

        if cur == end_cell:
            yield True
            return

        for nb, w in neighbors(cur):
            tentative = g_score[cur] + w
            if tentative < g_score.get(nb, math.inf):
                g_score[nb] = tentative
                parent[nb] = cur
                f_score[nb] = tentative + heuristic(nb, end_cell)
                heapq.heappush(pq, (f_score[nb], tentative, nb))
                if nb not in visited_set:
                    frontier_set.add(nb)

        time.sleep(0.01)


def get_current_path_cells():
    """Return list of cells [(r,c), ...] from start to end if path exists."""
    if end not in parent:
        return []
    path_cells = []
    cur = end
    while cur in parent:
        path_cells.append(cur)
        cur = parent[cur]
    path_cells.reverse()
    return path_cells


# -------------------------------
# ANIMATION
# -------------------------------
def step():
    global algo_gen, algorithm_running
    if not algorithm_running or paused:
        return

    try:
        done = next(algo_gen)
        draw()
        if done:
            algorithm_running = False
            print("A* finished | distance =", g_score.get(end, math.inf))
        else:
            root.after(10, step)
    except StopIteration:
        algorithm_running = False


# -------------------------------
# MOUSE HANDLERS
# -------------------------------
def on_mouse_down(event):
    global mouse_down, placing_wall, mode, start, end

    mouse_down = True
    r = event.y // CELL_SIZE
    c = event.x // CELL_SIZE

    if not (0 <= r < GRID_H and 0 <= c < GRID_W):
        return

    if mode == "s":
        if grid[r][c] != WALL:
            start = (r, c)
        mode = None
        draw()
        return

    if mode == "e":
        if grid[r][c] != WALL:
            end = (r, c)
        mode = None
        draw()
        return

    if grid[r][c] == WALL:
        placing_wall = False
        grid[r][c] = EMPTY
    else:
        placing_wall = True
        grid[r][c] = WALL

    draw()


def on_mouse_move(event):
    if not mouse_down or algorithm_running or mode in ("s", "e"):
        return
    r = event.y // CELL_SIZE
    c = event.x // CELL_SIZE
    if 0 <= r < GRID_H and 0 <= c < GRID_W:
        grid[r][c] = WALL if placing_wall else EMPTY
        draw()


def on_mouse_up(event):
    global mouse_down
    mouse_down = False


# -------------------------------
# KEYBOARD HANDLER
# -------------------------------
def on_key(event):
    global mode, algo_gen, algorithm_running, paused, start, end

    key = event.keysym.lower()

    if key == "s":
        mode = "s"
    elif key == "e":
        mode = "e"
    elif key == "space":
        if start and end:
            algo_gen = astar(start, end)
            algorithm_running = True
            paused = False
            root.after(1, step)
        else:
            print("Set START and END first.")
    elif key == "p":
        paused = not paused
        if not paused:
            step()
    elif key == "c":
        # clear only walls
        for r in range(GRID_H):
            for c in range(GRID_W):
                if grid[r][c] == WALL:
                    grid[r][c] = EMPTY
        draw()
    elif key == "r":
        # full reset (local)
        start = None
        end = None
        for r in range(GRID_H):
            for c in range(GRID_W):
                grid[r][c] = EMPTY
        draw()
    elif key == "g":
        # generate random map
        print("Generating random map...")
        generate_random_map(density=0.25)
    elif key in ("escape", "q"):
        root.destroy()


canvas.bind("<Button-1>", on_mouse_down)
canvas.bind("<ButtonRelease-1>", on_mouse_up)
canvas.bind("<B1-Motion>", on_mouse_move)
root.bind("<Key>", on_key)

draw()


# -------------------------------
# ROS2 NODE
# -------------------------------
class AStarNode(Node):
    def __init__(self):
        super().__init__('astar_gui_node')
        self.get_logger().info("A* GUI Node with RViz + Services started.")

        # Publishers
        self.occ_grid_pub = self.create_publisher(OccupancyGrid, 'astar_grid', 10)
        self.path_pub = self.create_publisher(Path, 'astar_path', 10)

        # Subscribers from RViz
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.initialpose_cb,
            10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_cb,
            10
        )

        # Services: clear walls / reset planner
        self.clear_srv = self.create_service(
            Empty,
            'clear_grid',
            self.clear_grid_cb
        )
        self.reset_srv = self.create_service(
            Empty,
            'reset_planner',
            self.reset_planner_cb
        )

        # Timer to periodically publish grid & current path
        self.timer = self.create_timer(0.5, self.timer_cb)

    # --- RViz callbacks ---
    def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        global start

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cell = world_to_grid(x, y)
        if cell is None:
            self.get_logger().warn("Received initialpose outside grid.")
            return

        self.get_logger().info(f"Received start from RViz: {cell}")

        def tk_update():
            global start
            start = cell
            draw()
        root.after(0, tk_update)

    def goal_cb(self, msg: PoseStamped):
        global end, algo_gen, algorithm_running, paused, start

        x = msg.pose.position.x
        y = msg.pose.position.y
        cell = world_to_grid(x, y)
        if cell is None:
            self.get_logger().warn("Received goal outside grid.")
            return

        self.get_logger().info(f"Received goal from RViz: {cell}")

        def tk_update():
            global end, algo_gen, algorithm_running, paused, start
            end = cell
            draw()
            if start and end:
                algo_gen = astar(start, end)
                algorithm_running = True
                paused = False
                root.after(1, step)

        root.after(0, tk_update)

    # --- Services ---
    def clear_grid_cb(self, request, response):
        """Clear only walls (leave start/end and A* state)."""
        self.get_logger().info("Received /clear_grid → removing all walls.")

        def tk_update():
            global grid
            for r in range(GRID_H):
                for c in range(GRID_W):
                    if grid[r][c] == WALL:
                        grid[r][c] = EMPTY
            draw()

        root.after(0, tk_update)
        return response

    def reset_planner_cb(self, request, response):
        """Reset everything: grid, start, end, visited, frontier, A* state."""
        self.get_logger().info("Received /reset_planner → full reset.")

        def tk_update():
            global grid, start, end
            global frontier_set, visited_set, parent, g_score, f_score
            global algo_gen, algorithm_running, paused, mode

            start = None
            end = None
            frontier_set = set()
            visited_set = set()
            parent = {}
            g_score = {}
            f_score = {}
            algo_gen = None
            algorithm_running = False
            paused = False
            mode = None

            for r in range(GRID_H):
                for c in range(GRID_W):
                    grid[r][c] = EMPTY

            draw()

        root.after(0, tk_update)
        return response

    # --- Timer to publish to RViz ---
    def timer_cb(self):
        self.publish_occupancy_grid()
        self.publish_path()

    def publish_occupancy_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = MAP_FRAME

        info = MapMetaData()
        info.resolution = MAP_RESOLUTION
        info.width = GRID_W
        info.height = GRID_H
        info.origin.position.x = MAP_ORIGIN_X
        info.origin.position.y = MAP_ORIGIN_Y
        info.origin.orientation.w = 1.0
        msg.info = info

        data = []
        for r in range(GRID_H):
            for c in range(GRID_W):
                if grid[r][c] == WALL:
                    data.append(100)   # occupied
                else:
                    data.append(0)     # free
        msg.data = data

        self.occ_grid_pub.publish(msg)

    def publish_path(self):
        cells = get_current_path_cells()
        if not cells:
            return

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = MAP_FRAME

        for r, c in cells:
            x, y = grid_to_world(r, c)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)


# -------------------------------
# MAIN
# -------------------------------
def main():
    rclpy.init()

    node = AStarNode()

    # Spin ROS2 in a background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Tkinter main loop on main thread
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
