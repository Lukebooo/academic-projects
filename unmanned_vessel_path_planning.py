"""
Sustainable Ocean Intelligent Autonomous Monitoring - Unmanned Vessel Path Planning
Shanghai Jiao Tong University Academic Project Simulation Code
"""
import heapq
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import geopandas as gpd
from shapely.geometry import Point

class WaterDomainGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=int)

    def add_obstacle_box(self, x_min, y_min, x_max, y_max):
        self.grid[y_min:y_max, x_min:x_max] = 1

    def is_navigable(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.grid[y, x] == 0
        return False

class VesselPathPlanner:
    def __init__(self, water_grid):
        self.grid = water_grid

    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        for dx, dy, cost in moves:
            nx, ny = x + dx, y + dy
            if self.grid.is_navigable(nx, ny):
                neighbors.append(((nx, ny), cost))
        return neighbors

    def a_star(self, start, goal):
        def heuristic(p1, p2):
            return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        pq = [(0 + heuristic(start, goal), 0, start)]
        costs = {start: 0}
        parent = {start: None}
        while pq:
            _, current_cost, current = heapq.heappop(pq)
            if current == goal:
                break
            for neighbor, step_cost in self.get_neighbors(current):
                new_cost = current_cost + step_cost
                if neighbor not in costs or new_cost < costs[neighbor]:
                    costs[neighbor] = new_cost
                    parent[neighbor] = current
                    priority = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(pq, (priority, new_cost, neighbor))
        return self._reconstruct_path(parent, start, goal)

    def dijkstra(self, start, goal):
        pq = [(0, start)]
        costs = {start: 0}
        parent = {start: None}
        while pq:
            current_cost, current = heapq.heappop(pq)
            if current == goal:
                break
            for neighbor, step_cost in self.get_neighbors(current):
                new_cost = current_cost + step_cost
                if neighbor not in costs or new_cost < costs[neighbor]:
                    costs[neighbor] = new_cost
                    parent[neighbor] = current
                    heapq.heappush(pq, (new_cost, neighbor))
        return self._reconstruct_path(parent, start, goal)

    def bfs(self, start, goal):
        queue = deque([start])
        parent = {start: None}
        while queue:
            current = queue.popleft()
            if current == goal:
                break
            for neighbor, _ in self.get_neighbors(current):
                if neighbor not in parent:
                    parent[neighbor] = current
                    queue.append(neighbor)
        return self._reconstruct_path(parent, start, goal)

    def _reconstruct_path(self, parent, start, goal):
        if goal not in parent:
            return []
        path = []
        curr = goal
        while curr is not None:
            path.append(curr)
            curr = parent[curr]
        return path[::-1]

class SensorDecisionSystem:
    def __init__(self, key_waypoints):
        self.waypoints = key_waypoints
        self.num_states = len(key_waypoints)

    def construct_transition_matrix(self, camera_risk_scores, sonar_obstacle_dists):
        transition_matrix = np.full((self.num_states, self.num_states), np.inf)
        for i in range(self.num_states):
            for j in range(self.num_states):
                if i != j:
                    dist = np.linalg.norm(np.array(self.waypoints[i]) - np.array(self.waypoints[j]))
                    camera_risk = camera_risk_scores.get((i, j), 1.0)
                    sonar_factor = 1.0 / max(sonar_obstacle_dists.get((i, j), 10.0), 0.1)
                    transition_matrix[i, j] = dist * camera_risk + sonar_factor * 5.0
        return transition_matrix

if __name__ == "__main__":
    print("=== SJTU Unmanned Vessel Autonomous Monitoring Demo ===")
    domain = WaterDomainGrid(50, 50)
    domain.add_obstacle_box(15, 10, 25, 40)
    domain.add_obstacle_box(30, 0, 38, 30)
    planner = VesselPathPlanner(domain)
    start_node, goal_node = (5, 5), (45, 42)
    path_astar = planner.a_star(start_node, goal_node)
    print("A* Path length:", len(path_astar))
    vessel_locations = [Point(x, y) for x, y in path_astar[::5]]
    gdf = gpd.GeoDataFrame({"step": range(len(vessel_locations))}, geometry=vessel_locations)
    print("GeoPandas Trajectory Sample:")
    print(gdf.head())
    key_states = [start_node, (10, 25), (28, 38), goal_node]
    decision_sys = SensorDecisionSystem(key_states)
    cam_risk = {(0, 1): 1.1, (1, 2): 1.5, (2, 3): 1.0}
    sonar_dist = {(0, 1): 8.0, (1, 2): 2.5, (2, 3): 12.0}
    t_matrix = decision_sys.construct_transition_matrix(cam_risk, sonar_dist)
    print("State Transition Matrix:")
    print(np.round(t_matrix, 2))