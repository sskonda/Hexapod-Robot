#!/usr/bin/env python3
"""
grid_graph.py

Pure-Python data structures and algorithms for a 2-ft tile grid maze graph.
No ROS2 imports — unit-testable standalone.

Tile size: 0.6096 m (2 ft exactly)
Coordinate convention:
  - Grid (i, j): i increases East, j increases North
  - Map (x, y):  x East, y North (standard ROS REP-103)
  - Cardinal directions: 'N', 'E', 'S', 'W'

Left-first DFS ordering (relative to incoming travel direction):
  Coming from S (heading N): try W first, then N, then E  (left = W)
  Coming from W (heading E): try N first, then E, then S  (left = N)
  Coming from N (heading S): try E first, then S, then W  (left = E)
  Coming from E (heading W): try S first, then W, then N  (left = S)
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TILE_SIZE_M: float = 0.6096  # 2 ft in metres

DIRECTIONS = ('N', 'E', 'S', 'W')

# Delta (di, dj) for each cardinal direction
DIRECTION_DELTA: Dict[str, Tuple[int, int]] = {
    'N': (0, 1),
    'E': (1, 0),
    'S': (0, -1),
    'W': (-1, 0),
}

OPPOSITE: Dict[str, str] = {'N': 'S', 'E': 'W', 'S': 'N', 'W': 'E'}

# Yaw angle (radians) for each cardinal direction in the ROS map frame.
DIRECTION_YAW: Dict[str, float] = {
    'N': math.pi / 2.0,
    'E': 0.0,
    'S': -math.pi / 2.0,
    'W': math.pi,
}

# Left-first branch ordering for DFS.
# Key = direction the robot is currently TRAVELLING (came FROM the opposite).
# Value = ordered list of directions to try next (left of travel first).
LEFT_FIRST_ORDER: Dict[str, List[str]] = {
    'N': ['W', 'N', 'E', 'S'],   # left=W, straight=N, right=E, back=S
    'E': ['N', 'E', 'S', 'W'],   # left=N, straight=E, right=S, back=W
    'S': ['E', 'S', 'W', 'N'],   # left=E, straight=S, right=W, back=N
    'W': ['S', 'W', 'N', 'E'],   # left=S, straight=W, right=N, back=E
}

# ---------------------------------------------------------------------------
# Node / Edge data classes
# ---------------------------------------------------------------------------

# Visit states
UNVISITED = 'unvisited'
VISITED = 'visited'
BACKTRACKED = 'backtracked'

# Node types
NODE_UNKNOWN = 'unknown'
NODE_CORRIDOR = 'corridor'   # exactly 2 open directions (one in, one out)
NODE_JUNCTION = 'junction'   # 3+ open directions
NODE_DEAD_END = 'dead_end'   # only 1 open direction
NODE_EXIT = 'exit'           # open space > 2×tile in ≥2 directions


@dataclass
class GridNode:
    grid_i: int                              # east grid coordinate
    grid_j: int                              # north grid coordinate
    x_m: float                              # map-frame x
    y_m: float                              # map-frame y
    walls: Dict[str, bool] = field(default_factory=dict)
    # True = wall present, False = open, missing = unobserved
    walls_observed: Dict[str, int] = field(default_factory=dict)
    # count of scans that agreed on the wall/open classification
    visit_state: str = UNVISITED
    node_type: str = NODE_UNKNOWN
    visit_order: int = -1                    # order in which the node was first visited

    @property
    def ij(self) -> Tuple[int, int]:
        return (self.grid_i, self.grid_j)


@dataclass
class GridEdge:
    from_ij: Tuple[int, int]
    to_ij: Tuple[int, int]
    direction: str          # direction of travel from from_ij to to_ij
    traversed: bool = False


# ---------------------------------------------------------------------------
# GridGraph
# ---------------------------------------------------------------------------

class GridGraph:
    def __init__(self, origin_x_m: float = 0.0, origin_y_m: float = 0.0):
        """
        origin_x_m, origin_y_m: map-frame coordinates of grid cell (0, 0).
        """
        self.origin_x_m = origin_x_m
        self.origin_y_m = origin_y_m
        self._nodes: Dict[Tuple[int, int], GridNode] = {}
        self._edges: Dict[Tuple[Tuple[int, int], Tuple[int, int]], GridEdge] = {}
        self._visit_counter = 0

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def ij_to_xy(self, i: int, j: int) -> Tuple[float, float]:
        x = self.origin_x_m + i * TILE_SIZE_M
        y = self.origin_y_m + j * TILE_SIZE_M
        return x, y

    def xy_to_ij(self, x_m: float, y_m: float) -> Tuple[int, int]:
        i = int(round((x_m - self.origin_x_m) / TILE_SIZE_M))
        j = int(round((y_m - self.origin_y_m) / TILE_SIZE_M))
        return i, j

    # ------------------------------------------------------------------
    # Node management
    # ------------------------------------------------------------------

    def get_or_create_node(self, i: int, j: int) -> GridNode:
        key = (i, j)
        if key not in self._nodes:
            x, y = self.ij_to_xy(i, j)
            self._nodes[key] = GridNode(grid_i=i, grid_j=j, x_m=x, y_m=y)
        return self._nodes[key]

    def get_node(self, i: int, j: int) -> Optional[GridNode]:
        return self._nodes.get((i, j))

    def all_nodes(self) -> List[GridNode]:
        return list(self._nodes.values())

    def mark_visited(self, i: int, j: int) -> GridNode:
        node = self.get_or_create_node(i, j)
        if node.visit_state == UNVISITED:
            node.visit_state = VISITED
            node.visit_order = self._visit_counter
            self._visit_counter += 1
        return node

    def mark_backtracked(self, i: int, j: int):
        node = self._nodes.get((i, j))
        if node is not None:
            node.visit_state = BACKTRACKED

    # ------------------------------------------------------------------
    # Wall classification
    # ------------------------------------------------------------------

    def record_walls(self, i: int, j: int, wall_readings: Dict[str, bool]):
        """
        wall_readings: dict mapping direction -> True (wall) / False (open).
        Updates node.walls and node.walls_observed counts.
        """
        node = self.get_or_create_node(i, j)
        for direction, is_wall in wall_readings.items():
            prev = node.walls.get(direction)
            if prev is None:
                node.walls[direction] = is_wall
                node.walls_observed[direction] = 1
            elif prev == is_wall:
                node.walls_observed[direction] = node.walls_observed.get(direction, 0) + 1
            else:
                # Disagreement — new observation overrides; reset count
                node.walls[direction] = is_wall
                node.walls_observed[direction] = 1
        node.node_type = self.classify_node(node)

    def open_directions(self, i: int, j: int) -> List[str]:
        """Return list of directions that have no wall (False in node.walls)."""
        node = self._nodes.get((i, j))
        if node is None:
            return []
        return [d for d in DIRECTIONS if node.walls.get(d) is False]

    def neighbor_ij(self, i: int, j: int, direction: str) -> Tuple[int, int]:
        di, dj = DIRECTION_DELTA[direction]
        return i + di, j + dj

    # ------------------------------------------------------------------
    # Node classification
    # ------------------------------------------------------------------

    def classify_node(self, node: GridNode) -> str:
        open_dirs = [d for d in DIRECTIONS if node.walls.get(d) is False]
        n_open = len(open_dirs)
        if n_open >= 3:
            return NODE_JUNCTION
        if n_open == 2:
            return NODE_CORRIDOR
        if n_open == 1:
            return NODE_DEAD_END
        return NODE_UNKNOWN

    def is_exit_candidate(self, node: GridNode, exit_threshold_m: float) -> bool:
        """
        Exit if open space > exit_threshold_m in ≥2 directions.
        Exit threshold default is 2×TILE_SIZE_M = 1.2192 m.
        wall.walls stores True/False only for directions where we have
        a definitive reading; the actual distance must be checked in the
        mission node via the scan.  This helper just checks if the node
        is typed as EXIT.
        """
        return node.node_type == NODE_EXIT

    def set_exit(self, i: int, j: int):
        node = self.get_or_create_node(i, j)
        node.node_type = NODE_EXIT

    # ------------------------------------------------------------------
    # Edge management
    # ------------------------------------------------------------------

    def add_edge(self, from_ij: Tuple[int, int], to_ij: Tuple[int, int], direction: str):
        key = (from_ij, to_ij)
        if key not in self._edges:
            self._edges[key] = GridEdge(from_ij=from_ij, to_ij=to_ij, direction=direction)

    def mark_edge_traversed(self, from_ij: Tuple[int, int], to_ij: Tuple[int, int]):
        key = (from_ij, to_ij)
        if key in self._edges:
            self._edges[key].traversed = True

    def all_edges(self) -> List[GridEdge]:
        return list(self._edges.values())

    # ------------------------------------------------------------------
    # DFS traversal helpers
    # ------------------------------------------------------------------

    def dfs_next_direction(
        self,
        current_ij: Tuple[int, int],
        travel_direction: Optional[str],
        allow_revisit: bool = False,
    ) -> Optional[str]:
        """
        Return the next direction to travel from current_ij using left-first DFS.

        travel_direction: the direction we just came from (None on first step).
        allow_revisit: if True, already-visited nodes are also candidates
                       (used during backtracking to reach an unvisited branch).
        Returns None if no candidate direction is available.
        """
        node = self._nodes.get(current_ij)
        if node is None:
            return None

        open_dirs = self.open_directions(*current_ij)
        if not open_dirs:
            return None

        # Build preference order
        if travel_direction is not None and travel_direction in LEFT_FIRST_ORDER:
            ordered = LEFT_FIRST_ORDER[travel_direction]
        else:
            # No prior direction — prefer N first
            ordered = LEFT_FIRST_ORDER.get('N', list(DIRECTIONS))

        back_dir = OPPOSITE.get(travel_direction, '') if travel_direction else ''

        # First pass: unvisited neighbours only (skip back direction)
        for d in ordered:
            if d == back_dir:
                continue
            if d not in open_dirs:
                continue
            ni, nj = self.neighbor_ij(*current_ij, d)
            neighbour = self._nodes.get((ni, nj))
            if neighbour is None or neighbour.visit_state == UNVISITED:
                return d

        if not allow_revisit:
            return None

        # Second pass: allow revisiting visited nodes (backtracking)
        for d in ordered:
            if d == back_dir:
                continue
            if d not in open_dirs:
                continue
            return d

        # Last resort: go back
        if back_dir in open_dirs:
            return back_dir

        return None

    def backtrack_path(
        self,
        from_ij: Tuple[int, int],
        travel_direction: Optional[str],
    ) -> Optional[List[Tuple[int, int]]]:
        """
        BFS to find the shortest path from from_ij to the nearest unvisited node
        reachable via known open edges.

        Returns list of (i, j) coordinates to follow (including from_ij),
        or None if no unvisited node is reachable.
        """
        # BFS
        queue: deque = deque()
        queue.append((from_ij, [from_ij]))
        visited_in_search: Set[Tuple[int, int]] = {from_ij}

        while queue:
            current, path = queue.popleft()
            node = self._nodes.get(current)
            if node is None:
                continue

            open_dirs = self.open_directions(*current)
            for d in open_dirs:
                ni, nj = self.neighbor_ij(*current, d)
                neighbour_ij = (ni, nj)
                if neighbour_ij in visited_in_search:
                    continue
                neighbour = self._nodes.get(neighbour_ij)
                if neighbour is None or neighbour.visit_state == UNVISITED:
                    return path + [neighbour_ij]
                visited_in_search.add(neighbour_ij)
                queue.append((neighbour_ij, path + [neighbour_ij]))

        return None  # Entire reachable graph exhausted

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    def to_json_dict(self) -> dict:
        nodes_list = []
        for node in self._nodes.values():
            nodes_list.append({
                'i': node.grid_i,
                'j': node.grid_j,
                'x_m': node.x_m,
                'y_m': node.y_m,
                'walls': node.walls,
                'walls_observed': node.walls_observed,
                'visit_state': node.visit_state,
                'node_type': node.node_type,
                'visit_order': node.visit_order,
            })
        edges_list = []
        for edge in self._edges.values():
            edges_list.append({
                'from_i': edge.from_ij[0],
                'from_j': edge.from_ij[1],
                'to_i': edge.to_ij[0],
                'to_j': edge.to_ij[1],
                'direction': edge.direction,
                'traversed': edge.traversed,
            })
        return {
            'origin_x_m': self.origin_x_m,
            'origin_y_m': self.origin_y_m,
            'tile_size_m': TILE_SIZE_M,
            'nodes': nodes_list,
            'edges': edges_list,
        }

    @classmethod
    def from_json_dict(cls, data: dict) -> 'GridGraph':
        g = cls(origin_x_m=data['origin_x_m'], origin_y_m=data['origin_y_m'])
        for nd in data.get('nodes', []):
            node = g.get_or_create_node(nd['i'], nd['j'])
            node.walls = nd.get('walls', {})
            node.walls_observed = nd.get('walls_observed', {})
            node.visit_state = nd.get('visit_state', UNVISITED)
            node.node_type = nd.get('node_type', NODE_UNKNOWN)
            node.visit_order = nd.get('visit_order', -1)
        for ed in data.get('edges', []):
            from_ij = (ed['from_i'], ed['from_j'])
            to_ij = (ed['to_i'], ed['to_j'])
            g.add_edge(from_ij, to_ij, ed['direction'])
            if ed.get('traversed'):
                g.mark_edge_traversed(from_ij, to_ij)
        return g


def save_graph_json(graph: GridGraph, path: str):
    with open(path, 'w') as fh:
        json.dump(graph.to_json_dict(), fh, indent=2)


def load_graph_json(path: str) -> GridGraph:
    with open(path, 'r') as fh:
        data = json.load(fh)
    return GridGraph.from_json_dict(data)
