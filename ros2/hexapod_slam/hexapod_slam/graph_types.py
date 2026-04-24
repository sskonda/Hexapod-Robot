"""
graph_types.py — Shared data structures for the maze graph system.
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional


class NodeType(Enum):
    START = "START"
    CORNER = "CORNER"
    JUNCTION = "JUNCTION"
    DEAD_END = "DEAD_END"
    FRONTIER = "FRONTIER"
    GOAL_CANDIDATE = "GOAL_CANDIDATE"


class VisitState(Enum):
    UNSEEN = "UNSEEN"
    SEEN = "SEEN"
    VISITED = "VISITED"
    BLOCKED = "BLOCKED"


class EdgeState(Enum):
    UNEXPLORED = "UNEXPLORED"
    OPEN = "OPEN"
    VISITED = "VISITED"
    BLOCKED = "BLOCKED"


class CellSideState(Enum):
    UNKNOWN = "UNKNOWN"
    OPEN = "OPEN"
    WALL = "WALL"


@dataclass
class MazeNode:
    id: int
    x_m: float
    y_m: float
    yaw_rad: float = 0.0
    node_type: NodeType = NodeType.FRONTIER
    visit_state: VisitState = VisitState.UNSEEN
    confidence: float = 0.0
    observation_count: int = 0
    last_seen_time: float = field(default_factory=time.time)
    connected_edge_ids: List[int] = field(default_factory=list)


@dataclass
class MazeEdge:
    id: int
    start_node_id: int
    end_node_id: int
    polyline_points: List[List[float]] = field(default_factory=list)  # [[x,y], ...]
    length_m: float = 0.0
    heading_rad: float = 0.0
    edge_state: EdgeState = EdgeState.UNEXPLORED
    visit_count: int = 0
    blocked_count: int = 0
    confidence: float = 0.0


@dataclass
class MazeCell:
    grid_x: int
    grid_y: int
    center_x_m: float
    center_y_m: float
    confidence: float = 0.0
    observation_count: int = 0
    visit_count: int = 0
    last_seen_time: float = field(default_factory=time.time)
    wall_confidence_by_side: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    open_confidence_by_side: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    associated_node_ids: List[int] = field(default_factory=list)


@dataclass
class GraphState:
    nodes: Dict[int, MazeNode] = field(default_factory=dict)
    edges: Dict[int, MazeEdge] = field(default_factory=dict)
    cells: Dict[str, MazeCell] = field(default_factory=dict)
    frontier_edges: List[int] = field(default_factory=list)
    active_target_node_id: Optional[int] = None
    cell_size_m: float = 0.0
    cell_origin_x_m: float = 0.0
    cell_origin_y_m: float = 0.0
    version: int = 0
    timestamp: float = field(default_factory=time.time)

    def node_count(self) -> int:
        return len(self.nodes)

    def edge_count(self) -> int:
        return len(self.edges)

    def cell_count(self) -> int:
        return len(self.cells)

    def get_node_edges(self, node_id: int) -> List[MazeEdge]:
        """Return all edges connected to the given node."""
        node = self.nodes.get(node_id)
        if node is None:
            return []
        result = []
        for eid in node.connected_edge_ids:
            edge = self.edges.get(eid)
            if edge is not None:
                result.append(edge)
        return result

    def get_adjacent_nodes(self, node_id: int) -> List[MazeNode]:
        """Return all nodes reachable from node_id via a single edge."""
        result = []
        for edge in self.get_node_edges(node_id):
            if edge.start_node_id == node_id:
                other_id = edge.end_node_id
            else:
                other_id = edge.start_node_id
            other = self.nodes.get(other_id)
            if other is not None:
                result.append(other)
        return result
