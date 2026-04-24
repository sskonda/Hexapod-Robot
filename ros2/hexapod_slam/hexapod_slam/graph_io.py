"""
graph_io.py — JSON serialization / deserialization for GraphState.
"""

import json
import time
from typing import Any, Dict

from .graph_types import (
    EdgeState, GraphState, MazeCell, MazeEdge, MazeNode, NodeType, VisitState,
)


# ---------------------------------------------------------------------------
# Serialisation helpers
# ---------------------------------------------------------------------------

def _node_to_dict(node: MazeNode) -> Dict[str, Any]:
    return {
        "id": node.id,
        "x_m": node.x_m,
        "y_m": node.y_m,
        "yaw_rad": node.yaw_rad,
        "node_type": node.node_type.value,
        "visit_state": node.visit_state.value,
        "confidence": node.confidence,
        "observation_count": node.observation_count,
        "last_seen_time": node.last_seen_time,
        "connected_edge_ids": list(node.connected_edge_ids),
    }


def _edge_to_dict(edge: MazeEdge) -> Dict[str, Any]:
    return {
        "id": edge.id,
        "start_node_id": edge.start_node_id,
        "end_node_id": edge.end_node_id,
        "polyline_points": [list(p) for p in edge.polyline_points],
        "length_m": edge.length_m,
        "heading_rad": edge.heading_rad,
        "edge_state": edge.edge_state.value,
        "visit_count": edge.visit_count,
        "blocked_count": edge.blocked_count,
        "confidence": edge.confidence,
    }


def _cell_to_dict(cell: MazeCell) -> Dict[str, Any]:
    return {
        "grid_x": cell.grid_x,
        "grid_y": cell.grid_y,
        "center_x_m": cell.center_x_m,
        "center_y_m": cell.center_y_m,
        "confidence": cell.confidence,
        "observation_count": cell.observation_count,
        "visit_count": cell.visit_count,
        "last_seen_time": cell.last_seen_time,
        "wall_confidence_by_side": list(cell.wall_confidence_by_side),
        "open_confidence_by_side": list(cell.open_confidence_by_side),
        "associated_node_ids": list(cell.associated_node_ids),
    }


def graph_to_dict(graph: GraphState) -> Dict[str, Any]:
    """Serialise GraphState to a plain-Python dict."""
    return {
        "version": graph.version,
        "timestamp": graph.timestamp,
        "active_target_node_id": graph.active_target_node_id,
        "frontier_edges": list(graph.frontier_edges),
        "cell_size_m": graph.cell_size_m,
        "cell_origin_x_m": graph.cell_origin_x_m,
        "cell_origin_y_m": graph.cell_origin_y_m,
        "nodes": {str(nid): _node_to_dict(n) for nid, n in graph.nodes.items()},
        "edges": {str(eid): _edge_to_dict(e) for eid, e in graph.edges.items()},
        "cells": {key: _cell_to_dict(cell) for key, cell in graph.cells.items()},
    }


# ---------------------------------------------------------------------------
# Deserialisation helpers
# ---------------------------------------------------------------------------

def _node_from_dict(d: Dict[str, Any]) -> MazeNode:
    return MazeNode(
        id=int(d["id"]),
        x_m=float(d["x_m"]),
        y_m=float(d["y_m"]),
        yaw_rad=float(d.get("yaw_rad", 0.0)),
        node_type=NodeType(d.get("node_type", NodeType.FRONTIER.value)),
        visit_state=VisitState(d.get("visit_state", VisitState.UNSEEN.value)),
        confidence=float(d.get("confidence", 0.0)),
        observation_count=int(d.get("observation_count", 0)),
        last_seen_time=float(d.get("last_seen_time", time.time())),
        connected_edge_ids=list(d.get("connected_edge_ids", [])),
    )


def _edge_from_dict(d: Dict[str, Any]) -> MazeEdge:
    return MazeEdge(
        id=int(d["id"]),
        start_node_id=int(d["start_node_id"]),
        end_node_id=int(d["end_node_id"]),
        polyline_points=[list(p) for p in d.get("polyline_points", [])],
        length_m=float(d.get("length_m", 0.0)),
        heading_rad=float(d.get("heading_rad", 0.0)),
        edge_state=EdgeState(d.get("edge_state", EdgeState.UNEXPLORED.value)),
        visit_count=int(d.get("visit_count", 0)),
        blocked_count=int(d.get("blocked_count", 0)),
        confidence=float(d.get("confidence", 0.0)),
    )


def _cell_from_dict(d: Dict[str, Any]) -> MazeCell:
    return MazeCell(
        grid_x=int(d["grid_x"]),
        grid_y=int(d["grid_y"]),
        center_x_m=float(d["center_x_m"]),
        center_y_m=float(d["center_y_m"]),
        confidence=float(d.get("confidence", 0.0)),
        observation_count=int(d.get("observation_count", 0)),
        visit_count=int(d.get("visit_count", 0)),
        last_seen_time=float(d.get("last_seen_time", time.time())),
        wall_confidence_by_side=list(d.get("wall_confidence_by_side", [0.0, 0.0, 0.0, 0.0])),
        open_confidence_by_side=list(d.get("open_confidence_by_side", [0.0, 0.0, 0.0, 0.0])),
        associated_node_ids=list(d.get("associated_node_ids", [])),
    )


def dict_to_graph(data: Dict[str, Any]) -> GraphState:
    """Reconstruct a GraphState from a plain-Python dict."""
    nodes = {int(k): _node_from_dict(v) for k, v in data.get("nodes", {}).items()}
    edges = {int(k): _edge_from_dict(v) for k, v in data.get("edges", {}).items()}
    cells = {str(k): _cell_from_dict(v) for k, v in data.get("cells", {}).items()}
    return GraphState(
        nodes=nodes,
        edges=edges,
        cells=cells,
        frontier_edges=list(data.get("frontier_edges", [])),
        active_target_node_id=data.get("active_target_node_id"),
        cell_size_m=float(data.get("cell_size_m", 0.0)),
        cell_origin_x_m=float(data.get("cell_origin_x_m", 0.0)),
        cell_origin_y_m=float(data.get("cell_origin_y_m", 0.0)),
        version=int(data.get("version", 0)),
        timestamp=float(data.get("timestamp", time.time())),
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def graph_to_json(graph: GraphState) -> str:
    """Serialise GraphState to a JSON string."""
    return json.dumps(graph_to_dict(graph))


def json_to_graph(json_str: str) -> GraphState:
    """Reconstruct GraphState from a JSON string."""
    return dict_to_graph(json.loads(json_str))


def save_graph(graph: GraphState, path: str) -> None:
    """Write the GraphState to a JSON file."""
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(graph_to_json(graph))


def load_graph(path: str) -> GraphState:
    """Load a GraphState from a JSON file."""
    with open(path, "r", encoding="utf-8") as fh:
        return json_to_graph(fh.read())
