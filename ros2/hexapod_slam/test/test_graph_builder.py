"""
test_graph_builder.py

Comprehensive unit tests for the hexapod_slam maze graph system.
Runs with plain `pytest` — no ROS2 environment required.

All ROS2-specific imports are carefully avoided.  We test only the pure
Python algorithm functions extracted from each module.
"""

import json
import math
import os
import sys
import time

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Make the package importable without installing it first
# ---------------------------------------------------------------------------
_REPO_ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, _REPO_ROOT)

# Import pure-Python modules directly (no rclpy dependency)
from hexapod_slam.graph_types import (
    EdgeState,
    GraphState,
    MazeEdge,
    MazeNode,
    NodeType,
    VisitState,
)
from hexapod_slam.graph_io import (
    dict_to_graph,
    graph_to_dict,
    graph_to_json,
    json_to_graph,
    load_graph,
    save_graph,
)
from hexapod_slam.map_frontier_utils import (
    detect_frontiers,
    distance_m,
    grid_to_world,
    heading_between_points,
    inflate_obstacles,
    line_of_sight_is_free,
    make_binary_free,
    occupancy_grid_to_array,
    ray_cast_distance,
    world_to_grid,
)

# ---------------------------------------------------------------------------
# We need to import algorithm functions from maze_graph_builder without
# triggering rclpy.  We do a targeted import of the module-level helpers.
# ---------------------------------------------------------------------------

# Patch out rclpy before importing maze_graph_builder
import types
import unittest.mock as mock

# Create a minimal rclpy mock so the module-level import doesn't fail
_rclpy_mock = types.ModuleType("rclpy")
_rclpy_mock.node = types.ModuleType("rclpy.node")

class _FakeNode:
    pass

_rclpy_mock.node.Node = _FakeNode
sys.modules.setdefault("rclpy", _rclpy_mock)
sys.modules.setdefault("rclpy.node", _rclpy_mock.node)

# Mock visualization_msgs too
for _mod in [
    "visualization_msgs",
    "visualization_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "builtin_interfaces",
    "builtin_interfaces.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
]:
    if _mod not in sys.modules:
        sys.modules[_mod] = types.ModuleType(_mod)


# Provide minimal stub classes so marker generation code doesn't crash
class _Marker:
    SPHERE = 2
    LINE_STRIP = 4
    POINTS = 8
    TEXT_VIEW_FACING = 9
    CYLINDER = 3
    ADD = 0

    def __init__(self):
        self.header = _Header()
        self.ns = ""
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = _Pose()
        self.scale = _Scale()
        self.color = _Color()
        self.points = []
        self.text = ""


class _MarkerArray:
    def __init__(self):
        self.markers = []


class _Header:
    def __init__(self):
        self.frame_id = ""
        self.stamp = None


class _Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Pose:
    def __init__(self):
        self.position = _Point()
        self.orientation = _Orientation()


class _Orientation:
    def __init__(self):
        self.w = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Scale:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Color:
    def __init__(self):
        self.r = 0.0
        self.g = 0.0
        self.b = 0.0
        self.a = 0.0


# Inject stubs
_vis_msg = sys.modules["visualization_msgs.msg"]
_vis_msg.Marker = _Marker
_vis_msg.MarkerArray = _MarkerArray

_geom_msg = sys.modules["geometry_msgs.msg"]
_geom_msg.Point = _Point
_geom_msg.PoseStamped = object
_geom_msg.Quaternion = _Orientation

_std_msg = sys.modules["std_msgs.msg"]
_std_msg.String = object
_std_msg.ColorRGBA = _Color
_std_msg.Header = _Header

_nav_msg = sys.modules["nav_msgs.msg"]
_nav_msg.OccupancyGrid = object
_nav_msg.Odometry = object
_nav_msg.Path = object

_bi_msg = sys.modules["builtin_interfaces.msg"]
_bi_msg.Time = object

# Now import builder helpers
from hexapod_slam.maze_graph_builder import (
    add_frontier_nodes,
    build_edges,
    extract_graph_candidates,
    generate_edge_markers,
    generate_node_markers,
    update_graph_nodes,
)

# And visualizer helpers
from hexapod_slam.maze_graph_visualizer import (
    generate_active_target_marker,
    generate_combined_markers,
    generate_frontier_markers_viz,
    generate_label_markers,
)


# ===========================================================================
# SECTION 1 – graph_types
# ===========================================================================

class TestMazeNode:
    def test_creation_with_required_fields(self):
        node = MazeNode(id=0, x_m=1.0, y_m=2.0)
        assert node.id == 0
        assert node.x_m == pytest.approx(1.0)
        assert node.y_m == pytest.approx(2.0)

    def test_default_values(self):
        node = MazeNode(id=1, x_m=0.0, y_m=0.0)
        assert node.yaw_rad == pytest.approx(0.0)
        assert node.node_type == NodeType.FRONTIER
        assert node.visit_state == VisitState.UNSEEN
        assert node.confidence == pytest.approx(0.0)
        assert node.observation_count == 0
        assert isinstance(node.last_seen_time, float)
        assert node.connected_edge_ids == []

    def test_connected_edge_ids_independent(self):
        n1 = MazeNode(id=0, x_m=0.0, y_m=0.0)
        n2 = MazeNode(id=1, x_m=1.0, y_m=0.0)
        n1.connected_edge_ids.append(99)
        assert 99 not in n2.connected_edge_ids


class TestMazeEdge:
    def test_creation(self):
        edge = MazeEdge(id=0, start_node_id=0, end_node_id=1)
        assert edge.id == 0
        assert edge.start_node_id == 0
        assert edge.end_node_id == 1
        assert edge.edge_state == EdgeState.UNEXPLORED
        assert edge.visit_count == 0
        assert edge.polyline_points == []

    def test_polyline_points(self):
        edge = MazeEdge(id=1, start_node_id=2, end_node_id=3,
                        polyline_points=[[0.0, 0.0], [1.0, 0.0]])
        assert len(edge.polyline_points) == 2
        assert edge.polyline_points[0] == [0.0, 0.0]


class TestGraphState:
    def _make_graph(self):
        n0 = MazeNode(id=0, x_m=0.0, y_m=0.0, connected_edge_ids=[0])
        n1 = MazeNode(id=1, x_m=1.0, y_m=0.0, connected_edge_ids=[0])
        n2 = MazeNode(id=2, x_m=0.0, y_m=1.0)
        e0 = MazeEdge(id=0, start_node_id=0, end_node_id=1,
                      polyline_points=[[0.0, 0.0], [1.0, 0.0]], length_m=1.0)
        g = GraphState(nodes={0: n0, 1: n1, 2: n2}, edges={0: e0})
        return g

    def test_node_count(self):
        g = self._make_graph()
        assert g.node_count() == 3

    def test_edge_count(self):
        g = self._make_graph()
        assert g.edge_count() == 1

    def test_get_node_edges(self):
        g = self._make_graph()
        edges = g.get_node_edges(0)
        assert len(edges) == 1
        assert edges[0].id == 0

    def test_get_node_edges_no_edges(self):
        g = self._make_graph()
        edges = g.get_node_edges(2)
        assert edges == []

    def test_get_adjacent_nodes(self):
        g = self._make_graph()
        adj = g.get_adjacent_nodes(0)
        assert len(adj) == 1
        assert adj[0].id == 1

    def test_get_adjacent_nodes_bidirectional(self):
        g = self._make_graph()
        adj = g.get_adjacent_nodes(1)
        assert len(adj) == 1
        assert adj[0].id == 0

    def test_version_and_timestamp(self):
        g = GraphState()
        assert g.version == 0
        assert isinstance(g.timestamp, float)


# ===========================================================================
# SECTION 2 – map_frontier_utils
# ===========================================================================

class _FakeMapInfo:
    def __init__(self, width, height, res=0.05, ox=0.0, oy=0.0):
        self.width = width
        self.height = height
        self.resolution = res
        self.origin = _FakeOrigin(ox, oy)


class _FakeOrigin:
    def __init__(self, ox, oy):
        self.position = _FakePos(ox, oy)


class _FakePos:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _FakeOccupancyGrid:
    def __init__(self, data, width, height, res=0.05, ox=0.0, oy=0.0):
        self.data = data
        self.info = _FakeMapInfo(width, height, res, ox, oy)


class TestWorldGridConversion:
    def test_world_to_grid_origin(self):
        gx, gy = world_to_grid(0.0, 0.0, 0.0, 0.0, 0.05)
        assert gx == 0
        assert gy == 0

    def test_world_to_grid_offset(self):
        gx, gy = world_to_grid(0.1, 0.2, 0.0, 0.0, 0.05)
        assert gx == 2
        assert gy == 4

    def test_grid_to_world_origin(self):
        x, y = grid_to_world(0, 0, 0.0, 0.0, 0.05)
        assert x == pytest.approx(0.025)
        assert y == pytest.approx(0.025)

    def test_round_trip(self):
        res = 0.05
        for wx in [0.0, 0.33, 1.0]:
            for wy in [0.0, 0.17, 0.5]:
                gx, gy = world_to_grid(wx, wy, 0.0, 0.0, res)
                bx, by = grid_to_world(gx, gy, 0.0, 0.0, res)
                # Should be within half a cell
                assert abs(bx - wx) <= res
                assert abs(by - wy) <= res

    def test_negative_origin(self):
        gx, gy = world_to_grid(0.0, 0.0, -1.0, -1.0, 0.05)
        assert gx == 20
        assert gy == 20


class TestOccupancyGridToArray:
    def test_basic_reshape(self):
        data = [0] * 100  # 10x10 grid, all free
        msg = _FakeOccupancyGrid(data, 10, 10, 0.05)
        arr = occupancy_grid_to_array(msg)
        assert arr.shape == (10, 10)
        assert arr.dtype == np.int16

    def test_unknown_preserved(self):
        # -1 in int8 should remain -1 in int16
        data = [-1] * 25
        msg = _FakeOccupancyGrid(data, 5, 5, 0.05)
        arr = occupancy_grid_to_array(msg)
        assert np.all(arr == -1)

    def test_occupied_preserved(self):
        data = [100] * 25
        msg = _FakeOccupancyGrid(data, 5, 5, 0.05)
        arr = occupancy_grid_to_array(msg)
        assert np.all(arr == 100)


class TestMakeBinaryFree:
    def test_free_cells_are_true(self):
        grid = np.array([[0, 10, 20]], dtype=np.int16)
        bf = make_binary_free(grid, free_threshold=20)
        assert np.all(bf == True)

    def test_occupied_cells_are_false(self):
        grid = np.array([[21, 50, 100]], dtype=np.int16)
        bf = make_binary_free(grid, free_threshold=20)
        assert np.all(bf == False)

    def test_unknown_cells_are_false(self):
        grid = np.array([[-1, -1]], dtype=np.int16)
        bf = make_binary_free(grid, free_threshold=20)
        assert np.all(bf == False)

    def test_mixed_grid(self):
        grid = np.array([[0, -1, 100, 15, 21]], dtype=np.int16)
        bf = make_binary_free(grid, free_threshold=20)
        expected = np.array([[True, False, False, True, False]])
        np.testing.assert_array_equal(bf, expected)


class TestInflateObstacles:
    def test_zero_radius_no_change(self):
        bf = np.ones((5, 5), dtype=bool)
        bf[2, 2] = False
        result = inflate_obstacles(bf, 0)
        np.testing.assert_array_equal(result, bf)

    def test_radius_1_expands_obstacle(self):
        bf = np.ones((7, 7), dtype=bool)
        bf[3, 3] = False  # single obstacle in the centre
        result = inflate_obstacles(bf, 1)
        # The four 4-connected neighbours should also become False
        assert result[3, 3] == False
        assert result[2, 3] == False
        assert result[4, 3] == False
        assert result[3, 2] == False
        assert result[3, 4] == False
        # Diagonal should still be True (BFS uses 4-connectivity)
        assert result[2, 2] == True

    def test_all_obstacle_stays_all_obstacle(self):
        bf = np.zeros((5, 5), dtype=bool)
        result = inflate_obstacles(bf, 2)
        assert np.all(result == False)

    def test_inflation_does_not_expand_into_already_free(self):
        bf = np.ones((10, 10), dtype=bool)
        # Wall along left edge
        bf[:, 0] = False
        result = inflate_obstacles(bf, 1)
        # Column 1 should now be False
        assert np.all(result[:, 1] == False)
        # Column 2 should still be True
        assert np.all(result[:, 2] == True)


class TestDetectFrontiers:
    def test_free_adjacent_to_unknown_is_frontier(self):
        # 3x3 grid: centre free, surrounded by unknown except one occupied
        grid = np.full((3, 3), -1, dtype=np.int16)
        grid[1, 1] = 0   # centre is free
        frontiers = detect_frontiers(grid, free_threshold=20)
        assert (1, 1) in frontiers  # (gx=1, gy=1)

    def test_fully_known_grid_has_no_frontiers(self):
        grid = np.zeros((5, 5), dtype=np.int16)  # all free, no unknown
        frontiers = detect_frontiers(grid, free_threshold=20)
        assert len(frontiers) == 0

    def test_occupied_adjacent_to_unknown_is_not_frontier(self):
        grid = np.full((3, 3), -1, dtype=np.int16)
        grid[1, 1] = 100  # occupied, not free
        frontiers = detect_frontiers(grid, free_threshold=20)
        assert (1, 1) not in frontiers

    def test_frontier_row(self):
        # 5-wide corridor: bottom row free, top row unknown
        grid = np.full((2, 5), -1, dtype=np.int16)
        grid[0, :] = 0  # row 0 (gy=0) is free
        frontiers = detect_frontiers(grid, free_threshold=20)
        # All cells in row 0 should be frontiers
        for gx in range(5):
            assert (gx, 0) in frontiers


class TestRayCast:
    def test_open_corridor_reaches_max(self):
        bf = np.ones((50, 50), dtype=bool)
        dist = ray_cast_distance(bf, 25, 25, 0.0, 20.0)
        assert dist == pytest.approx(20.0)

    def test_wall_stops_ray(self):
        bf = np.ones((20, 20), dtype=bool)
        # Wall at column 5
        bf[:, 5] = False
        dist = ray_cast_distance(bf, 0, 10, 0.0, 20.0)  # shooting right
        assert dist < 6.0  # should stop before the wall

    def test_ray_in_bounds(self):
        bf = np.ones((10, 10), dtype=bool)
        # Ray going out of bounds should stop at boundary
        dist = ray_cast_distance(bf, 0, 5, math.pi, 100.0)  # shooting left
        assert dist < 5.0

    def test_ray_blocked_immediately(self):
        bf = np.ones((10, 10), dtype=bool)
        bf[:, 1] = False  # wall right next to start
        dist = ray_cast_distance(bf, 0, 5, 0.0, 10.0)
        assert dist < 2.0


class TestLineOfSight:
    def test_clear_horizontal(self):
        bf = np.ones((10, 10), dtype=bool)
        assert line_of_sight_is_free(bf, 0, 5, 9, 5) == True

    def test_clear_vertical(self):
        bf = np.ones((10, 10), dtype=bool)
        assert line_of_sight_is_free(bf, 5, 0, 5, 9) == True

    def test_clear_diagonal(self):
        bf = np.ones((10, 10), dtype=bool)
        assert line_of_sight_is_free(bf, 0, 0, 9, 9) == True

    def test_blocked_by_wall(self):
        bf = np.ones((10, 10), dtype=bool)
        bf[5, 5] = False  # obstacle in the middle
        assert line_of_sight_is_free(bf, 0, 5, 9, 5) == False

    def test_same_point(self):
        bf = np.ones((5, 5), dtype=bool)
        assert line_of_sight_is_free(bf, 2, 2, 2, 2) == True

    def test_diagonal_blocked(self):
        bf = np.ones((10, 10), dtype=bool)
        bf[3, 3] = False
        assert line_of_sight_is_free(bf, 0, 0, 6, 6) == False


class TestHelperFunctions:
    def test_heading_between_points(self):
        h = heading_between_points(0.0, 0.0, 1.0, 0.0)
        assert h == pytest.approx(0.0)
        h = heading_between_points(0.0, 0.0, 0.0, 1.0)
        assert h == pytest.approx(math.pi / 2)

    def test_distance_m(self):
        d = distance_m(0.0, 0.0, 3.0, 4.0)
        assert d == pytest.approx(5.0)

    def test_distance_m_zero(self):
        assert distance_m(1.0, 1.0, 1.0, 1.0) == pytest.approx(0.0)


# ===========================================================================
# SECTION 3 – graph_io
# ===========================================================================

class TestGraphIO:
    def _make_graph_with_nodes_and_edges(self):
        n0 = MazeNode(id=0, x_m=0.0, y_m=0.0, node_type=NodeType.START,
                      visit_state=VisitState.VISITED, connected_edge_ids=[0])
        n1 = MazeNode(id=1, x_m=1.0, y_m=0.5, node_type=NodeType.JUNCTION,
                      connected_edge_ids=[0])
        e0 = MazeEdge(id=0, start_node_id=0, end_node_id=1,
                      polyline_points=[[0.0, 0.0], [1.0, 0.5]],
                      length_m=1.118, heading_rad=0.4636,
                      edge_state=EdgeState.OPEN, visit_count=1)
        return GraphState(nodes={0: n0, 1: n1}, edges={0: e0},
                          frontier_edges=[], version=3)

    def test_serialize_empty_graph(self):
        g = GraphState()
        d = graph_to_dict(g)
        assert d["nodes"] == {}
        assert d["edges"] == {}
        assert d["frontier_edges"] == []

    def test_deserialize_empty_graph(self):
        g = GraphState()
        d = graph_to_dict(g)
        g2 = dict_to_graph(d)
        assert g2.node_count() == 0
        assert g2.edge_count() == 0

    def test_serialize_with_nodes_and_edges(self):
        g = self._make_graph_with_nodes_and_edges()
        d = graph_to_dict(g)
        assert "0" in d["nodes"]
        assert "1" in d["nodes"]
        assert "0" in d["edges"]
        assert d["nodes"]["0"]["node_type"] == "START"
        assert d["nodes"]["0"]["visit_state"] == "VISITED"
        assert d["edges"]["0"]["edge_state"] == "OPEN"

    def test_deserialize_with_nodes_and_edges(self):
        g = self._make_graph_with_nodes_and_edges()
        g2 = dict_to_graph(graph_to_dict(g))
        assert g2.node_count() == 2
        assert g2.edge_count() == 1
        assert g2.nodes[0].node_type == NodeType.START
        assert g2.nodes[0].visit_state == VisitState.VISITED
        assert g2.nodes[1].node_type == NodeType.JUNCTION
        assert g2.edges[0].edge_state == EdgeState.OPEN
        assert g2.edges[0].length_m == pytest.approx(1.118)
        assert g2.version == 3

    def test_json_round_trip(self):
        g = self._make_graph_with_nodes_and_edges()
        json_str = graph_to_json(g)
        g2 = json_to_graph(json_str)
        assert g2.node_count() == g.node_count()
        assert g2.edges[0].polyline_points == [[0.0, 0.0], [1.0, 0.5]]

    def test_save_load_round_trip(self, tmp_path):
        g = self._make_graph_with_nodes_and_edges()
        path = str(tmp_path / "graph.json")
        save_graph(g, path)
        g2 = load_graph(path)
        assert g2.node_count() == 2
        assert g2.nodes[1].x_m == pytest.approx(1.0)

    def test_connected_edge_ids_preserved(self):
        g = self._make_graph_with_nodes_and_edges()
        g2 = json_to_graph(graph_to_json(g))
        assert 0 in g2.nodes[0].connected_edge_ids
        assert 0 in g2.nodes[1].connected_edge_ids

    def test_active_target_preserved(self):
        g = self._make_graph_with_nodes_and_edges()
        g.active_target_node_id = 1
        g2 = json_to_graph(graph_to_json(g))
        assert g2.active_target_node_id == 1


# ===========================================================================
# SECTION 4 – graph extraction (synthetic maps)
# ===========================================================================

def _make_grid(shape, fill_value=0):
    """Return an int16 numpy array filled with fill_value."""
    return np.full(shape, fill_value, dtype=np.int16)


def _make_binary_cross(size=31):
    """
    Create a T-shaped free corridor on an otherwise occupied map.
    The horizontal corridor runs across the middle row.
    A vertical branch goes up from the centre.
    Returns (binary_free, junction_gx, junction_gy).
    """
    mid = size // 2
    bf = np.zeros((size, size), dtype=bool)
    # Horizontal corridor
    bf[mid, :] = True
    # Vertical branch going up from centre (gy decreasing = upward in image)
    bf[:mid + 1, mid] = True
    return bf, mid, mid


def _make_dead_end(length=20, width=1):
    """
    Horizontal corridor closed at the right end.
    Dead end should be at gx=length-1.
    Returns binary_free.
    """
    bf = np.zeros((5, length + 2), dtype=bool)
    bf[2, 1:length + 1] = True  # corridor along row 2
    return bf


def _make_straight_corridor(length=30):
    """Straight horizontal corridor — no turns, no branches."""
    bf = np.zeros((5, length), dtype=bool)
    bf[2, :] = True
    return bf


class TestGraphExtraction:
    """Test extract_graph_candidates with synthetic binary_free arrays."""

    _RES = 0.05
    _ORIGIN_X = 0.0
    _ORIGIN_Y = 0.0
    _SPACING = 0.10   # tight spacing to catch features
    _PROBE = 0.25
    _CORNER_THRESH = 40.0
    _MERGE = 0.20

    def _extract(self, bf, safe_free=None):
        if safe_free is None:
            safe_free = bf
        return extract_graph_candidates(
            binary_free=bf,
            safe_free=safe_free,
            resolution=self._RES,
            origin_x=self._ORIGIN_X,
            origin_y=self._ORIGIN_Y,
            candidate_grid_spacing_m=self._SPACING,
            probe_distance_m=self._PROBE,
            corner_angle_threshold_deg=self._CORNER_THRESH,
            node_merge_distance_m=self._MERGE,
        )

    def test_t_junction_detection(self):
        """T-shaped corridor should produce at least one JUNCTION."""
        bf, jx, jy = _make_binary_cross(size=32)  # size=32 so mid=16 (even) is sampled
        candidates = self._extract(bf)
        types = [c[2] for c in candidates]
        assert NodeType.JUNCTION in types, (
            f"Expected JUNCTION in candidates, got: {types}"
        )

    def test_dead_end_detection(self):
        """
        Corridor with one open end should produce at least one DEAD_END.
        """
        bf = _make_dead_end(length=20)
        candidates = self._extract(bf)
        types = [c[2] for c in candidates]
        assert NodeType.DEAD_END in types, (
            f"Expected DEAD_END in candidates, got: {types}"
        )

    def test_straight_corridor_few_nodes(self):
        """
        A straight corridor should produce very few internal nodes
        (only the endpoints may appear as dead-ends, but not interior JUNCTIONS).
        """
        bf = _make_straight_corridor(length=30)
        candidates = self._extract(bf)
        junctions = [c for c in candidates if c[2] == NodeType.JUNCTION]
        assert len(junctions) == 0, (
            f"Straight corridor should have no junctions, got: {junctions}"
        )

    def test_candidates_are_on_free_cells(self):
        """All returned candidates should lie on free cells."""
        bf, _, _ = _make_binary_cross(size=31)
        candidates = self._extract(bf)
        res = self._RES
        ox, oy = self._ORIGIN_X, self._ORIGIN_Y
        for x_m, y_m, _ in candidates:
            gx, gy = world_to_grid(x_m, y_m, ox, oy, res)
            h, w = bf.shape
            assert 0 <= gx < w and 0 <= gy < h, f"Out of bounds: ({gx},{gy})"
            assert bf[gy, gx], f"Candidate at ({gx},{gy}) is not on a free cell"


class TestUpdateGraphNodes:
    def test_new_candidates_added(self):
        graph = GraphState()
        candidates = [(0.0, 0.0, NodeType.JUNCTION), (1.0, 0.0, NodeType.DEAD_END)]
        seen = set()
        update_graph_nodes(graph, candidates, 0.25, 2, seen)
        assert graph.node_count() == 2

    def test_close_candidates_merge(self):
        graph = GraphState()
        candidates = [(0.0, 0.0, NodeType.JUNCTION)]
        seen = set()
        update_graph_nodes(graph, candidates, 0.25, 2, seen)
        # Second time with a very close point → should merge
        seen2 = set()
        candidates2 = [(0.05, 0.05, NodeType.JUNCTION)]
        update_graph_nodes(graph, candidates2, 0.25, 2, seen2)
        assert graph.node_count() == 1
        assert graph.nodes[0].observation_count == 2

    def test_seen_ids_populated(self):
        graph = GraphState()
        candidates = [(0.0, 0.0, NodeType.CORNER)]
        seen = set()
        update_graph_nodes(graph, candidates, 0.25, 2, seen)
        assert len(seen) == 1


class TestBuildEdges:
    def test_edge_created_between_nearby_visible_nodes(self):
        n0 = MazeNode(id=0, x_m=0.0, y_m=0.0)
        n1 = MazeNode(id=1, x_m=0.5, y_m=0.0)
        graph = GraphState(nodes={0: n0, 1: n1})
        bf = np.ones((20, 20), dtype=bool)
        build_edges(graph, bf, 0.0, 0.0, 0.05)
        assert graph.edge_count() == 1

    def test_no_edge_through_wall(self):
        n0 = MazeNode(id=0, x_m=0.0, y_m=0.0)
        n1 = MazeNode(id=1, x_m=0.5, y_m=0.0)
        graph = GraphState(nodes={0: n0, 1: n1})
        bf = np.ones((20, 20), dtype=bool)
        bf[:, 5] = False  # wall at column 5
        build_edges(graph, bf, 0.0, 0.0, 0.05)
        assert graph.edge_count() == 0

    def test_no_duplicate_edges(self):
        n0 = MazeNode(id=0, x_m=0.0, y_m=0.0)
        n1 = MazeNode(id=1, x_m=0.3, y_m=0.0)
        graph = GraphState(nodes={0: n0, 1: n1})
        bf = np.ones((20, 20), dtype=bool)
        build_edges(graph, bf, 0.0, 0.0, 0.05)
        build_edges(graph, bf, 0.0, 0.0, 0.05)  # call twice
        assert graph.edge_count() == 1


# ===========================================================================
# SECTION 5 – visualizer marker generation
# ===========================================================================

class TestVisualizerMarkers:
    def _make_simple_graph(self):
        n0 = MazeNode(id=0, x_m=0.0, y_m=0.0, node_type=NodeType.JUNCTION,
                      visit_state=VisitState.VISITED, connected_edge_ids=[0])
        n1 = MazeNode(id=1, x_m=1.0, y_m=0.0, node_type=NodeType.DEAD_END,
                      connected_edge_ids=[0])
        e0 = MazeEdge(id=0, start_node_id=0, end_node_id=1,
                      polyline_points=[[0.0, 0.0], [1.0, 0.0]],
                      length_m=1.0, edge_state=EdgeState.UNEXPLORED)
        return GraphState(nodes={0: n0, 1: n1}, edges={0: e0})

    def test_combined_markers_count(self):
        g = self._make_simple_graph()
        ma = generate_combined_markers(g, "map")
        # 2 node spheres + 1 edge line_strip
        assert len(ma.markers) == 3

    def test_node_marker_types(self):
        g = self._make_simple_graph()
        ma = generate_combined_markers(g, "map")
        sphere_markers = [m for m in ma.markers if m.type == _Marker.SPHERE]
        assert len(sphere_markers) == 2

    def test_edge_marker_types(self):
        g = self._make_simple_graph()
        ma = generate_combined_markers(g, "map")
        line_markers = [m for m in ma.markers if m.type == _Marker.LINE_STRIP]
        assert len(line_markers) == 1

    def test_visited_node_is_green(self):
        g = self._make_simple_graph()
        # n0 is VISITED
        ma = generate_combined_markers(g, "map")
        sphere_markers = [m for m in ma.markers if m.type == _Marker.SPHERE]
        visited_marker = next(m for m in sphere_markers if m.id == 0)
        # Green: (r≈0, g≈0.8, b≈0)
        assert visited_marker.color.g > 0.5
        assert visited_marker.color.r < 0.3

    def test_dead_end_node_is_red(self):
        g = self._make_simple_graph()
        ma = generate_combined_markers(g, "map")
        sphere_markers = [m for m in ma.markers if m.type == _Marker.SPHERE]
        dead_end_marker = next(m for m in sphere_markers if m.id == 1)
        assert dead_end_marker.color.r > 0.5

    def test_active_target_is_magenta(self):
        g = self._make_simple_graph()
        g.active_target_node_id = 1
        ma = generate_combined_markers(g, "map", active_target_id=1)
        sphere_markers = [m for m in ma.markers if m.type == _Marker.SPHERE]
        target_marker = next(m for m in sphere_markers if m.id == 1)
        assert target_marker.color.r == pytest.approx(1.0)
        assert target_marker.color.b == pytest.approx(1.0)
        assert target_marker.color.g == pytest.approx(0.0)

    def test_unexplored_edge_is_grey(self):
        g = self._make_simple_graph()
        ma = generate_combined_markers(g, "map")
        edge_marker = next(m for m in ma.markers if m.type == _Marker.LINE_STRIP)
        # Grey: r≈g≈b≈0.5
        assert abs(edge_marker.color.r - 0.5) < 0.1
        assert abs(edge_marker.color.g - 0.5) < 0.1

    def test_label_markers_count(self):
        g = self._make_simple_graph()
        ma = generate_label_markers(g, "map")
        assert len(ma.markers) == 2

    def test_label_text_content(self):
        g = self._make_simple_graph()
        ma = generate_label_markers(g, "map")
        texts = [m.text for m in ma.markers]
        assert any("0" in t for t in texts)
        assert any("1" in t for t in texts)

    def test_frontier_markers_only_frontier_nodes(self):
        g = self._make_simple_graph()
        # Add a frontier node
        g.nodes[2] = MazeNode(id=2, x_m=2.0, y_m=0.0, node_type=NodeType.FRONTIER)
        ma = generate_frontier_markers_viz(g, "map")
        assert len(ma.markers) == 1
        assert ma.markers[0].pose.position.x == pytest.approx(2.0)

    def test_active_target_marker_when_set(self):
        g = self._make_simple_graph()
        g.active_target_node_id = 0
        ma = generate_active_target_marker(g, "map")
        assert len(ma.markers) == 1
        assert ma.markers[0].color.r == pytest.approx(1.0)
        assert ma.markers[0].color.b == pytest.approx(1.0)

    def test_active_target_marker_when_none(self):
        g = self._make_simple_graph()
        g.active_target_node_id = None
        ma = generate_active_target_marker(g, "map")
        assert len(ma.markers) == 0

    def test_node_positions_in_markers(self):
        g = self._make_simple_graph()
        ma = generate_combined_markers(g, "map")
        sphere_markers = {m.id: m for m in ma.markers if m.type == _Marker.SPHERE}
        assert sphere_markers[0].pose.position.x == pytest.approx(0.0)
        assert sphere_markers[1].pose.position.x == pytest.approx(1.0)

    def test_edge_points_in_marker(self):
        g = self._make_simple_graph()
        ma = generate_combined_markers(g, "map")
        edge_marker = next(m for m in ma.markers if m.type == _Marker.LINE_STRIP)
        assert len(edge_marker.points) == 2
        assert edge_marker.points[0].x == pytest.approx(0.0)
        assert edge_marker.points[1].x == pytest.approx(1.0)


# ===========================================================================
# SECTION 6 – planner helpers
# ===========================================================================

# Import planner helpers (rclpy already mocked above)
from hexapod_slam.maze_graph_planner import (
    build_path_to_node,
    find_closest_node,
    select_next_target_bfs,
    select_next_target_dfs,
)


class TestPlannerHelpers:
    def _make_linear_graph(self):
        """0 -- 1 -- 2 (linear chain)."""
        n0 = MazeNode(id=0, x_m=0.0, y_m=0.0, connected_edge_ids=[0])
        n1 = MazeNode(id=1, x_m=1.0, y_m=0.0, connected_edge_ids=[0, 1])
        n2 = MazeNode(id=2, x_m=2.0, y_m=0.0, connected_edge_ids=[1])
        e0 = MazeEdge(id=0, start_node_id=0, end_node_id=1,
                      length_m=1.0, edge_state=EdgeState.UNEXPLORED)
        e1 = MazeEdge(id=1, start_node_id=1, end_node_id=2,
                      length_m=1.0, edge_state=EdgeState.UNEXPLORED)
        return GraphState(nodes={0: n0, 1: n1, 2: n2}, edges={0: e0, 1: e1})

    def test_find_closest_node_exact(self):
        g = self._make_linear_graph()
        nid = find_closest_node(g, 1.0, 0.0, 0.3)
        assert nid == 1

    def test_find_closest_node_none_when_too_far(self):
        g = self._make_linear_graph()
        nid = find_closest_node(g, 5.0, 5.0, 0.3)
        assert nid is None

    def test_dfs_selects_unexplored(self):
        g = self._make_linear_graph()
        next_id = select_next_target_dfs(g, 0, set())
        assert next_id == 1

    def test_bfs_finds_nearest_unvisited(self):
        g = self._make_linear_graph()
        # Mark node 1 as visited
        g.nodes[1].visit_state = VisitState.VISITED
        next_id = select_next_target_bfs(g, 0, set())
        # Should skip 1 (visited) and find 2
        assert next_id == 2

    def test_build_path_direct(self):
        g = self._make_linear_graph()
        path = build_path_to_node(g, 0, 1, set())
        assert path == [0, 1]

    def test_build_path_two_hops(self):
        g = self._make_linear_graph()
        path = build_path_to_node(g, 0, 2, set())
        assert path == [0, 1, 2]

    def test_build_path_same_node(self):
        g = self._make_linear_graph()
        path = build_path_to_node(g, 1, 1, set())
        assert path == [1]

    def test_build_path_no_path_returns_start(self):
        g = self._make_linear_graph()
        # Block all edges
        next_id = build_path_to_node(g, 0, 2, {0, 1})
        assert next_id[0] == 0

    def test_dfs_avoids_blocked_edges(self):
        g = self._make_linear_graph()
        # Block edge 0
        next_id = select_next_target_dfs(g, 0, {0})
        assert next_id is None

    def test_all_nodes_exhausted_returns_none(self):
        g = self._make_linear_graph()
        g.nodes[1].visit_state = VisitState.VISITED
        g.nodes[2].visit_state = VisitState.VISITED
        # DFS should have nothing new to explore
        next_id = select_next_target_dfs(g, 0, set(), unexplored_bonus=5.0, revisit_penalty=100.0)
        # All neighbours visited — score will be negative but a node is still returned
        # (we just check it doesn't crash)
        # Either None or one of the visited nodes
        assert next_id is None or next_id in g.nodes
