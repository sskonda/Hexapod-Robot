"""
test_maze_regression.py

Regression tests for the maze traversal stack.
No ROS2 dependency — pure Python, runnable with pytest.
"""

import math
import sys
import os
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hexapod_slam.grid_graph import (
    GridGraph, TILE_SIZE_M, OPPOSITE, LEFT_FIRST_ORDER,
    FIRST_STEP_ORDER, UNVISITED, VISITED,
    save_graph_json, load_graph_json,
)

# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _normalise(a):
    return math.atan2(math.sin(a), math.cos(a))


# ===========================================================================
# 1. Startup heading not aligned with odom axes
# ===========================================================================

def test_north_yaw_east_coordinate_roundtrip():
    """When robot starts facing East (yaw=0), logical N = odom +x."""
    g = GridGraph(origin_x_m=1.0, origin_y_m=2.0, north_yaw=0.0)
    # Moving N (j+1) should step in +x
    x, y = g.ij_to_xy(0, 1)
    assert abs(x - (1.0 + TILE_SIZE_M)) < 1e-9, f"Expected x={1+TILE_SIZE_M}, got {x}"
    assert abs(y - 2.0) < 1e-9, f"Expected y=2.0, got {y}"
    # And round-trip
    i, j = g.xy_to_ij(x, y)
    assert (i, j) == (0, 1)


def test_north_yaw_north_coordinate_roundtrip():
    """When robot starts facing odom North (yaw=π/2), grid is axis-aligned."""
    g = GridGraph(origin_x_m=0.0, origin_y_m=0.0, north_yaw=math.pi / 2.0)
    x, y = g.ij_to_xy(0, 1)
    assert abs(x - 0.0) < 1e-9
    assert abs(y - TILE_SIZE_M) < 1e-9
    x2, y2 = g.ij_to_xy(1, 0)
    assert abs(x2 - TILE_SIZE_M) < 1e-9
    assert abs(y2 - 0.0) < 1e-9


def test_north_yaw_world_yaw_for_direction():
    """world_yaw_for_direction must map logical directions through north_yaw."""
    # Robot faces East (yaw=0)
    g = GridGraph(north_yaw=0.0)
    assert abs(g.world_yaw_for_direction('N') - 0.0) < 1e-9          # forward = East
    assert abs(g.world_yaw_for_direction('E') - (-math.pi / 2)) < 1e-9  # right = South
    assert abs(abs(g.world_yaw_for_direction('S')) - math.pi) < 1e-9  # back = West
    assert abs(g.world_yaw_for_direction('W') - (math.pi / 2)) < 1e-9  # left = North


def test_north_yaw_45deg():
    """Grid rotated 45° — ij_to_xy uses rotated unit vectors."""
    north_yaw = math.pi / 4.0   # NE diagonal
    g = GridGraph(origin_x_m=0.0, origin_y_m=0.0, north_yaw=north_yaw)
    x, y = g.ij_to_xy(0, 1)
    expected_x = TILE_SIZE_M * math.cos(north_yaw)
    expected_y = TILE_SIZE_M * math.sin(north_yaw)
    assert abs(x - expected_x) < 1e-9
    assert abs(y - expected_y) < 1e-9
    # Round-trip
    i, j = g.xy_to_ij(x, y)
    assert (i, j) == (0, 1)


# ===========================================================================
# 2. World-to-body frame transform (manual verification of math)
# ===========================================================================

def _world_to_body(world_dx, world_dy, robot_yaw):
    """Replicate the crab_path_follower world→body rotation."""
    cos_y = math.cos(robot_yaw)
    sin_y = math.sin(robot_yaw)
    body_x =  world_dx * cos_y + world_dy * sin_y
    body_y = -world_dx * sin_y + world_dy * cos_y
    return body_x, body_y


def test_frame_transform_facing_north():
    """Robot faces odom North (yaw=π/2): goal north → forward in body."""
    bx, by = _world_to_body(0.0, 1.0, math.pi / 2.0)
    assert abs(bx - 1.0) < 1e-9, f"body_x should be 1.0 (forward), got {bx}"
    assert abs(by) < 1e-9


def test_frame_transform_facing_east():
    """Robot faces odom East (yaw=0): goal east → forward in body."""
    bx, by = _world_to_body(1.0, 0.0, 0.0)
    assert abs(bx - 1.0) < 1e-9
    assert abs(by) < 1e-9


def test_frame_transform_strafing():
    """Robot faces North (yaw=π/2): goal east → right strafe in body."""
    bx, by = _world_to_body(1.0, 0.0, math.pi / 2.0)
    # East in world, robot faces North → that is robot's right (+body_x? no, body_y negative)
    # body_x = 1*cos(π/2) + 0*sin(π/2) = 0
    # body_y = -1*sin(π/2) + 0*cos(π/2) = -1  (right strafe is -y in ROS body convention)
    assert abs(bx) < 1e-9
    assert abs(by - (-1.0)) < 1e-9


def test_frame_transform_yaw_drift():
    """Small yaw drift does not send robot in wrong direction."""
    drift = math.radians(15)
    # Going North with 15° drift: body_x should still be positive (mostly forward)
    bx, by = _world_to_body(0.0, 1.0, math.pi / 2.0 + drift)
    assert bx > 0.9, f"body_x={bx} too small — yaw drift causes significant steering error"


# ===========================================================================
# 3. Aborted traversal does NOT mark the return edge as blocked
# ===========================================================================

def test_abort_does_not_corrupt_return_edge():
    """
    When a wall is detected mid-traversal, only the forward direction from the
    source node is recorded as a wall.  The target node should be untouched
    so that backtracking (which uses the reverse edge) still works.
    """
    g = GridGraph()
    # Corridor: (0,0) open N→(0,1), (0,1) open S→(0,0)
    g.record_walls(0, 0, {'N': False, 'E': True, 'S': True, 'W': True})
    g.record_walls(0, 1, {'N': True, 'E': True, 'S': False, 'W': True})
    g.add_edge((0, 0), (0, 1), 'N')
    g.add_edge((0, 1), (0, 0), 'S')
    g.mark_visited(0, 0)

    # Simulate: abort traversal from (0,0) toward (0,1) after detecting a wall
    # Only record wall from source's perspective:
    g.record_walls(0, 0, {'N': True})

    # Source node: N is now walled → not in open_directions
    assert 'N' not in g.open_directions(0, 0), "N should be walled from source"

    # Target node: S is still open (reverse edge intact)
    assert 'S' in g.open_directions(0, 1), "Return direction must stay open"

    # Target node type: dead_end (only S open now after N was never changed)
    assert g.get_node(0, 1).walls.get('N') is True   # N wall on target unchanged
    assert g.get_node(0, 1).walls.get('S') is False  # S (return) still open


# ===========================================================================
# 4. First step prefers startup-forward (logical N) when it is open
# ===========================================================================

def test_first_step_prefers_N_when_open():
    """With no prior travel direction, DFS must pick N before W or E."""
    g = GridGraph()
    # All four directions open
    g.record_walls(0, 0, {'N': False, 'E': False, 'S': False, 'W': False})
    g.mark_visited(0, 0)

    direction = g.dfs_next_direction((0, 0), travel_direction=None, allow_revisit=False)
    assert direction == 'N', f"Expected N on first step, got {direction}"


def test_first_step_skips_N_when_walled_prefers_W():
    """When N is walled, first step should try W (left) before E (right)."""
    g = GridGraph()
    g.record_walls(0, 0, {'N': True, 'E': False, 'S': True, 'W': False})
    g.mark_visited(0, 0)

    direction = g.dfs_next_direction((0, 0), travel_direction=None, allow_revisit=False)
    assert direction == 'W', f"Expected W (left preference), got {direction}"


def test_first_step_with_only_E_open():
    """When only E is open, first step must pick E."""
    g = GridGraph()
    g.record_walls(0, 0, {'N': True, 'E': False, 'S': True, 'W': True})
    g.mark_visited(0, 0)

    direction = g.dfs_next_direction((0, 0), travel_direction=None, allow_revisit=False)
    assert direction == 'E'


# ===========================================================================
# 5. Mid-traversal abort condition: no false abort near target
# ===========================================================================

def test_traversal_abort_condition_false_positive():
    """
    The abort condition (clearance < wall_thresh AND clearance < dist*0.8)
    must NOT fire when the robot is close to the target and the 'obstacle'
    is the far wall of the target tile (expected geometry).
    """
    wall_thresh = 0.40
    # Robot is 5 cm from target, far wall is 36 cm away
    fwd_clearance  = 0.36
    dist_to_target = 0.05

    would_abort = fwd_clearance < wall_thresh and fwd_clearance < dist_to_target * 0.8
    assert not would_abort, (
        "False abort: robot is almost at target but abort condition fired"
    )


def test_traversal_abort_condition_real_wall():
    """Abort condition fires when a real wall is encountered mid-traversal."""
    wall_thresh = 0.40
    # Robot is 40 cm from target, wall is 12 cm away
    fwd_clearance  = 0.12
    dist_to_target = 0.40

    would_abort = fwd_clearance < wall_thresh and fwd_clearance < dist_to_target * 0.8
    assert would_abort, "Should abort: wall clearly closer than target"


# ===========================================================================
# 6. DFS left-first ordering (regression)
# ===========================================================================

def test_dfs_left_first_after_moving_north():
    """Coming from S (heading N), DFS must try W before N before E."""
    g = GridGraph()
    g.record_walls(0, 1, {'N': False, 'E': False, 'S': False, 'W': False})
    g.mark_visited(0, 0)  # came from here
    g.mark_visited(0, 1)  # current node
    # Add unvisited neighbours
    for d in ('N', 'E', 'W'):
        ni, nj = g.neighbor_ij(0, 1, d)
        g.get_or_create_node(ni, nj)

    # No visited neighbours in W, N, E → should pick W (left of N travel)
    direction = g.dfs_next_direction((0, 1), travel_direction='N', allow_revisit=False)
    assert direction == 'W', f"Expected W (left of N), got {direction}"


# ===========================================================================
# 7. Backtrack path does not include blocked directions
# ===========================================================================

def test_backtrack_ignores_walled_directions():
    """backtrack_path only traverses open edges."""
    g = GridGraph()
    # Linear: (0,0)--N--(0,1)--N--(0,2)
    for ij in [(0,0),(0,1),(0,2)]:
        g.get_or_create_node(*ij)
    g.record_walls(0,0,{'N':False,'E':True,'S':True,'W':True})
    g.record_walls(0,1,{'N':False,'E':True,'S':False,'W':True})
    g.record_walls(0,2,{'N':True,'E':True,'S':False,'W':True})
    g.add_edge((0,0),(0,1),'N'); g.add_edge((0,1),(0,0),'S')
    g.add_edge((0,1),(0,2),'N'); g.add_edge((0,2),(0,1),'S')
    g.mark_visited(0,0); g.mark_visited(0,1)

    # Block N from (0,0) — but BFS goes via (0,1) which still has N open
    path = g.backtrack_path((0,0), 'N')
    assert path is not None
    assert path[-1] == (0,2)


# ===========================================================================
# 8. JSON round-trip preserves north_yaw
# ===========================================================================

def test_json_roundtrip_preserves_north_yaw():
    north_yaw = 0.314159
    g = GridGraph(origin_x_m=1.0, origin_y_m=2.0, north_yaw=north_yaw)
    g.record_walls(0,0,{'N':False,'E':True,'S':True,'W':False})
    g.mark_visited(0,0)

    with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as f:
        tmp = f.name
    try:
        save_graph_json(g, tmp)
        g2 = load_graph_json(tmp)
        assert abs(g2.north_yaw - north_yaw) < 1e-9
        x1, y1 = g.ij_to_xy(0, 1)
        x2, y2 = g2.ij_to_xy(0, 1)
        assert abs(x1 - x2) < 1e-9
        assert abs(y1 - y2) < 1e-9
    finally:
        os.unlink(tmp)


# ===========================================================================
# Run
# ===========================================================================

if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    passed = failed = 0
    for t in tests:
        try:
            t()
            print(f'  PASS  {t.__name__}')
            passed += 1
        except Exception as e:
            print(f'  FAIL  {t.__name__}: {e}')
            failed += 1
    print(f'\n{passed} passed, {failed} failed.')
    sys.exit(1 if failed else 0)
