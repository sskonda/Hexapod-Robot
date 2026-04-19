"""
maze_cell_memory.py -- Quantized cell-memory helpers built on the SLAM map.
"""

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

from .graph_types import CellSideState, GraphState, MazeCell
from .map_frontier_utils import world_to_grid

SIDE_NAMES = ("north", "east", "south", "west")
SIDE_NORMALS = (
    (0.0, 1.0),
    (1.0, 0.0),
    (0.0, -1.0),
    (-1.0, 0.0),
)
SIDE_TANGENTS = (
    (1.0, 0.0),
    (0.0, 1.0),
    (1.0, 0.0),
    (0.0, 1.0),
)


@dataclass(frozen=True)
class CellObservation:
    grid_x: int
    grid_y: int
    center_x_m: float
    center_y_m: float
    center_free_fraction: float
    center_known_fraction: float
    wall_ratio_by_side: Tuple[float, float, float, float]
    open_ratio_by_side: Tuple[float, float, float, float]
    known_ratio_by_side: Tuple[float, float, float, float]


def cell_key(grid_x: int, grid_y: int) -> str:
    return f"{grid_x},{grid_y}"


def parse_cell_key(key: str) -> Tuple[int, int]:
    parts = key.split(",", maxsplit=1)
    return int(parts[0]), int(parts[1])


def cell_indices_from_world(
    x_m: float,
    y_m: float,
    cell_origin_x_m: float,
    cell_origin_y_m: float,
    cell_size_m: float,
) -> Tuple[int, int]:
    return (
        int(math.floor((x_m - cell_origin_x_m) / cell_size_m)),
        int(math.floor((y_m - cell_origin_y_m) / cell_size_m)),
    )


def cell_center_from_indices(
    grid_x: int,
    grid_y: int,
    cell_origin_x_m: float,
    cell_origin_y_m: float,
    cell_size_m: float,
) -> Tuple[float, float]:
    return (
        cell_origin_x_m + (grid_x + 0.5) * cell_size_m,
        cell_origin_y_m + (grid_y + 0.5) * cell_size_m,
    )


def classify_cell_side(
    cell: MazeCell,
    side_index: int,
    commit_confidence: float,
    dominance_margin: float,
) -> CellSideState:
    wall_conf = cell.wall_confidence_by_side[side_index]
    open_conf = cell.open_confidence_by_side[side_index]
    if wall_conf >= commit_confidence and wall_conf >= open_conf + dominance_margin:
        return CellSideState.WALL
    if open_conf >= commit_confidence and open_conf >= wall_conf + dominance_margin:
        return CellSideState.OPEN
    return CellSideState.UNKNOWN


def cell_wall_segment(
    cell: MazeCell,
    side_index: int,
    cell_size_m: float,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    half = 0.5 * cell_size_m
    if side_index == 0:  # north
        return (
            (cell.center_x_m - half, cell.center_y_m + half),
            (cell.center_x_m + half, cell.center_y_m + half),
        )
    if side_index == 1:  # east
        return (
            (cell.center_x_m + half, cell.center_y_m - half),
            (cell.center_x_m + half, cell.center_y_m + half),
        )
    if side_index == 2:  # south
        return (
            (cell.center_x_m - half, cell.center_y_m - half),
            (cell.center_x_m + half, cell.center_y_m - half),
        )
    return (
        (cell.center_x_m - half, cell.center_y_m - half),
        (cell.center_x_m - half, cell.center_y_m + half),
    )


def _map_value_at_world(
    grid_array: np.ndarray,
    origin_x_m: float,
    origin_y_m: float,
    resolution_m: float,
    x_m: float,
    y_m: float,
) -> Optional[int]:
    gx, gy = world_to_grid(x_m, y_m, origin_x_m, origin_y_m, resolution_m)
    height, width = grid_array.shape
    if not (0 <= gx < width and 0 <= gy < height):
        return None
    return int(grid_array[gy, gx])


def _sample_offsets(max_offset_m: float, step_m: float) -> Iterable[float]:
    if max_offset_m <= 0.0 or step_m <= 0.0:
        return [0.0]
    step_count = max(1, int(math.ceil(max_offset_m / step_m)))
    offsets = [0.0]
    for step_index in range(1, step_count + 1):
        offset = min(max_offset_m, step_index * step_m)
        offsets.extend([-offset, offset])
    return sorted(set(offsets))


def _sample_disc_ratios(
    grid_array: np.ndarray,
    origin_x_m: float,
    origin_y_m: float,
    resolution_m: float,
    center_x_m: float,
    center_y_m: float,
    radius_m: float,
    step_m: float,
    free_threshold: int,
    occupied_threshold: int,
) -> Tuple[float, float, float]:
    total_samples = 0
    known_samples = 0
    free_samples = 0
    occupied_samples = 0

    for offset_x_m in _sample_offsets(radius_m, step_m):
        for offset_y_m in _sample_offsets(radius_m, step_m):
            if offset_x_m * offset_x_m + offset_y_m * offset_y_m > radius_m * radius_m:
                continue
            total_samples += 1
            value = _map_value_at_world(
                grid_array,
                origin_x_m,
                origin_y_m,
                resolution_m,
                center_x_m + offset_x_m,
                center_y_m + offset_y_m,
            )
            if value is None or value < 0:
                continue
            known_samples += 1
            if value <= free_threshold:
                free_samples += 1
            elif value >= occupied_threshold:
                occupied_samples += 1

    if total_samples <= 0:
        return 0.0, 0.0, 0.0
    known_fraction = known_samples / total_samples
    free_fraction = free_samples / total_samples
    occupied_fraction = occupied_samples / total_samples
    return free_fraction, known_fraction, occupied_fraction


def _sample_side_band_ratios(
    grid_array: np.ndarray,
    origin_x_m: float,
    origin_y_m: float,
    resolution_m: float,
    center_x_m: float,
    center_y_m: float,
    side_index: int,
    cell_size_m: float,
    tangential_margin_m: float,
    spacing_m: float,
    band_half_width_m: float,
    free_threshold: int,
    occupied_threshold: int,
) -> Tuple[float, float, float]:
    normal_x, normal_y = SIDE_NORMALS[side_index]
    tangent_x, tangent_y = SIDE_TANGENTS[side_index]
    half_size_m = 0.5 * cell_size_m
    tangent_half_extent_m = max(spacing_m, half_size_m - tangential_margin_m)
    boundary_center_x_m = center_x_m + normal_x * half_size_m
    boundary_center_y_m = center_y_m + normal_y * half_size_m

    total_samples = 0
    known_samples = 0
    free_samples = 0
    occupied_samples = 0

    for tangent_offset_m in _sample_offsets(tangent_half_extent_m, spacing_m):
        for normal_offset_m in _sample_offsets(band_half_width_m, spacing_m):
            sample_x_m = (
                boundary_center_x_m
                + tangent_x * tangent_offset_m
                + normal_x * normal_offset_m
            )
            sample_y_m = (
                boundary_center_y_m
                + tangent_y * tangent_offset_m
                + normal_y * normal_offset_m
            )
            total_samples += 1
            value = _map_value_at_world(
                grid_array,
                origin_x_m,
                origin_y_m,
                resolution_m,
                sample_x_m,
                sample_y_m,
            )
            if value is None or value < 0:
                continue
            known_samples += 1
            if value <= free_threshold:
                free_samples += 1
            elif value >= occupied_threshold:
                occupied_samples += 1

    if total_samples <= 0:
        return 0.0, 0.0, 0.0
    wall_ratio = occupied_samples / total_samples
    open_ratio = free_samples / total_samples
    known_ratio = known_samples / total_samples
    return wall_ratio, open_ratio, known_ratio


def extract_cell_observations(
    grid_array: np.ndarray,
    origin_x_m: float,
    origin_y_m: float,
    resolution_m: float,
    free_threshold: int,
    occupied_threshold: int,
    cell_size_m: float,
    cell_origin_x_m: float,
    cell_origin_y_m: float,
    center_probe_radius_m: float,
    center_probe_step_m: float,
    min_center_free_fraction: float,
    min_center_known_fraction: float,
    side_sample_spacing_m: float,
    side_band_half_width_m: float,
    side_tangential_margin_m: float,
) -> Dict[str, CellObservation]:
    if grid_array.size == 0 or resolution_m <= 0.0 or cell_size_m <= 0.0:
        return {}

    height, width = grid_array.shape
    min_x_m = origin_x_m
    max_x_m = origin_x_m + width * resolution_m
    min_y_m = origin_y_m
    max_y_m = origin_y_m + height * resolution_m

    min_cell_x = int(math.floor((min_x_m - cell_origin_x_m) / cell_size_m)) - 1
    max_cell_x = int(math.ceil((max_x_m - cell_origin_x_m) / cell_size_m)) + 1
    min_cell_y = int(math.floor((min_y_m - cell_origin_y_m) / cell_size_m)) - 1
    max_cell_y = int(math.ceil((max_y_m - cell_origin_y_m) / cell_size_m)) + 1

    observations: Dict[str, CellObservation] = {}
    for grid_x in range(min_cell_x, max_cell_x + 1):
        for grid_y in range(min_cell_y, max_cell_y + 1):
            center_x_m, center_y_m = cell_center_from_indices(
                grid_x,
                grid_y,
                cell_origin_x_m,
                cell_origin_y_m,
                cell_size_m,
            )
            if (
                center_x_m < min_x_m
                or center_x_m > max_x_m
                or center_y_m < min_y_m
                or center_y_m > max_y_m
            ):
                continue

            center_free_fraction, center_known_fraction, _ = _sample_disc_ratios(
                grid_array,
                origin_x_m,
                origin_y_m,
                resolution_m,
                center_x_m,
                center_y_m,
                center_probe_radius_m,
                center_probe_step_m,
                free_threshold,
                occupied_threshold,
            )
            if center_known_fraction < min_center_known_fraction:
                continue
            if center_free_fraction < min_center_free_fraction:
                continue

            wall_ratios: List[float] = []
            open_ratios: List[float] = []
            known_ratios: List[float] = []
            for side_index in range(4):
                wall_ratio, open_ratio, known_ratio = _sample_side_band_ratios(
                    grid_array,
                    origin_x_m,
                    origin_y_m,
                    resolution_m,
                    center_x_m,
                    center_y_m,
                    side_index,
                    cell_size_m,
                    side_tangential_margin_m,
                    side_sample_spacing_m,
                    side_band_half_width_m,
                    free_threshold,
                    occupied_threshold,
                )
                wall_ratios.append(wall_ratio)
                open_ratios.append(open_ratio)
                known_ratios.append(known_ratio)

            observation = CellObservation(
                grid_x=grid_x,
                grid_y=grid_y,
                center_x_m=center_x_m,
                center_y_m=center_y_m,
                center_free_fraction=center_free_fraction,
                center_known_fraction=center_known_fraction,
                wall_ratio_by_side=tuple(wall_ratios),
                open_ratio_by_side=tuple(open_ratios),
                known_ratio_by_side=tuple(known_ratios),
            )
            observations[cell_key(grid_x, grid_y)] = observation

    return observations


def update_cell_memory(
    graph: GraphState,
    observations: Dict[str, CellObservation],
    now_sec: float,
    min_side_known_fraction: float,
    wall_presence_threshold: float,
    open_presence_threshold: float,
    observation_gain: float,
    wall_gain: float,
    open_gain: float,
    side_decay: float,
) -> None:
    for key, observation in observations.items():
        cell = graph.cells.get(key)
        if cell is None:
            cell = MazeCell(
                grid_x=observation.grid_x,
                grid_y=observation.grid_y,
                center_x_m=observation.center_x_m,
                center_y_m=observation.center_y_m,
            )
            graph.cells[key] = cell

        cell.center_x_m = observation.center_x_m
        cell.center_y_m = observation.center_y_m
        cell.confidence = min(
            1.0,
            max(cell.confidence, 0.0) + observation_gain * observation.center_free_fraction,
        )
        cell.observation_count += 1
        cell.last_seen_time = now_sec

        for side_index in range(4):
            if observation.known_ratio_by_side[side_index] < min_side_known_fraction:
                continue

            wall_score = observation.wall_ratio_by_side[side_index]
            open_score = observation.open_ratio_by_side[side_index]

            cell.wall_confidence_by_side[side_index] = max(
                0.0,
                cell.wall_confidence_by_side[side_index] - side_decay,
            )
            cell.open_confidence_by_side[side_index] = max(
                0.0,
                cell.open_confidence_by_side[side_index] - side_decay,
            )

            if wall_score >= wall_presence_threshold and wall_score >= open_score:
                cell.wall_confidence_by_side[side_index] = min(
                    1.0,
                    cell.wall_confidence_by_side[side_index] + wall_gain * wall_score,
                )
            elif open_score >= open_presence_threshold and open_score > wall_score:
                cell.open_confidence_by_side[side_index] = min(
                    1.0,
                    cell.open_confidence_by_side[side_index] + open_gain * open_score,
                )


def associate_nodes_to_cells(
    graph: GraphState,
    cell_origin_x_m: float,
    cell_origin_y_m: float,
    cell_size_m: float,
    association_radius_m: float,
) -> None:
    for cell in graph.cells.values():
        cell.associated_node_ids = []

    if cell_size_m <= 0.0:
        return

    for node in graph.nodes.values():
        base_grid_x, base_grid_y = cell_indices_from_world(
            node.x_m,
            node.y_m,
            cell_origin_x_m,
            cell_origin_y_m,
            cell_size_m,
        )
        best_key = None
        best_dist = association_radius_m
        for candidate_grid_x in range(base_grid_x - 1, base_grid_x + 2):
            for candidate_grid_y in range(base_grid_y - 1, base_grid_y + 2):
                candidate_key = cell_key(candidate_grid_x, candidate_grid_y)
                candidate_cell = graph.cells.get(candidate_key)
                if candidate_cell is None:
                    continue
                dist = math.hypot(
                    node.x_m - candidate_cell.center_x_m,
                    node.y_m - candidate_cell.center_y_m,
                )
                if dist <= best_dist:
                    best_dist = dist
                    best_key = candidate_key
        if best_key is not None:
            graph.cells[best_key].associated_node_ids.append(node.id)


def update_robot_cell_visit(
    graph: GraphState,
    robot_x_m: float,
    robot_y_m: float,
    cell_origin_x_m: float,
    cell_origin_y_m: float,
    cell_size_m: float,
    visit_radius_m: float,
    last_robot_cell_key: Optional[str],
) -> Optional[str]:
    if cell_size_m <= 0.0:
        return last_robot_cell_key

    grid_x, grid_y = cell_indices_from_world(
        robot_x_m,
        robot_y_m,
        cell_origin_x_m,
        cell_origin_y_m,
        cell_size_m,
    )
    current_key = cell_key(grid_x, grid_y)
    current_cell = graph.cells.get(current_key)
    if current_cell is None:
        return None

    if math.hypot(robot_x_m - current_cell.center_x_m, robot_y_m - current_cell.center_y_m) > visit_radius_m:
        return last_robot_cell_key

    if current_key != last_robot_cell_key:
        current_cell.visit_count += 1
    return current_key
