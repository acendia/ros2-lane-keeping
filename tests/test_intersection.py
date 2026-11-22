"""Check the painted stop line and the simulated route's stop sequence."""

import cv2
import numpy as np
from dataclasses import replace
from pathlib import Path
import pytest

from auto_drive_sim.intersection import StraightRoute, stop_line_distance
from auto_drive_sim.lane_perception import CameraModel, GroundPlane, PerceptionConfig, analyze_frame


def test_stop_line_distinguishes_transverse_paint_from_lane_lines():
    ground = GroundPlane(CameraModel())
    image = np.full((360, 640, 3), 80, dtype=np.uint8)
    for side in (-1.81, 1.81):
        cv2.line(image, ground.project(3.5, side), ground.project(12.0, side),
                 (230, 230, 230), 3)
    assert stop_line_distance(image, ground) is None
    corners = [ground.project(x, y) for x, y in
               ((5.0, -1.3), (5.34, -1.3), (5.34, 1.3), (5.0, 1.3))]
    cv2.fillConvexPoly(image, np.asarray(corners), (230, 230, 230))
    assert stop_line_distance(image, ground) == pytest.approx(5.17, abs=0.25)


def test_route_holds_at_rest_after_a_stale_input_and_stops_at_goal():
    route = StraightRoute(28.0, 6.0)
    assert route.limit(1.0, 6.0, 0.0, True) == 0.0
    assert route.phase == 'approach'
    for station in (11.0, 11.1, 10.9):
        route.observe_line(station)
    assert route.line_m == pytest.approx(11.0)
    assert route.limit(3.0, 8.5, 0.0, True) == 0.0
    assert route.phase == 'hold'
    assert route.limit(4.5, 8.5, 0.0, False) == 0.0
    assert route.limit(5.0, 8.5, 0.0, True) == 0.0
    assert route.limit(6.9, 8.5, 0.0, True) == 0.0
    assert route.phase == 'hold'
    route.limit(7.0, 8.5, 0.0, True)
    assert route.phase == 'cross'
    assert route.limit(7.1, 8.5, 0.0, True) > 0
    assert route.limit(30.0, 27.9, 0.0, True) == 0
    assert route.phase == 'arrived'


def test_crosswalk_and_stop_bar_do_not_replace_lane_boundaries():
    frame = cv2.imread(str(Path(__file__).parent / 'frames/intersection_approach.png'))
    config = replace(PerceptionConfig(), lane_strategy='intersection')
    result = analyze_frame(frame, GroundPlane(CameraModel()), config)
    assert result.valid
    assert result.target_y_m == pytest.approx(0.0, abs=0.12)


def test_yielding_requires_observed_traffic_followed_by_a_clear_view():
    route = StraightRoute(28.0, 6.0, yield_to_traffic=True)
    for _ in range(3):
        route.observe_line(11.0)
    for time in (1.0, 4.0):
        assert route.limit(time, 8.5, 0.0, True, False) == 0.0
    assert route.phase == 'hold'
    route.limit(4.1, 8.5, 0.0, True, True)
    route.limit(4.3, 8.5, 0.0, True, False)
    route.limit(4.9, 8.5, 0.0, False, False)
    route.limit(5.0, 8.5, 0.0, True, False)
    route.limit(5.7, 8.5, 0.0, True, False)
    assert route.phase == 'hold'
    route.limit(6.0, 8.5, 0.0, True, False)
    assert route.phase == 'hold'
    route.limit(7.1, 8.5, 0.0, True, False)
    assert route.phase == 'cross'
