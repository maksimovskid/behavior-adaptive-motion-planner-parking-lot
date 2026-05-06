"""Tests for behavior-planner decisions and shared path preparation."""

from __future__ import annotations

import unittest

import numpy as np

from motion_planning.local_planning.behaviour_and_local_planner import BehaviouralLocalPlanner, BehaviourPlannerConfig
from motion_planning.velocity_planning import calc_lead_vehicle_ego_speed_profile
from simulation.parking_lot_map import build_parking_lot_obstacles
from simulation.path_setup import prepare_hybrid_path
from vehicle.kinematics import State


class BehaviourDecisionTests(unittest.TestCase):
    """Coverage for small behavior decisions that should stay stable."""

    def test_lane_change_requested_for_close_slow_lead_vehicle(self) -> None:
        """A close slow lead vehicle should trigger lane-change intent."""
        planner = BehaviouralLocalPlanner([], [], [], [], config=BehaviourPlannerConfig())
        ego = State(x=8.0, y=8.0, yaw=0.0, v=4.0)
        lead = State(x=10.0, y=8.0, yaw=0.0, v=0.5)

        self.assertTrue(planner.check_for_lane_change(ego, lead))

    def test_lane_change_not_requested_for_fast_lead_vehicle(self) -> None:
        """A close but fast lead vehicle should not trigger overtaking."""
        planner = BehaviouralLocalPlanner([], [], [], [], config=BehaviourPlannerConfig())
        ego = State(x=8.0, y=8.0, yaw=0.0, v=4.0)
        lead = State(x=10.0, y=8.0, yaw=0.0, v=4.0)

        self.assertFalse(planner.check_for_lane_change(ego, lead))

    def test_return_lane_change_requested_after_overtake_path_progress(self) -> None:
        """A return maneuver should be requested after the first lane-change path is long enough."""
        config = BehaviourPlannerConfig(follow_lane_after_points=3)
        planner = BehaviouralLocalPlanner([], [], [], [], config=config)
        planner.rrt_lane_change_x = [0.0, 1.0, 2.0, 3.0]
        ego = State(x=12.0, y=12.0, yaw=0.0, v=4.0)
        lead = State(x=0.0, y=8.0, yaw=0.0, v=0.5)

        self.assertTrue(planner.check_for_double_lane_change(ego, lead))

    def test_local_path_extension_uses_nearest_reference_suffix(self) -> None:
        """A finished RRT path should continue from the nearest reference point, not from path start."""
        planner = BehaviouralLocalPlanner([], [], [], [], config=BehaviourPlannerConfig())
        planner.rrt_lane_change_x_2 = [32.0, 38.0]
        planner.rrt_lane_change_y_2 = [22.0, 30.0]
        planner.rrt_lane_change_yaw_2 = [0.0, 0.0]
        planner.rrt_lane_change_k_2 = [0.0, 0.0]

        planner._extend_with_reference_suffix(
            planner.rrt_lane_change_x_2,
            planner.rrt_lane_change_y_2,
            planner.rrt_lane_change_yaw_2,
            planner.rrt_lane_change_k_2,
            [0.0, 10.0, 37.5, 39.0],
            [8.0, 8.0, 30.0, 31.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        )

        self.assertEqual(planner.rrt_lane_change_x_2[2], 37.5)


class VelocityPlanningTests(unittest.TestCase):
    """Coverage for speed adaptation around a lead vehicle."""

    def test_ego_speed_profile_slows_to_lead_vehicle_speed(self) -> None:
        """When the lead vehicle is close, ego target speed is capped by lead speed."""
        cx = [float(index) for index in range(100)]
        cy = [0.0] * len(cx)
        cyaw = [0.0] * len(cx)
        ego = State(x=8.0, y=8.0, yaw=0.0, v=4.0)
        lead = State(x=10.0, y=8.0, yaw=0.0, v=1.5)

        profile = calc_lead_vehicle_ego_speed_profile(cx, cy, cyaw, 4.0, ego, lead, target_ind=0)

        self.assertEqual(profile[0], 0.0)


class PathPreparationTests(unittest.TestCase):
    """Coverage for shared Hybrid A* path preparation contracts."""

    def test_prepared_path_curvature_matches_path_length(self) -> None:
        """Curvature samples should be aligned with path points for controller indexing."""
        ox, oy = build_parking_lot_obstacles()
        path = prepare_hybrid_path(
            [1.5, 8.0, np.deg2rad(0.0)],
            [13.0, 32.0, np.deg2rad(180.0)],
            ox,
            oy,
            1.0,
            np.deg2rad(15.0),
        )

        self.assertEqual(len(path.x), len(path.curvature))


if __name__ == "__main__":
    unittest.main()
