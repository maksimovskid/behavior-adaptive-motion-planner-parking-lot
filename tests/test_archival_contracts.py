"""Regression checks for planner failure and archival verification contracts."""

import unittest
from unittest.mock import patch

import numpy as np

from motion_planning.global_planning import hybrid_a_star
from motion_planning.local_planning.behaviour_and_local_planner import smooth_rrt_path
from motion_planning.local_planning.rrt_reeds_shepp import RRTReedsShepp
from simulation.parking_lot_map import build_parking_lot_obstacles
from simulation.path_setup import prepare_hybrid_path


class ArchivalContractTests(unittest.TestCase):
    def test_global_planner_rejects_invalid_poses(self):
        ox, oy = build_parking_lot_obstacles()
        for start in ([-5, 8, 0], [10, 6, 0], [float("nan"), 8, 0]):
            with self.subTest(start=start), self.assertRaises(ValueError):
                hybrid_a_star.hybrid_a_star_planning(start, [13, 32, np.pi], ox, oy, 1, np.pi / 12)

    def test_exhausted_search_has_explicit_failure(self):
        ox, oy = build_parking_lot_obstacles()
        with patch.object(hybrid_a_star, "update_node_with_analystic_expantion", return_value=(False, None)), \
                patch.object(hybrid_a_star, "get_neighbors", return_value=[]):
            with self.assertRaisesRegex(RuntimeError, "usable path"):
                prepare_hybrid_path([1.5, 8, 0], [13, 32, np.pi], ox, oy, 1, np.pi / 12)

    def test_empty_rrt_result_has_explicit_failure(self):
        for path in (None, [], [[0, 0, 0]], [[0, 0, 0], [0, 0, 0]]):
            with self.subTest(path=path), self.assertRaises(RuntimeError):
                smooth_rrt_path(path)

    def test_rrt_sampling_can_explore_when_configured(self):
        planner = RRTReedsShepp([0, 0, 0], [10, 0, 0], [], [0, 40], goal_sample_rate=10)
        with patch("random.randint", return_value=90), patch("random.uniform", return_value=5):
            self.assertEqual(planner.get_random_node().x, 5)
        planner = RRTReedsShepp([0, 0, 0], [10, 0, 0], [], [0, 40])
        with patch("random.randint", return_value=90):
            self.assertEqual(planner.get_random_node().x, 10)

    def test_blocked_rrt_returns_none(self):
        planner = RRTReedsShepp([0, 0, 0], [10, 0, 0], [(0, 0)], [0, 40], max_iter=1)
        self.assertIsNone(planner.planning(animation=False))
        with patch("motion_planning.local_planning.rrt_reeds_shepp.reeds_shepp_path_planning.reeds_shepp_path_planning",
                   return_value=(None, None, None, None, None)):
            self.assertIsNone(planner.steer(planner.start, planner.end))
