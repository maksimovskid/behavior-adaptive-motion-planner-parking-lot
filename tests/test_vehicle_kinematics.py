"""Tests for constant-acceleration helper equations."""

from __future__ import annotations

import unittest

from vehicle.kinematics import calc_distance, calc_final_speed, calc_time


class ConstantAccelerationKinematicsTests(unittest.TestCase):
    """Coverage for helper formulas reused by speed-profile planners."""

    def test_distance_for_speed_change(self) -> None:
        """Distance follows v_f^2 = v_i^2 + 2ad."""
        self.assertAlmostEqual(calc_distance(0.0, 10.0, 2.0), 25.0)

    def test_time_for_speed_change(self) -> None:
        """Time follows v_f = v_i + at."""
        self.assertAlmostEqual(calc_time(10.0, 4.0, -2.0), 3.0)

    def test_final_speed_clamps_negative_root(self) -> None:
        """Negative roots are clamped to a small positive speed."""
        self.assertAlmostEqual(calc_final_speed(1.0, -5.0, 2.0), 0.000001)

    def test_final_speed_for_distance(self) -> None:
        """Final speed is recovered from acceleration and distance."""
        self.assertAlmostEqual(calc_final_speed(0.0, 2.0, 25.0), 10.0)

if __name__ == "__main__":
    unittest.main()
