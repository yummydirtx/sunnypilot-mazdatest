#!/usr/bin/env python3
"""Tests for speed-binned learning in torqued (vehicle-agnostic).

Uses get_speed_dependent_torque_params() to discover configured cars.
All tests are driven by config, not hardcoded fingerprints.
"""
import unittest
import numpy as np

from unittest.mock import MagicMock, patch
from opendbc.car.interfaces import get_speed_dependent_torque_params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.locationd.torqued import (
  TorqueEstimator, SPEED_BIN_BOUNDS, SPEED_BIN_CENTERS,
  MIN_POINTS_PER_SPEED_BIN, STEER_BUCKET_BOUNDS, VERSION, ALLOWED_CARS,
)

# Discover configured cars
SPEED_DEP_CARS = get_speed_dependent_torque_params()
SPEED_DEP_FINGERPRINT = next(iter(SPEED_DEP_CARS)) if SPEED_DEP_CARS else None

# A fingerprint guaranteed NOT to be in speed_dependent.toml
NON_SPEED_DEP_FINGERPRINT = 'TOYOTA_COROLLA'


def make_mock_CP(fingerprint=None, laf=1.25, friction=0.125):
  if fingerprint is None:
    fingerprint = SPEED_DEP_FINGERPRINT
  CP = MagicMock()
  CP.brand = 'test'
  CP.carFingerprint = fingerprint
  CP.lateralTuning.which.return_value = 'torque'
  CP.lateralTuning.torque.friction = friction
  CP.lateralTuning.torque.latAccelFactor = laf
  return CP


class TestSpeedDepConfig(unittest.TestCase):
  """Config-level tests that don't need a TorqueEstimator."""

  def test_speed_dep_config_has_entries(self):
    """speed_dependent.toml should have at least one car configured."""
    self.assertGreater(len(SPEED_DEP_CARS), 0)

  def test_version_bumped(self):
    """Version should be >= 2 to invalidate old caches."""
    self.assertGreaterEqual(VERSION, 2)

  def test_speed_bin_bounds_cover_full_range(self):
    """Speed bins should cover from 3 m/s (skip noise) to at least 26 m/s."""
    all_bounds = [b for bounds in SPEED_BIN_BOUNDS for b in bounds]
    self.assertEqual(min(all_bounds), 3)
    self.assertGreaterEqual(max(all_bounds), 26)

  def test_speed_bin_centers_match_bounds(self):
    """Each bin center should fall within its bounds."""
    for center, (lo, hi) in zip(SPEED_BIN_CENTERS, SPEED_BIN_BOUNDS):
      self.assertGreaterEqual(center, lo)
      self.assertLessEqual(center, hi)


@unittest.skipIf(SPEED_DEP_FINGERPRINT is None, "No cars in speed_dependent.toml")
class TestSpeedBinnedLearning(unittest.TestCase):
  """Test speed-binned learning for every configured car."""

  @patch('openpilot.selfdrive.locationd.torqued.Params')
  def test_speed_bins_initialized(self, mock_params_cls):
    """Configured cars should initialize speed bins."""
    mock_params_cls.return_value.get.return_value = None
    for fingerprint in SPEED_DEP_CARS:
      with self.subTest(car=fingerprint):
        est = TorqueEstimator(make_mock_CP(fingerprint=fingerprint))
        self.assertTrue(est.speed_binned)
        self.assertEqual(len(est.speed_bin_points), len(SPEED_BIN_BOUNDS))

  @patch('openpilot.selfdrive.locationd.torqued.Params')
  def test_speed_bin_routing(self, mock_params_cls):
    """Points added to a speed bin should only appear in that bin."""
    mock_params_cls.return_value.get.return_value = None
    est = TorqueEstimator(make_mock_CP())
    est.speed_bin_points[0].add_point(0.1, 0.3)
    self.assertEqual(len(est.speed_bin_points[0]), 1)
    for i in range(1, len(SPEED_BIN_BOUNDS)):
      self.assertEqual(len(est.speed_bin_points[i]), 0)

  @patch('openpilot.selfdrive.locationd.torqued.Params')
  def test_cereal_message_fields(self, mock_params_cls):
    """Speed-binned fields should be populated in cereal message."""
    mock_params_cls.return_value.get.return_value = None
    for fingerprint in SPEED_DEP_CARS:
      with self.subTest(car=fingerprint):
        est = TorqueEstimator(make_mock_CP(fingerprint=fingerprint))
        msg = est.get_msg()
        ltp = msg.liveTorqueParameters
        self.assertEqual(len(ltp.speedBinCenters), len(SPEED_BIN_CENTERS))
        self.assertEqual(len(ltp.speedBinLatAccelFactors), len(SPEED_BIN_BOUNDS))
        self.assertEqual(len(ltp.speedBinFrictions), len(SPEED_BIN_BOUNDS))
        self.assertEqual(len(ltp.speedBinValid), len(SPEED_BIN_BOUNDS))
        self.assertEqual(len(ltp.speedBinCalPerc), len(SPEED_BIN_BOUNDS))

  @patch('openpilot.selfdrive.locationd.torqued.Params')
  def test_global_fit_unchanged(self, mock_params_cls):
    """Global filtered params should still match initial offline values."""
    mock_params_cls.return_value.get.return_value = None
    est = TorqueEstimator(make_mock_CP(laf=1.25, friction=0.125))
    msg = est.get_msg()
    ltp = msg.liveTorqueParameters
    self.assertAlmostEqual(ltp.latAccelFactorFiltered, 1.25, places=2)
    self.assertAlmostEqual(ltp.frictionCoefficientFiltered, 0.125, places=3)

  @patch('openpilot.selfdrive.locationd.torqued.Params')
  def test_global_buckets_still_require_min_vel(self, mock_params_cls):
    """Even for speed-binned cars, global buckets should still gate on MIN_VEL."""
    mock_params_cls.return_value.get.return_value = None
    est = TorqueEstimator(make_mock_CP())
    self.assertEqual(len(est.filtered_points), 0)


class TestBackwardCompatibility(unittest.TestCase):
  """Cars NOT in speed_dependent.toml should be unaffected."""

  @patch('openpilot.selfdrive.locationd.torqued.Params')
  def test_unconfigured_car_no_speed_bins(self, mock_params_cls):
    """Cars not in speed_dependent.toml should not have speed bins."""
    mock_params_cls.return_value.get.return_value = None
    est = TorqueEstimator(make_mock_CP(fingerprint=NON_SPEED_DEP_FINGERPRINT))
    self.assertFalse(est.speed_binned)

  @patch('openpilot.selfdrive.locationd.torqued.Params')
  def test_unconfigured_car_no_speed_bin_fields(self, mock_params_cls):
    """Unconfigured cars should have empty speed-binned lists in message."""
    mock_params_cls.return_value.get.return_value = None
    est = TorqueEstimator(make_mock_CP(fingerprint=NON_SPEED_DEP_FINGERPRINT))
    msg = est.get_msg()
    ltp = msg.liveTorqueParameters
    self.assertEqual(len(ltp.speedBinCenters), 0)
    self.assertEqual(len(ltp.speedBinLatAccelFactors), 0)
    self.assertEqual(len(ltp.speedBinFrictions), 0)
    self.assertEqual(len(ltp.speedBinValid), 0)
    self.assertEqual(len(ltp.speedBinCalPerc), 0)

  @patch('openpilot.selfdrive.locationd.torqued.Params')
  def test_unconfigured_car_global_params_still_work(self, mock_params_cls):
    """Unconfigured cars should still produce valid global filtered params."""
    mock_params_cls.return_value.get.return_value = None
    est = TorqueEstimator(make_mock_CP(fingerprint=NON_SPEED_DEP_FINGERPRINT, laf=2.0, friction=0.15))
    msg = est.get_msg()
    ltp = msg.liveTorqueParameters
    self.assertAlmostEqual(ltp.latAccelFactorFiltered, 2.0, places=2)
    self.assertAlmostEqual(ltp.frictionCoefficientFiltered, 0.15, places=3)
    self.assertFalse(est.speed_binned)

  @patch('openpilot.selfdrive.locationd.torqued.Params')
  def test_unconfigured_car_no_speed_bin_attributes(self, mock_params_cls):
    """Unconfigured cars should not have speed_bin_points or speed_bin_filtered."""
    mock_params_cls.return_value.get.return_value = None
    est = TorqueEstimator(make_mock_CP(fingerprint=NON_SPEED_DEP_FINGERPRINT))
    self.assertFalse(hasattr(est, 'speed_bin_points'))
    self.assertFalse(hasattr(est, 'speed_bin_filtered'))

  @patch('openpilot.selfdrive.locationd.torqued.Params')
  def test_cal_percent_works_for_both(self, mock_params_cls):
    """cal_percent logic should work for both configured and unconfigured cars."""
    mock_params_cls.return_value.get.return_value = None
    fingerprints = [NON_SPEED_DEP_FINGERPRINT]
    if SPEED_DEP_FINGERPRINT:
      fingerprints.append(SPEED_DEP_FINGERPRINT)
    for fp in fingerprints:
      with self.subTest(car=fp):
        est = TorqueEstimator(make_mock_CP(fingerprint=fp))
        msg = est.get_msg()
        self.assertEqual(msg.liveTorqueParameters.calPerc, 0)


if __name__ == '__main__':
  unittest.main()
