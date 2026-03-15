#!/usr/bin/env python3
"""Tests for speed-dependent torque mechanism (vehicle-agnostic).

Uses get_speed_dependent_torque_params() to discover configured cars from
speed_dependent.toml, then tests the CarInterfaceExt callbacks generically.
"""
import unittest
import numpy as np

from unittest.mock import MagicMock
from opendbc.car import structs
from opendbc.car.interfaces import get_speed_dependent_torque_params
from opendbc.sunnypilot.car.interfaces import LatControlInputs

# Discover all cars with speed-dependent torque config
SPEED_DEP_CARS = get_speed_dependent_torque_params()


def get_car_interface_ext(fingerprint):
  """Dynamically import and instantiate CarInterfaceExt for a given fingerprint."""
  from opendbc.car.values import PLATFORMS
  platform = PLATFORMS.get(fingerprint)
  if platform is None:
    return None
  # Derive brand from the platform enum's module path (e.g. opendbc.car.mazda.values -> mazda)
  brand = platform.__class__.__module__.split('.')[-2]
  mod = __import__(f'opendbc.sunnypilot.car.{brand}.interface_ext', fromlist=['CarInterfaceExt'])
  CP = MagicMock()
  CP.carFingerprint = fingerprint
  CI_Base = MagicMock()
  return mod.CarInterfaceExt(CP, CI_Base)


@unittest.skipIf(len(SPEED_DEP_CARS) == 0, "No cars configured in speed_dependent.toml")
class TestSpeedDepTorqueCallbacks(unittest.TestCase):
  """Test speed-dependent torque callbacks for every configured car."""

  def test_config_has_required_keys(self):
    """Every entry in speed_dependent.toml must have speed_bp and laf_bp."""
    for fingerprint, cfg in SPEED_DEP_CARS.items():
      with self.subTest(car=fingerprint):
        self.assertIn('speed_bp', cfg, f"{fingerprint} missing speed_bp")
        self.assertIn('laf_bp', cfg, f"{fingerprint} missing laf_bp")
        self.assertEqual(len(cfg['speed_bp']), len(cfg['laf_bp']),
                         f"{fingerprint} speed_bp/laf_bp length mismatch")
        self.assertGreater(len(cfg['speed_bp']), 1,
                           f"{fingerprint} needs at least 2 breakpoints")

  def test_laf_table_is_consistent(self):
    """laf_bp and speed_bp should have matching lengths and valid values."""
    for fingerprint, cfg in SPEED_DEP_CARS.items():
      with self.subTest(car=fingerprint):
        self.assertEqual(len(cfg['speed_bp']), len(cfg['laf_bp']))
        for laf in cfg['laf_bp']:
          self.assertGreater(laf, 0, f"{fingerprint}: LAF must be positive")

  def test_closure_uses_table_values(self):
    """Closure should use the LAF table for torque computation."""
    for fingerprint in SPEED_DEP_CARS:
      with self.subTest(car=fingerprint):
        ext = get_car_interface_ext(fingerprint)
        if ext is None or not hasattr(ext, 'torque_from_lateral_accel_speed_dep_closure'):
          self.skipTest(f"No CarInterfaceExt with speed-dep closure for {fingerprint}")
        tp = MagicMock()
        ext.v_ego = 15.0
        laf = float(np.interp(15.0, ext.speed_dep_speed_bp, ext.speed_dep_laf_v))
        torque = ext.torque_from_lateral_accel_speed_dep_closure(1.0, tp)
        self.assertAlmostEqual(torque, 1.0 / laf, places=6,
                                msg=f"{fingerprint}: closure should use table LAF")

  def test_closure_tracks_updated_table(self):
    """After update_speed_dep_laf, closure should produce different torque."""
    for fingerprint, cfg in SPEED_DEP_CARS.items():
      with self.subTest(car=fingerprint):
        ext = get_car_interface_ext(fingerprint)
        if ext is None or not hasattr(ext, 'torque_from_lateral_accel_speed_dep_closure'):
          self.skipTest(f"No CarInterfaceExt with speed-dep closure for {fingerprint}")
        tp = MagicMock()
        ext.v_ego = cfg['speed_bp'][0]
        torque_before = ext.torque_from_lateral_accel_speed_dep_closure(1.0, tp)
        # Nudge LAF by +10% (within ±30% bounds)
        nudged = [v * 1.1 for v in ext.speed_dep_laf_v]
        ext.update_speed_dep_laf(cfg['speed_bp'], nudged,
                                  cfg.get('friction_bp', [0.1] * len(cfg['speed_bp'])),
                                  [True] * len(cfg['speed_bp']))
        torque_after = ext.torque_from_lateral_accel_speed_dep_closure(1.0, tp)
        self.assertNotAlmostEqual(torque_before, torque_after, places=3,
                                   msg=f"{fingerprint}: closure should reflect updated LAF")

  def test_inverse_consistency(self):
    """torque -> lat_accel -> torque roundtrip should be identity."""
    for fingerprint in SPEED_DEP_CARS:
      with self.subTest(car=fingerprint):
        ext = get_car_interface_ext(fingerprint)
        if ext is None or not hasattr(ext, 'torque_from_lateral_accel_speed_dep_closure'):
          self.skipTest(f"No CarInterfaceExt with speed-dep closure for {fingerprint}")
        tp = MagicMock()
        for speed in [5.0, 15.0, 25.0]:
          ext.v_ego = speed
          lat_accel = 0.8
          torque = ext.torque_from_lateral_accel_speed_dep_closure(lat_accel, tp)
          recovered = ext.lateral_accel_from_torque_speed_dep_closure(torque, tp)
          self.assertAlmostEqual(lat_accel, recovered, places=6,
                                 msg=f"{fingerprint} @ {speed} m/s: inverse failed")

  def test_torque_space_callback_returns_speed_dep(self):
    """Configured cars should return speed-dep callback from torque_from_lateral_accel_in_torque_space()."""
    for fingerprint in SPEED_DEP_CARS:
      with self.subTest(car=fingerprint):
        ext = get_car_interface_ext(fingerprint)
        if ext is None or not hasattr(ext, 'torque_from_lateral_accel_in_torque_space'):
          self.skipTest(f"No CarInterfaceExt for {fingerprint}")
        cb = ext.torque_from_lateral_accel_in_torque_space()
        self.assertNotEqual(cb.__name__, 'torque_from_lateral_accel_linear_in_torque_space',
                            f"{fingerprint} should not use linear callback")

  def test_update_speed_dep_laf_within_bounds(self):
    """Live-learned values within sanity bounds should update the table."""
    for fingerprint, cfg in SPEED_DEP_CARS.items():
      with self.subTest(car=fingerprint):
        ext = get_car_interface_ext(fingerprint)
        if ext is None or not hasattr(ext, 'update_speed_dep_laf'):
          self.skipTest(f"No update_speed_dep_laf for {fingerprint}")
        original = list(ext.speed_dep_laf_v)
        # Nudge each LAF by +5% (within 0.5x-2.0x bounds)
        nudged_laf = [v * 1.05 for v in original]
        friction = cfg.get('friction_bp', [0.1] * len(cfg['speed_bp']))
        ext.update_speed_dep_laf(cfg['speed_bp'], nudged_laf, friction,
                                 [True] * len(cfg['speed_bp']))
        for i in range(len(original)):
          self.assertAlmostEqual(ext.speed_dep_laf_v[i], nudged_laf[i], places=4,
                                 msg=f"{fingerprint} bin {i}: should accept +5% nudge")

  def test_update_speed_dep_laf_out_of_bounds_rejected(self):
    """Values far outside sanity range should be rejected."""
    for fingerprint, cfg in SPEED_DEP_CARS.items():
      with self.subTest(car=fingerprint):
        ext = get_car_interface_ext(fingerprint)
        if ext is None or not hasattr(ext, 'update_speed_dep_laf'):
          self.skipTest(f"No update_speed_dep_laf for {fingerprint}")
        original = list(ext.speed_dep_laf_v)
        # 5x is way outside 0.5x-2.0x bounds
        bad_laf = [v * 5.0 for v in original]
        friction = cfg.get('friction_bp', [0.1] * len(cfg['speed_bp']))
        ext.update_speed_dep_laf(cfg['speed_bp'], bad_laf, friction,
                                 [True] * len(cfg['speed_bp']))
        self.assertEqual(ext.speed_dep_laf_v, original,
                         f"{fingerprint}: out-of-bounds LAF should be rejected")

  def test_update_speed_dep_laf_invalid_bins_skipped(self):
    """Bins marked invalid should not be updated."""
    for fingerprint, cfg in SPEED_DEP_CARS.items():
      with self.subTest(car=fingerprint):
        ext = get_car_interface_ext(fingerprint)
        if ext is None or not hasattr(ext, 'update_speed_dep_laf'):
          self.skipTest(f"No update_speed_dep_laf for {fingerprint}")
        n = len(cfg['speed_bp'])
        original = list(ext.speed_dep_laf_v)
        nudged_laf = [v * 1.05 for v in original]
        friction = cfg.get('friction_bp', [0.1] * n)
        # Alternate valid/invalid
        valid = [(i % 2 == 0) for i in range(n)]
        ext.update_speed_dep_laf(cfg['speed_bp'], nudged_laf, friction, valid)
        for i in range(n):
          if valid[i]:
            self.assertAlmostEqual(ext.speed_dep_laf_v[i], nudged_laf[i], places=4)
          else:
            self.assertAlmostEqual(ext.speed_dep_laf_v[i], original[i], places=4)


@unittest.skipIf(len(SPEED_DEP_CARS) == 0, "No cars configured in speed_dependent.toml")
class TestNonConfiguredCarsUnaffected(unittest.TestCase):
  """Cars NOT in speed_dependent.toml should use linear model."""

  def test_unconfigured_car_uses_linear(self):
    """A car not in speed_dependent.toml should return linear callback."""
    # Use a known non-speed-dep fingerprint
    CP = MagicMock()
    CP.carFingerprint = 'DEFINITELY_NOT_A_REAL_CAR'
    CI_Base = MagicMock()
    # Import any brand's CarInterfaceExt — the fingerprint check should fall through
    brand_fingerprint = next(iter(SPEED_DEP_CARS))
    from opendbc.car.values import PLATFORMS
    platform = PLATFORMS.get(brand_fingerprint)
    brand = platform.__class__.__module__.split('.')[-2]
    mod = __import__(f'opendbc.sunnypilot.car.{brand}.interface_ext', fromlist=['CarInterfaceExt'])
    ext = mod.CarInterfaceExt(CP, CI_Base)
    cb = ext.torque_from_lateral_accel_in_torque_space()
    self.assertEqual(cb, CI_Base.torque_from_lateral_accel_linear_in_torque_space)


if __name__ == '__main__':
  unittest.main()
