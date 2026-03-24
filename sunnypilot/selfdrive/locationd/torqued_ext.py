"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from cereal import car

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

RELAXED_MIN_BUCKET_POINTS = np.array([1, 200, 300, 500, 500, 300, 200, 1])

ALLOWED_CARS = ['toyota', 'hyundai', 'rivian', 'honda']

# Speed-binned learning constants — bins aligned with common US driving speeds.
# Skip <5 m/s where lat_accel = v*yaw_rate is noisy.
SPEED_BIN_BOUNDS = [(5, 12), (12, 18), (18, 24), (24, 31), (31, 40)]
SPEED_BIN_CENTERS = [8.5, 15.0, 21.0, 27.5, 35.5]
MIN_POINTS_PER_SPEED_BIN = 600
FIT_POINTS_PER_SPEED_BIN = 400
POINTS_PER_SPEED_BUCKET = 500
SPEED_BIN_MIN_CAL_PERC = 80  # min cal% before applying learned values to closure
FRICTION_FACTOR = 1.5  # same as upstream


class TorqueEstimatorExt:
  def __init__(self, CP: car.CarParams):
    self.CP = CP
    self._params = Params()
    self.frame = -1

    self.enforce_torque_control_toggle = self._params.get_bool("EnforceTorqueControl")  # only during init
    self.use_params = self.CP.brand in ALLOWED_CARS and self.CP.lateralTuning.which() == 'torque'
    self.use_live_torque_params = self._params.get_bool("LiveTorqueParamsToggle")
    self.torque_override_enabled = self._params.get_bool("TorqueParamsOverrideEnabled")
    self.use_speed_dep = self._params.get_bool("SpeedDependentTorqueToggle")
    self.speed_binned = False
    self.min_bucket_points = RELAXED_MIN_BUCKET_POINTS
    self.factor_sanity = 0.0
    self.friction_sanity = 0.0
    self.offline_latAccelFactor = 0.0
    self.offline_friction = 0.0

  def initialize_custom_params(self, decimated=False):
    self.update_use_params()

    if self.enforce_torque_control_toggle:
      if self._params.get_bool("LiveTorqueParamsRelaxedToggle"):
        self.min_bucket_points = RELAXED_MIN_BUCKET_POINTS / (10 if decimated else 1)
        self.factor_sanity = 0.5 if decimated else 1.0
        self.friction_sanity = 0.8 if decimated else 1.0

      if self._params.get_bool("CustomTorqueParams"):
        self.offline_latAccelFactor = float(self._params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
        self.offline_friction = float(self._params.get("TorqueParamsOverrideFriction", return_default=True))

    # Speed-binned learning: toggle-gated, works for any torque car
    self.speed_binned = self.CP.lateralTuning.which() == 'torque' and self.use_speed_dep

  def _update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.use_live_torque_params = self._params.get_bool("LiveTorqueParamsToggle")
      self.torque_override_enabled = self._params.get_bool("TorqueParamsOverrideEnabled")

  def update_use_params(self):
    self._update_params()

    if self.enforce_torque_control_toggle:
      if self.torque_override_enabled:
        self.use_params = False
      else:
        self.use_params = self.use_live_torque_params

    self.frame += 1

  # --- Speed-binned learning hooks (called from torqued.py) ---

  def _post_reset(self):
    """Called from TorqueEstimator.reset(). Initializes per-speed-bin buckets."""
    if not self.speed_binned:
      return

    from openpilot.selfdrive.locationd.torqued import TorqueBuckets, STEER_BUCKET_BOUNDS, MIN_FILTER_DECAY
    from opendbc.sunnypilot.car.interfaces import _get_speed_dep_config

    self.speed_bin_points = [
      TorqueBuckets(x_bounds=STEER_BUCKET_BOUNDS,
                    min_points=self.min_bucket_points,
                    min_points_total=MIN_POINTS_PER_SPEED_BIN,
                    points_per_bucket=POINTS_PER_SPEED_BUCKET,
                    rowsize=3)
      for _ in SPEED_BIN_BOUNDS
    ]

    cfg = _get_speed_dep_config().get(self.CP.carFingerprint, {})
    ref_lafs = cfg.get('laf_bp', [self.offline_latAccelFactor] * len(SPEED_BIN_BOUNDS))
    ref_frictions = cfg.get('friction_bp', [self.offline_friction] * len(SPEED_BIN_BOUNDS))
    self.speed_bin_decays = [MIN_FILTER_DECAY] * len(SPEED_BIN_BOUNDS)
    self.speed_bin_filtered = [
      {'latAccelFactor': FirstOrderFilter(ref_lafs[i], self.speed_bin_decays[i], DT_MDL),
       'frictionCoefficient': FirstOrderFilter(ref_frictions[i], self.speed_bin_decays[i], DT_MDL)}
      for i in range(len(SPEED_BIN_BOUNDS))
    ]
    self.speed_bin_laf_bounds = [
      ((1.0 - self.factor_sanity) * laf, (1.0 + self.factor_sanity) * laf)
      for laf in ref_lafs
    ]
    self.speed_bin_friction_bounds = [
      ((1.0 - self.friction_sanity) * f, (1.0 + self.friction_sanity) * f)
      for f in ref_frictions
    ]

  def _ensure_speed_bins(self):
    """Lazy init: create bins and restore cache on first use."""
    if hasattr(self, 'speed_bin_points') and self._speed_bin_resets == self.resets:
      return
    self._speed_bin_resets = self.resets
    self._post_reset()
    # Restore from cache if available
    try:
      from cereal import log
      cache = self._params.get("LiveTorqueParameters")
      if cache:
        with log.Event.from_bytes(cache) as evt:
          self._restore_ext_cache(evt.liveTorqueParameters)
    except Exception:
      pass

  def _on_torque_point(self, steer, lateral_acc, vego):
    """Called from handle_log. Routes quality-filtered points to speed bins."""
    if not self.speed_binned:
      return
    self._ensure_speed_bins()
    for i, (lo, hi) in enumerate(SPEED_BIN_BOUNDS):
      if lo <= vego < hi:
        self.speed_bin_points[i].add_point(steer, lateral_acc)
        break

  def _restore_ext_cache(self, cache_ltp):
    """Restores per-bin filter values and points from cache."""
    if not self.speed_binned:
      return
    if (len(cache_ltp.speedBinLatAccelFactors) == len(SPEED_BIN_BOUNDS) and
        len(cache_ltp.speedBinFrictions) == len(SPEED_BIN_BOUNDS)):
      for i in range(len(SPEED_BIN_BOUNDS)):
        self.speed_bin_filtered[i]['latAccelFactor'].x = cache_ltp.speedBinLatAccelFactors[i]
        self.speed_bin_filtered[i]['frictionCoefficient'].x = cache_ltp.speedBinFrictions[i]
      if len(cache_ltp.speedBinPoints) == len(SPEED_BIN_BOUNDS):
        for i in range(len(SPEED_BIN_BOUNDS)):
          self.speed_bin_points[i].load_points(cache_ltp.speedBinPoints[i])
      self.speed_bin_decays = [self.decay] * len(SPEED_BIN_BOUNDS)
      cloudlog.info("restored speed-bin torque params from cache")

  def _estimate_params_speed_binned(self):
    """Run independent SVD fit per speed bin."""
    from openpilot.selfdrive.locationd.torqued import slope2rot, MAX_FILTER_DECAY

    results = []
    for i, bucket in enumerate(self.speed_bin_points):
      if bucket.is_calculable():
        points = bucket.get_points(FIT_POINTS_PER_SPEED_BIN)
        try:
          _, _, v = np.linalg.svd(points, full_matrices=False)
          slope, offset = -v.T[0:2, 2] / v.T[2, 2]
          _, spread = np.matmul(points[:, [0, 2]], slope2rot(slope)).T
          friction_coeff = np.std(spread) * FRICTION_FACTOR
          if not any(np.isnan(val) for val in [slope, friction_coeff]):
            laf_lo, laf_hi = self.speed_bin_laf_bounds[i]
            fric_lo, fric_hi = self.speed_bin_friction_bounds[i]
            laf = np.clip(slope, laf_lo, laf_hi)
            fric = np.clip(friction_coeff, fric_lo, fric_hi)
            self.speed_bin_decays[i] = min(self.speed_bin_decays[i] + DT_MDL, MAX_FILTER_DECAY)
            self.speed_bin_filtered[i]['latAccelFactor'].update(laf)
            self.speed_bin_filtered[i]['latAccelFactor'].update_alpha(self.speed_bin_decays[i])
            self.speed_bin_filtered[i]['frictionCoefficient'].update(fric)
            self.speed_bin_filtered[i]['frictionCoefficient'].update_alpha(self.speed_bin_decays[i])
            results.append((i, True))
            continue
        except np.linalg.LinAlgError:
          pass
      results.append((i, False))
    return results

  def _extend_msg(self, ltp, with_points):
    """Called from get_msg. Populates speed-bin fields in cereal message."""
    if not self.speed_binned or not hasattr(self, 'speed_bin_points'):
      return
    bin_results = self._estimate_params_speed_binned()
    bin_cal_percs = [float(self.speed_bin_points[i].get_valid_percent()) for i in range(len(SPEED_BIN_BOUNDS))]
    ltp.speedBinCenters = SPEED_BIN_CENTERS
    ltp.speedBinLatAccelFactors = [float(self.speed_bin_filtered[i]['latAccelFactor'].x) for i in range(len(SPEED_BIN_BOUNDS))]
    ltp.speedBinFrictions = [float(self.speed_bin_filtered[i]['frictionCoefficient'].x) for i in range(len(SPEED_BIN_BOUNDS))]
    ltp.speedBinValid = [valid and bin_cal_percs[i] >= SPEED_BIN_MIN_CAL_PERC for i, (_, valid) in enumerate(bin_results)]
    ltp.speedBinCalPerc = bin_cal_percs
    if with_points:
      ltp.speedBinPoints = [bucket.get_points(FIT_POINTS_PER_SPEED_BIN)[:, [0, 2]].tolist() for bucket in self.speed_bin_points]
