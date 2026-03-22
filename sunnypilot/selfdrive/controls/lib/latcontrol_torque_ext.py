"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride


class LatControlTorqueExt(NeuralNetworkLateralControl, LatControlTorqueExtOverride):
  def __init__(self, lac_torque, CP, CP_SP, CI):
    NeuralNetworkLateralControl.__init__(self, lac_torque, CP, CP_SP, CI)
    LatControlTorqueExtOverride.__init__(self, CP)
    self._CI = CI

  def update(self, CS, VM, pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
             desired_lateral_accel, actual_lateral_accel, lateral_accel_deadzone, gravity_adjusted_lateral_accel,
             desired_curvature, actual_curvature, steer_limited_by_safety, output_torque):
    self._ff = ff
    self._pid = pid
    self._pid_log = pid_log
    self._setpoint = setpoint
    self._measurement = measurement
    self._roll_compensation = roll_compensation
    self._lateral_accel_deadzone = lateral_accel_deadzone
    self._desired_lateral_accel = desired_lateral_accel
    self._actual_lateral_accel = actual_lateral_accel
    self._desired_curvature = desired_curvature
    self._actual_curvature = actual_curvature
    self._gravity_adjusted_lateral_accel = gravity_adjusted_lateral_accel
    self._steer_limited_by_safety = steer_limited_by_safety
    self._output_torque = output_torque

    self.update_calculations(CS, VM, desired_lateral_accel)
    self.update_neural_network_feedforward(CS, params, calibrated_pose)

    return self._pid_log, self._output_torque

  def update_speed_dep_torque(self, torque_params):
    """Apply speed-dependent learned values from torqued to the CI and PID limits."""
    speed_bp = list(torque_params.speedBinCenters)
    laf_bp = list(torque_params.speedBinLatAccelFactors)
    friction_bp = list(torque_params.speedBinFrictions)
    valid_bp = list(torque_params.speedBinValid)

    if hasattr(self._CI, 'update_speed_dep_laf'):
      self._CI.update_speed_dep_laf(speed_bp, laf_bp, friction_bp, valid_bp)

    # Use representative LAF for PID limits (most common driving speed)
    self.lac_torque.torque_params.latAccelFactor = float(np.interp(20.0, speed_bp, laf_bp))
    self.lac_torque.torque_params.latAccelOffset = torque_params.latAccelOffsetFiltered
    self.lac_torque.update_limits()
