#!/usr/bin/env python3
import argparse
import os
from pathlib import Path
import threading
import numpy as np
from inputs import UnpluggedError, get_gamepad

from cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.system.hardware import HARDWARE
from openpilot.tools.lib.kbhit import KBHit

EXPO = 0.4
DEFAULT_CONTROLLER = "auto"


def detect_controller_profile() -> str:
  if HARDWARE.get_device_type() == 'pc':
    return "ps5"

  by_id_path = Path("/dev/input/by-id")
  if by_id_path.exists():
    for entry in by_id_path.iterdir():
      name = entry.name.lower()
      if "054c:09cc" in name:
        return "ps4"
      if "sony" in name and ("0ce6" in name or "0df2" in name):
        return "ps5"

  proc_devices = Path("/proc/bus/input/devices")
  if proc_devices.exists():
    text = proc_devices.read_text(errors="ignore").lower()
    if "vendor=054c product=09cc" in text:
      return "ps4"

  return "ps5"


class Keyboard:
  def __init__(self):
    self.kb = KBHit()
    self.axis_increment = 0.05  # 5% of full actuation each key press
    self.axes_map = {'w': 'gb', 's': 'gb',
                     'a': 'steer', 'd': 'steer'}
    self.axes_values = {'gb': 0., 'steer': 0.}
    self.axes_order = ['gb', 'steer']
    self.cancel = False

  def update(self):
    key = self.kb.getch().lower()
    self.cancel = False
    if key == 'r':
      self.axes_values = dict.fromkeys(self.axes_values, 0.)
    elif key == 'c':
      self.cancel = True
    elif key in self.axes_map:
      axis = self.axes_map[key]
      incr = self.axis_increment if key in ['w', 'a'] else -self.axis_increment
      self.axes_values[axis] = float(np.clip(self.axes_values[axis] + incr, -1, 1))
    else:
      return False
    return True


class Joystick:
  def __init__(self, controller: str = DEFAULT_CONTROLLER, dump_events: bool = False):
    self.dump_events = dump_events
    self.last_dumped_states: dict[str, int] = {}
    self.dump_abs_threshold = 12
    self.controller = detect_controller_profile() if controller == "auto" else controller
    self.cancel_button = 'BTN_NORTH'  # BTN_NORTH=X/triangle

    if self.controller == "ps4":
      # Sony DualShock 4 on the comma exposes triggers on ABS_RX/ABS_RY and left stick steer on ABS_X.
      accel_axis = 'ABS_RY'
      steer_axis = 'ABS_X'
      self.flip_map = {'ABS_RX': accel_axis}
    elif HARDWARE.get_device_type() == 'pc':
      accel_axis = 'ABS_Z'
      steer_axis = 'ABS_RX'
      # TODO: once the longcontrol API is finalized, we can replace this with outputting gas/brake and steering
      self.flip_map = {'ABS_RZ': accel_axis}
    else:
      accel_axis = 'ABS_RX'
      steer_axis = 'ABS_Z'
      self.flip_map = {'ABS_RY': accel_axis}

    self.min_axis_value = {accel_axis: 0., steer_axis: 0.}
    self.max_axis_value = {accel_axis: 255., steer_axis: 255.}
    self.axes_values = {accel_axis: 0., steer_axis: 0.}
    self.axes_order = [accel_axis, steer_axis]
    self.cancel = False

  def update(self):
    try:
      joystick_events = get_gamepad()
    except (OSError, UnpluggedError):
      self.axes_values = dict.fromkeys(self.axes_values, 0.)
      return False

    handled = False
    for joystick_event in joystick_events:
      event = (joystick_event.code, joystick_event.state)

      if self.dump_events and self._should_dump_event(*event):
        print(f"raw event: code={event[0]} state={event[1]}")

      # flip left trigger to negative accel
      if event[0] in self.flip_map:
        event = (self.flip_map[event[0]], -event[1])

      if event[0] == self.cancel_button:
        if event[1] == 1:
          self.cancel = True
        elif event[1] == 0:   # state 0 is falling edge
          self.cancel = False
        handled = True
      elif event[0] in self.axes_values:
        self.max_axis_value[event[0]] = max(event[1], self.max_axis_value[event[0]])
        self.min_axis_value[event[0]] = min(event[1], self.min_axis_value[event[0]])

        norm = -float(np.interp(event[1], [self.min_axis_value[event[0]], self.max_axis_value[event[0]]], [-1., 1.]))
        norm = norm if abs(norm) > 0.03 else 0.  # center can be noisy, deadzone of 3%
        self.axes_values[event[0]] = EXPO * norm ** 3 + (1 - EXPO) * norm  # less action near center for fine control
        handled = True

    return handled

  def _should_dump_event(self, code: str, state: int) -> bool:
    if code in ('SYN_REPORT', 'MSC_SCAN'):
      return False

    prev = self.last_dumped_states.get(code)
    self.last_dumped_states[code] = state

    if prev is None:
      return True

    if code.startswith('ABS_'):
      return abs(state - prev) >= self.dump_abs_threshold

    return state != prev


def send_thread(joystick):
  pm = messaging.PubMaster(['testJoystick'])

  rk = Ratekeeper(100, print_delay_threshold=None)

  while True:
    if not getattr(joystick, "dump_events", False) and rk.frame % 20 == 0:
      print('\n' + ', '.join(f'{name}: {round(v, 3)}' for name, v in joystick.axes_values.items()))

    joystick_msg = messaging.new_message('testJoystick')
    joystick_msg.valid = True
    joystick_msg.testJoystick.axes = [joystick.axes_values[ax] for ax in joystick.axes_order]

    pm.send('testJoystick', joystick_msg)

    rk.keep_time()


def joystick_control_thread(joystick):
  Params().put_bool('JoystickDebugMode', True)
  threading.Thread(target=send_thread, args=(joystick,), daemon=True).start()
  while True:
    joystick.update()


def main():
  joystick_control_thread(Joystick())


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                               'openpilot must be offroad before starting joystick_control. This tool supports ' +
                                               'a PlayStation 5 DualSense controller on the comma 3X.',
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--keyboard', action='store_true', help='Use your keyboard instead of a joystick')
  parser.add_argument('--controller', choices=['auto', 'ps5', 'ps4'], default=DEFAULT_CONTROLLER, help='Joystick mapping profile to use')
  parser.add_argument('--dump-events', action='store_true', help='Print raw joystick event codes and states')
  args = parser.parse_args()

  if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ:
    print("The car must be off before running joystick_control.")
    exit()

  print()
  if args.keyboard:
    print('Gas/brake control: `W` and `S` keys')
    print('Steering control: `A` and `D` keys')
    print('Buttons')
    print('- `R`: Resets axes')
    print('- `C`: Cancel cruise control')
  else:
    print('Using joystick, make sure to run cereal/messaging/bridge on your device if running over the network!')
    print('If not running on a comma device, the mapping may need to be adjusted.')

  joystick = Keyboard() if args.keyboard else Joystick(controller=args.controller, dump_events=args.dump_events)
  if not args.keyboard:
    print(f'Controller profile: {joystick.controller}')
  joystick_control_thread(joystick)
