from __future__ import annotations

from typing import Any

from openpilot.common.swaglog import cloudlog


EXTERNAL_AUDIO_NAME_PARTS = (
  "wireless controller",
  "sony",
  "playstation",
  "dualshock",
  "dualsense",
  "usb audio",
  "headset",
)

PREFERRED_INTERNAL_NAME_PARTS = (
  "speaker",
  "builtin",
  "internal",
  "codec",
  "qcom",
  "msm",
  "sdm",
  "tegra",
  "rt5682",
)


def _default_device_index(default_device: Any, direction: str) -> int | None:
  if isinstance(default_device, (tuple, list)):
    idx = 0 if direction == "input" else 1
    if len(default_device) <= idx:
      return None
    default_device = default_device[idx]

  if default_device is None:
    return None

  try:
    default_idx = int(default_device)
  except (TypeError, ValueError):
    return None

  return default_idx if default_idx >= 0 else None


def _is_external_audio_name(name: str) -> bool:
  lowered = name.lower()
  return any(part in lowered for part in EXTERNAL_AUDIO_NAME_PARTS)


def _is_preferred_internal_name(name: str) -> bool:
  lowered = name.lower()
  return any(part in lowered for part in PREFERRED_INTERNAL_NAME_PARTS)


def _supports_direction(device_info: dict[str, Any], direction: str) -> bool:
  key = "max_input_channels" if direction == "input" else "max_output_channels"
  return int(device_info.get(key, 0)) > 0


def _check_stream_settings(sd, direction: str, device_index: int, samplerate: int, channels: int) -> bool:
  try:
    if direction == "input":
      sd.check_input_settings(device=device_index, channels=channels, samplerate=samplerate)
    else:
      sd.check_output_settings(device=device_index, channels=channels, samplerate=samplerate)
    return True
  except Exception:
    return False


def choose_sounddevice(sd, direction: str, samplerate: int, channels: int = 1) -> int | None:
  devices = list(sd.query_devices())
  default_idx = _default_device_index(sd.default.device, direction)

  candidate_groups: list[list[int]] = []
  if default_idx is not None and 0 <= default_idx < len(devices):
    candidate_groups.append([default_idx])

  candidate_groups.append([
    idx for idx, dev in enumerate(devices)
    if _supports_direction(dev, direction) and _is_preferred_internal_name(dev.get("name", "")) and not _is_external_audio_name(dev.get("name", ""))
  ])
  candidate_groups.append([
    idx for idx, dev in enumerate(devices)
    if _supports_direction(dev, direction) and not _is_external_audio_name(dev.get("name", ""))
  ])

  seen: set[int] = set()
  for group in candidate_groups:
    for idx in group:
      if idx in seen:
        continue
      seen.add(idx)

      dev = devices[idx]
      if not _supports_direction(dev, direction):
        continue

      if _check_stream_settings(sd, direction, idx, samplerate, channels):
        cloudlog.info(f"{direction} audio device selected: idx={idx} name={dev.get('name', '')!r} samplerate={samplerate}")
        return idx

  cloudlog.warning(f"no explicit {direction} audio device matched; falling back to PortAudio default")
  return None
