#!/usr/bin/env python3
"""Create and validate the D4S powerline SLAM evaluation variant.

The production ``d4s_dc_drone`` model and its ``99999_gz_d4s_dc_drone``
airframe remain the source of truth.  This tool derives
``d4s_dc_drone_powerline_eval`` and ``99997_gz_d4s_dc_drone_powerline_eval``
from them with the sensor layout used for powerline SLAM development
(powerline_slam layout U0_F50_C20):

- Radar-U: the production upward mmWave radar (same mount, topics
  ``/sensor/mmwave/*`` and frame ``mmwave``) running the simulator-v2
  IWR6843AOP model (``AOP_FAST_POINT``) with profile ``RADAR_U_v1``;
- Radar-F: a second simulator-v2 radar tilted 50 deg forward from upward
  (topics ``/sensor/mmwave_forward/*``, frame ``mmwave_forward``) with profile
  ``RADAR_F_v1``, triggered 5.10112 ms after Radar-U;
- the cable camera tilted to 20 deg from upward, with ROS frame
  ``cable_camera``, plus an evaluator-only pylon segmentation camera with the
  same pose and intrinsics;
- pylon camera truth and ``/sensor/cable_camera/camera_info`` from the sensor
  plugin.

The radar profiles live in ``models/d4s_dc_drone_powerline_eval/radar`` and
the radar scene in ``world_models/hcaa_pylon_setup/radar``.  Radar returns
from pylons, terrain and clutter are part of ``/sensor/mmwave/points``, so the
variant changes runtime inputs and is selected explicitly
(``PX4_SIM_MODEL=gz_d4s_dc_drone_powerline_eval``); it never replaces the
production model.  Dynamics, collision, rotors and the other sensors stay
identical to production.

Run from this asset repository after changing the production model or
airframe:
  python3 scripts/create_powerline_eval_drone_variant.py
  python3 scripts/create_powerline_eval_drone_variant.py --check
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


ASSET_ROOT = Path(__file__).resolve().parents[1]
SOURCE_MODEL = ASSET_ROOT / "models" / "d4s_dc_drone"
SOURCE_AIRFRAME = ASSET_ROOT / "init.d-posix_airframes" / "99999_gz_d4s_dc_drone"
VARIANT_NAME = "d4s_dc_drone_powerline_eval"
VARIANT_MODEL = ASSET_ROOT / "models" / VARIANT_NAME
VARIANT_SDF = VARIANT_MODEL / "model.sdf"
VARIANT_CONFIG = VARIANT_MODEL / "model.config"
VARIANT_AIRFRAME = ASSET_ROOT / "init.d-posix_airframes" / f"99997_gz_{VARIANT_NAME}"

# Cable camera 20 deg from upward (powerline_slam layout C20).
CAMERA_POSE = "0 -0.215 0.3 0 -1.2217304763960306 0"
# Radar-F 50 deg forward from upward (powerline_slam layout F50).
RADAR_F_POSE = "0.105 -0.24 0.285 0.0 -0.6981317007977318 0.0"
RADAR_CONFIG_URI = f"model://{VARIANT_NAME}/radar"

PRODUCTION_CAMERA_POSE = "        <pose>0 -0.215 0.3 0 -1.571 0</pose>\n"
PRODUCTION_CAMERA_TOPIC = "        <topic>/sensor/cable_camera/image_raw</topic>\n"
RADAR_PLUGIN = re.compile(
    r'    <plugin filename="iii_drone_mmwave_conductor_sensor_plugin".*?</plugin>\n', re.S
)

SEMANTIC_CAMERA = f"""
      <!-- Evaluator-only, renderer-exact pylon labels.  Pose, rate, FOV,
           resolution and clipping are intentionally identical to cable_camera.
           The runtime graph must never subscribe to this topic. -->
      <sensor type="segmentation" name="pylon_semantic_camera">
        <pose>{CAMERA_POSE}</pose>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <topic>/simulation/ground_truth/cable_camera/pylon_semantic_raw</topic>
        <camera>
          <segmentation_type>semantic</segmentation_type>
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.02</near>
            <far>30000</far>
          </clip>
        </camera>
      </sensor>
"""

CAMERA_TRUTH_SETTINGS = (
    "      <camera_info_topic>/sensor/cable_camera/camera_info</camera_info_topic>\n"
    "      <pylon_semantic_topic>/simulation/ground_truth/cable_camera/pylon_semantic_raw/labels_map</pylon_semantic_topic>\n"
    "      <pylon_mask_topic>/simulation/ground_truth/cable_camera/pylon_instance_mask</pylon_mask_topic>\n"
    "      <pylon_asset_uri>model://hcaa_pylon_setup/pylons.yaml</pylon_asset_uri>\n"
)

RADAR_F_PLUGIN = f"""    <!-- Radar-F: simulator-v2 IWR6843AOP, 50 deg forward from upward. -->
    <plugin filename="iii_drone_mmwave_conductor_sensor_plugin" name="iii_drone::simulation::MmwaveConductorSensorPlugin">
      <link_name>base_link</link_name>
      <radar_model>AOP_FAST_POINT</radar_model>
      <radar_instance>mmwave_forward</radar_instance>
      <aop_config>{RADAR_CONFIG_URI}/RADAR_F.yaml</aop_config>
      <radar_seed>2</radar_seed>
      <topic>/sensor/mmwave_forward/points</topic>
      <full_topic>/sensor/mmwave_forward/points_full</full_topic>
      <label_topic>/simulation/ground_truth/mmwave_forward/conductor_labels</label_topic>
      <frame_id>mmwave_forward</frame_id>
      <camera_mask_topic>/simulation/ground_truth/cable_camera/conductor_instance_mask</camera_mask_topic>
      <camera_image_topic>/sensor/cable_camera/image_raw</camera_image_topic>
{CAMERA_TRUTH_SETTINGS}      <conductor_id_map_topic>/simulation/ground_truth/conductor_id_map</conductor_id_map_topic>
      <conductor_asset_uri>model://hcaa_pylon_setup/conductors.yaml</conductor_asset_uri>
      <sensor_pose>{RADAR_F_POSE}</sensor_pose>
      <camera_pose>{CAMERA_POSE}</camera_pose>
      <publish_camera>false</publish_camera>
      <publish_drone_state>false</publish_drone_state>
      <publish_static_geometry>false</publish_static_geometry>
      <schedule_offset_ms>5.10112</schedule_offset_ms>
      <peer_offset_ms>0.0</peer_offset_ms>
      <peer_active_ms>4.10112</peer_active_ms>
      <schedule_jitter_sigma_us>5.0</schedule_jitter_sigma_us>
    </plugin>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>Drones4Safety DC-Drone (powerline SLAM evaluation)</name>
  <version>1.0</version>
  <sdf version='1.9'>model.sdf</sdf>
  <description>D4S model with two simulator-v2 IWR6843AOP radars (upward and 50 deg forward), the cable camera at 20 deg from upward, and evaluator-only pylon truth, for powerline SLAM evaluation.</description>
</model>
"""


def replace_once(text: str, old: str, new: str) -> str:
    if text.count(old) != 1:
        raise RuntimeError(f"Expected exactly one occurrence of {old!r} in the production asset.")
    return text.replace(old, new, 1)


def render_radar_u_plugin(production_plugin: str) -> str:
    """Production radar block switched to simulator-v2 Radar-U."""
    block = replace_once(
        production_plugin,
        "      <link_name>base_link</link_name>\n",
        "      <link_name>base_link</link_name>\n"
        "      <radar_model>AOP_FAST_POINT</radar_model>\n"
        "      <radar_instance>mmwave</radar_instance>\n"
        f"      <aop_config>{RADAR_CONFIG_URI}/RADAR_U.yaml</aop_config>\n"
        "      <radar_seed>1</radar_seed>\n",
    )
    block = replace_once(
        block,
        "      <camera_image_topic>/sensor/cable_camera/image_raw</camera_image_topic>\n",
        "      <camera_image_topic>/sensor/cable_camera/image_raw</camera_image_topic>\n"
        + CAMERA_TRUTH_SETTINGS
        + "      <publish_camera_info>true</publish_camera_info>\n",
    )
    block = re.sub(
        r"      <camera_pose>[^<]*</camera_pose>\n",
        f"      <camera_pose>{CAMERA_POSE}</camera_pose>\n",
        block,
        count=1,
    )
    block = replace_once(
        block,
        "    </plugin>\n",
        "      <publish_camera>true</publish_camera>\n"
        "      <publish_drone_state>true</publish_drone_state>\n"
        "      <publish_static_geometry>true</publish_static_geometry>\n"
        "      <schedule_offset_ms>0.0</schedule_offset_ms>\n"
        "      <peer_offset_ms>5.10112</peer_offset_ms>\n"
        "      <peer_active_ms>12.3405</peer_active_ms>\n"
        "      <schedule_jitter_sigma_us>5.0</schedule_jitter_sigma_us>\n"
        "    </plugin>\n",
    )
    return (
        "    <!-- Radar-U: simulator-v2 IWR6843AOP on the production upward mount. -->\n"
        + block
    )


def render_variant_sdf() -> str:
    source = (SOURCE_MODEL / "model.sdf").read_text()
    source = replace_once(
        source, "<model name='d4s_dc_drone'>", f"<model name='{VARIANT_NAME}'>"
    )

    camera_start = source.find('<sensor type="camera" name="cable_camera">')
    if camera_start < 0:
        raise RuntimeError("Production model has no cable_camera sensor.")
    camera_end = source.index("</sensor>\n", camera_start) + len("</sensor>\n")
    camera = source[camera_start:camera_end]
    camera = replace_once(camera, PRODUCTION_CAMERA_POSE, f"        <pose>{CAMERA_POSE}</pose>\n")
    camera = replace_once(
        camera,
        PRODUCTION_CAMERA_TOPIC,
        PRODUCTION_CAMERA_TOPIC + "        <gz_frame_id>cable_camera</gz_frame_id>\n",
    )
    source = source[:camera_start] + camera + SEMANTIC_CAMERA + source[camera_end:]

    radar_plugins = RADAR_PLUGIN.findall(source)
    if len(radar_plugins) != 1:
        raise RuntimeError("Expected exactly one mmWave sensor plugin in the production model.")
    return replace_once(
        source,
        radar_plugins[0],
        render_radar_u_plugin(radar_plugins[0]) + "\n" + RADAR_F_PLUGIN,
    )


def render_variant_airframe() -> str:
    source = SOURCE_AIRFRAME.read_text()
    source = replace_once(
        source,
        "# @name Drones4Safety DC drone model\n",
        "# @name Drones4Safety DC drone (powerline SLAM evaluation)\n",
    )
    return replace_once(
        source,
        "PX4_SIM_MODEL=${PX4_SIM_MODEL:=d4s_dc_drone}\n",
        f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:={VARIANT_NAME}}}\n",
    )


# ---------------------------------------------------------------------------
# Sensor timing profiles (powerline_slam FUSION30 phase-stress simulation).
#
# The evaluation variant above is the nominal profile and is not changed by
# anything below.  Each timing profile is one more model, selected explicitly
# like the nominal one (PX4_SIM_MODEL=gz_d4s_dc_drone_powerline_eval_<profile>),
# that differs from it only in *when* the radars sample relative to the camera.
#
# The cable camera and the evaluator pylon camera are Gazebo rendering sensors:
# they sample at multiples of their 100 ms period on the simulation clock,
# deterministically, and Gazebo offers no phase offset for them.  (Triggered
# cameras were measured and rejected: the trigger is delivered asynchronously,
# 4-5 % of the frames came one or more simulation steps off schedule and the
# two cameras lost their common stamp in half of the frames.)  The camera phase
# is therefore the fixed reference, 0, and a profile places the two radars on
# the shared frame clock around it: only the radars' trigger offsets, the
# matching peer windows and the trigger jitter change.  Radar profiles, plugin,
# mounts, the cameras and everything else are those of the nominal model.
#
# Frame period.  The frozen radar profiles give the frame period as 33.333 ms,
# three of which are 1 us short of the camera's 100 ms: on the nominal model
# the radars drift 10 us/s against the camera, a millisecond every 100 s of
# simulation time, so no phase relation holds for a flight.  A *locked* profile
# therefore uses copies of the two radar configurations in which the frame
# period is exactly one third of the camera period (33.333333... ms, 10 ppm
# longer; nothing else differs), which keeps its phases for the whole run.  The
# drift profile keeps the nominal 33.333 ms.
#
# Radar-U is active for 4.10112 ms and Radar-F for 12.3405 ms of the 33.333 ms
# frame (radar/RF_COEXISTENCE_SCHEDULE.json, guard 1.0 ms).  A profile is
# RF-valid when both guards between the two active windows, after the worst
# trigger jitter, are at least 1.0 ms.
RADAR_FRAME_MS = 33.333
RADAR_U_ACTIVE_MS = 4.10112
RADAR_F_ACTIVE_MS = 12.3405
RF_GUARD_MS = 1.0
CAMERA_PERIOD_MS = 100.0
LOCKED_RADAR_FRAME_MS = CAMERA_PERIOD_MS / 3.0
NOMINAL_JITTER_US = 5.0
NOMINAL_PERIOD_LINE = "  frame_period_ms: 33.333\n"

TIMING_PROFILES = {
    # name: airframe id, Radar-U and Radar-F frame-start offsets (ms after the camera phase), trigger jitter sigma (us)
    "aligned": {
        "airframe": 99993, "radar_u_offset_ms": 0.0, "radar_f_offset_ms": 0.0, "jitter_us": NOMINAL_JITTER_US,
        "interference": False, "locked": True,
        "description": "all three sensors on one phase (control; the radars' active windows overlap, so this profile is "
                       "not RF-valid and the mutual-interference model is switched off)",
    },
    "halfshift": {
        "airframe": 99994, "radar_u_offset_ms": 2.0 * RADAR_FRAME_MS / 3.0, "radar_f_offset_ms": 5.0 * RADAR_FRAME_MS / 6.0,
        "jitter_us": NOMINAL_JITTER_US, "interference": True, "locked": True,
        "description": "half of the maximum separation: Radar-F one sixth of a frame after Radar-U, the camera one third "
                       "of a frame after Radar-U",
    },
    "maxphase": {
        "airframe": 99995, "radar_u_offset_ms": RADAR_FRAME_MS / 3.0, "radar_f_offset_ms": 2.0 * RADAR_FRAME_MS / 3.0,
        "jitter_us": NOMINAL_JITTER_US, "interference": True, "locked": True,
        "description": "maximum phase separation: camera, Radar-U and Radar-F one third of a frame apart",
    },
    "maxphase_drift": {
        "airframe": 99992, "radar_u_offset_ms": RADAR_FRAME_MS / 3.0, "radar_f_offset_ms": 2.0 * RADAR_FRAME_MS / 3.0,
        "jitter_us": 300.0, "interference": True, "locked": False,
        "description": "maximum phase separation at simulation time zero with the nominal 33.333 ms radar frame (the radars "
                       "drift 10 us/s against the camera) and a deterministic bounded trigger jitter (sigma 300 us, clamped "
                       "to +-4 sigma) on both radars",
    },
}


def profile_name(profile: str) -> str:
    return f"{VARIANT_NAME}_{profile}"


def _circular(a: float, b: float, frame: float = RADAR_FRAME_MS) -> float:
    d = abs(a - b) % frame
    return min(d, frame - d)


def profile_schedule(profile: str) -> dict:
    """The profile's source-time and RF schedule; raises when an RF-valid profile violates the guard."""
    p = TIMING_PROFILES[profile]
    frame = LOCKED_RADAR_FRAME_MS if p["locked"] else RADAR_FRAME_MS
    u0, f0 = p["radar_u_offset_ms"], p["radar_f_offset_ms"]
    u = (u0, u0 + RADAR_U_ACTIVE_MS)
    f = (f0, f0 + RADAR_F_ACTIVE_MS)
    guards = ((f[0] - u[1]) % frame, (u[0] - f[1]) % frame)
    overlap = (f0 - u0) % frame < RADAR_U_ACTIVE_MS or (u0 - f0) % frame < RADAR_F_ACTIVE_MS
    worst_jitter_ms = 2 * 4.0 * p["jitter_us"] * 1e-3
    rf_valid = (not overlap) and min(guards) - worst_jitter_ms >= RF_GUARD_MS
    if p["interference"] and not rf_valid:
        raise RuntimeError(f"timing profile {profile} violates the {RF_GUARD_MS} ms RF guard: {guards}")
    separations = {"camera_radar_u": _circular(0.0, u0, frame), "camera_radar_f": _circular(0.0, f0, frame),
                   "radar_u_radar_f": _circular(u0, f0, frame)}
    return {
        "profile": profile, "model": profile_name(profile), "description": p["description"],
        "radar_frame_period_ms": frame, "radar_frame_period_locked_to_camera": bool(p["locked"]),
        "radar_drift_against_camera_us_per_s": 0.0 if p["locked"] else (CAMERA_PERIOD_MS - 3.0 * RADAR_FRAME_MS) / CAMERA_PERIOD_MS * 1e6,
        "camera_period_ms": CAMERA_PERIOD_MS,
        "camera": {"offset_ms": 0.0, "control": "Gazebo rendering sensor, frames at multiples of the camera period (the phase reference)"},
        "radar_u": {"offset_ms": u0, "active_window_ms": list(u)},
        "radar_f": {"offset_ms": f0, "active_window_ms": list(f)},
        "circular_separation_ms": separations, "minimum_circular_separation_ms": min(separations.values()),
        "maximum_possible_minimum_separation_ms": frame / 3.0,
        "trigger_jitter_sigma_us": p["jitter_us"], "trigger_jitter_bound_us": 4.0 * p["jitter_us"],
        "rf_guards_ms": list(guards), "rf_guard_required_ms": RF_GUARD_MS,
        "rf_guard_after_worst_jitter_ms": min(guards) - worst_jitter_ms, "rf_active_windows_overlap": overlap, "rf_valid": rf_valid,
        "mutual_interference_model": p["interference"],
        "sampling": "each sensor samples at the first simulation step at or after its frame start; the world step "
                    "quantizes the realized stamps",
    }


def _ms(value: float) -> str:
    return repr(round(float(value), 9))


def render_profile_sdf(profile: str) -> str:
    p = TIMING_PROFILES[profile]
    source = render_variant_sdf()
    source = replace_once(source, f"<model name='{VARIANT_NAME}'>", f"<model name='{profile_name(profile)}'>")
    if p["locked"]:                       # the profile's own radar configurations (exact frame period)
        for name in ("RADAR_U.yaml", "RADAR_F.yaml"):
            source = replace_once(source, f"<aop_config>{RADAR_CONFIG_URI}/{name}</aop_config>",
                                  f"<aop_config>model://{profile_name(profile)}/radar/{name}</aop_config>")
    f_active = RADAR_F_ACTIVE_MS if p["interference"] else 0.0
    u_active = RADAR_U_ACTIVE_MS if p["interference"] else 0.0
    source = replace_once(
        source,
        "      <schedule_offset_ms>0.0</schedule_offset_ms>\n"
        "      <peer_offset_ms>5.10112</peer_offset_ms>\n"
        "      <peer_active_ms>12.3405</peer_active_ms>\n"
        "      <schedule_jitter_sigma_us>5.0</schedule_jitter_sigma_us>\n",
        f"      <schedule_offset_ms>{_ms(p['radar_u_offset_ms'])}</schedule_offset_ms>\n"
        f"      <peer_offset_ms>{_ms(p['radar_f_offset_ms'])}</peer_offset_ms>\n"
        f"      <peer_active_ms>{_ms(f_active)}</peer_active_ms>\n"
        f"      <schedule_jitter_sigma_us>{_ms(p['jitter_us'])}</schedule_jitter_sigma_us>\n",
    )
    return replace_once(
        source,
        "      <schedule_offset_ms>5.10112</schedule_offset_ms>\n"
        "      <peer_offset_ms>0.0</peer_offset_ms>\n"
        "      <peer_active_ms>4.10112</peer_active_ms>\n"
        "      <schedule_jitter_sigma_us>5.0</schedule_jitter_sigma_us>\n",
        f"      <schedule_offset_ms>{_ms(p['radar_f_offset_ms'])}</schedule_offset_ms>\n"
        f"      <peer_offset_ms>{_ms(p['radar_u_offset_ms'])}</peer_offset_ms>\n"
        f"      <peer_active_ms>{_ms(u_active)}</peer_active_ms>\n"
        f"      <schedule_jitter_sigma_us>{_ms(p['jitter_us'])}</schedule_jitter_sigma_us>\n",
    )


def render_profile_airframe(profile: str) -> str:
    source = render_variant_airframe()
    source = replace_once(
        source,
        "# @name Drones4Safety DC drone (powerline SLAM evaluation)\n",
        f"# @name Drones4Safety DC drone (powerline SLAM evaluation, sensor timing profile {profile})\n",
    )
    return replace_once(
        source,
        f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:={VARIANT_NAME}}}\n",
        f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:={profile_name(profile)}}}\n",
    )


def render_profile_config(profile: str) -> str:
    return replace_once(
        MODEL_CONFIG,
        "  <name>Drones4Safety DC-Drone (powerline SLAM evaluation)</name>\n",
        f"  <name>Drones4Safety DC-Drone (powerline SLAM evaluation, timing profile {profile})</name>\n",
    )


def profile_files(profile: str) -> dict[Path, str]:
    model = ASSET_ROOT / "models" / profile_name(profile)
    airframe = ASSET_ROOT / "init.d-posix_airframes" / f"{TIMING_PROFILES[profile]['airframe']}_gz_{profile_name(profile)}"
    files = {
        model / "model.sdf": render_profile_sdf(profile),
        model / "model.config": render_profile_config(profile),
        model / "TIMING_PROFILE.json": json.dumps(profile_schedule(profile), indent=2, sort_keys=True) + "\n",
        airframe: render_profile_airframe(profile),
    }
    if TIMING_PROFILES[profile]["locked"]:
        for name in ("RADAR_U.yaml", "RADAR_F.yaml"):
            nominal = (VARIANT_MODEL / "radar" / name).read_text()
            files[model / "radar" / name] = replace_once(nominal, NOMINAL_PERIOD_LINE,
                                                         f"  frame_period_ms: {LOCKED_RADAR_FRAME_MS!r}\n")
    return files


# ---------------------------------------------------------------------------
# Long-operation profiles (powerline_slam WO-2026-10-08-001).
#
# The evaluation models carry a truth-only segmentation sensor
# (pylon_semantic_camera) for the evaluator's pylon labels.  With it the Gazebo
# render thread costs more per simulated second the longer the simulation runs
# (stock gz-rendering; the real-time factor falls from 0.95 to 0.61 in an hour).
# A long-operation profile is one more model, selected explicitly like every
# other, that is its base timing profile without that one sensor: the radars,
# the RGB camera, the IMU and every other vehicle sensor, their poses, rates,
# source-time schedule, RF schedule, radar configurations and noise are those
# of the base profile.  The radar plugin is told that there is no segmentation
# source (an empty pylon_semantic_topic), so it creates no pylon-frame truth
# topic.  A long-operation profile carries no pixel-exact pylon truth and is
# not a model for evaluator scoring that needs it; the evaluation models above
# are not changed by anything below.
LONG_OPERATION_PROFILES = {
    # name: base timing profile, airframe id
    "maxphase_longrun": {"base": "maxphase", "airframe": 99991},
}
PYLON_SEMANTIC_SETTING = (
    "      <pylon_semantic_topic>/simulation/ground_truth/cable_camera/pylon_semantic_raw/labels_map</pylon_semantic_topic>\n"
)
NO_PYLON_SEMANTIC_SETTING = "      <pylon_semantic_topic></pylon_semantic_topic>\n"


def render_long_operation_sdf(profile: str) -> str:
    base = LONG_OPERATION_PROFILES[profile]["base"]
    source = render_profile_sdf(base)
    source = replace_once(source, f"<model name='{profile_name(base)}'>", f"<model name='{profile_name(profile)}'>")
    source = replace_once(source, SEMANTIC_CAMERA, "")
    if PYLON_SEMANTIC_SETTING not in source:
        raise RuntimeError("The base profile names no pylon segmentation topic.")
    source = source.replace(PYLON_SEMANTIC_SETTING, NO_PYLON_SEMANTIC_SETTING)   # both radar plugin instances carry the setting
    if TIMING_PROFILES[base]["locked"]:   # the profile's own copies of the base profile's radar configurations
        for name in ("RADAR_U.yaml", "RADAR_F.yaml"):
            source = replace_once(source, f"<aop_config>model://{profile_name(base)}/radar/{name}</aop_config>",
                                  f"<aop_config>model://{profile_name(profile)}/radar/{name}</aop_config>")
    return source


def long_operation_files(profile: str) -> dict[Path, str]:
    base = LONG_OPERATION_PROFILES[profile]["base"]
    model = ASSET_ROOT / "models" / profile_name(profile)
    airframe = ASSET_ROOT / "init.d-posix_airframes" / f"{LONG_OPERATION_PROFILES[profile]['airframe']}_gz_{profile_name(profile)}"
    schedule = dict(profile_schedule(base), profile=profile, model=profile_name(profile), base_profile=base,
                    long_operation={"omitted_sensor": "pylon_semantic_camera", "pylon_frame_truth": False})
    base_airframe = render_profile_airframe(base)
    files = {
        model / "model.sdf": render_long_operation_sdf(profile),
        model / "model.config": replace_once(
            render_profile_config(base), f"timing profile {base})</name>", f"long-operation profile {profile})</name>"),
        model / "TIMING_PROFILE.json": json.dumps(schedule, indent=2, sort_keys=True) + "\n",
        airframe: replace_once(
            replace_once(base_airframe, f"sensor timing profile {base})", f"long-operation profile {profile})"),
            f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:={profile_name(base)}}}\n", f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:={profile_name(profile)}}}\n"),
    }
    if TIMING_PROFILES[base]["locked"]:
        for name in ("RADAR_U.yaml", "RADAR_F.yaml"):
            files[model / "radar" / name] = profile_files(base)[ASSET_ROOT / "models" / profile_name(base) / "radar" / name]
    return files


# ---------------------------------------------------------------------------
# Camera-mount profiles (powerline_slam WO-2026-10-09-001).
#
# A camera-mount profile is one more model, selected explicitly like every
# other, that is its base model (a timing profile or a long-operation profile)
# with the cable camera at another pitch about the same mount point.  The RGB
# camera, the evaluator-only pylon segmentation camera of an evaluation base
# and the camera pose both radar plugins use for the evaluator's camera truth
# all carry the one new pose; the camera intrinsics, both radars with their
# mounts, configurations, source-time and RF schedule, every other sensor and
# the vehicle are the base model's.  The models above (camera 20 deg from
# upward, layout C20) are not changed by anything below.
C25_CAMERA_POSE = "0 -0.215 0.3 0 -1.1344640137963142 0"     # 25 deg forward from upward (powerline_slam layout C25)
CAMERA_MOUNT_PROFILES = {
    # name: base model profile, airframe id, camera pose, mount label
    "c25_maxphase": {"base": "maxphase", "airframe": 99990, "camera_pose": C25_CAMERA_POSE, "mount": "C25"},
    "c25_maxphase_drift": {"base": "maxphase_drift", "airframe": 99989, "camera_pose": C25_CAMERA_POSE, "mount": "C25"},
    "c25_maxphase_longrun": {"base": "maxphase_longrun", "airframe": 99988, "camera_pose": C25_CAMERA_POSE, "mount": "C25"},
}


def _base_files(base: str) -> dict[Path, str]:
    return long_operation_files(base) if base in LONG_OPERATION_PROFILES else profile_files(base)


def _camera_pose_count(base: str) -> int:
    # the RGB sensor and the pose in each of the two radar plugins, plus the segmentation sensor of an evaluation base
    return 3 if base in LONG_OPERATION_PROFILES else 4


def render_camera_mount_sdf(profile: str) -> str:
    spec = CAMERA_MOUNT_PROFILES[profile]
    base = spec["base"]
    source = _base_files(base)[ASSET_ROOT / "models" / profile_name(base) / "model.sdf"]
    source = replace_once(source, f"<model name='{profile_name(base)}'>", f"<model name='{profile_name(profile)}'>")
    if source.count(CAMERA_POSE) != _camera_pose_count(base):
        raise RuntimeError(f"camera-mount profile {profile}: the base model carries the camera pose {source.count(CAMERA_POSE)} times.")
    source = source.replace(CAMERA_POSE, spec["camera_pose"])
    own_radar = f"model://{profile_name(base)}/radar/"
    if own_radar in source:               # a base with its own radar configurations: this profile's own copies of them
        source = source.replace(own_radar, f"model://{profile_name(profile)}/radar/")
    return source


def camera_mount_files(profile: str) -> dict[Path, str]:
    spec = CAMERA_MOUNT_PROFILES[profile]
    base = spec["base"]
    base_files = _base_files(base)
    base_model = ASSET_ROOT / "models" / profile_name(base)
    model = ASSET_ROOT / "models" / profile_name(profile)
    base_airframe = next(text for path, text in base_files.items() if path.parent.name == "init.d-posix_airframes")
    schedule = dict(json.loads(base_files[base_model / "TIMING_PROFILE.json"]), profile=profile, model=profile_name(profile),
                    camera_mount={"label": spec["mount"], "base_model": profile_name(base), "pose_xyz_rpy": spec["camera_pose"],
                                  "base_pose_xyz_rpy": CAMERA_POSE})
    config = base_files[base_model / "model.config"]
    name_end = config.index(")</name>")
    airframe_name_end = base_airframe.index(")\n", base_airframe.index("# @name "))
    files = {
        model / "model.sdf": render_camera_mount_sdf(profile),
        model / "model.config": config[:name_end] + f", camera mount {spec['mount']}" + config[name_end:],
        model / "TIMING_PROFILE.json": json.dumps(schedule, indent=2, sort_keys=True) + "\n",
        ASSET_ROOT / "init.d-posix_airframes" / f"{spec['airframe']}_gz_{profile_name(profile)}": replace_once(
            base_airframe[:airframe_name_end] + f", camera mount {spec['mount']}" + base_airframe[airframe_name_end:],
            f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:={profile_name(base)}}}\n", f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:={profile_name(profile)}}}\n"),
    }
    for path, text in base_files.items():
        if path.parent == base_model / "radar":
            files[model / "radar" / path.name] = text
    return files


def rendered_files() -> dict[Path, str]:
    files = {
        VARIANT_SDF: render_variant_sdf(),
        VARIANT_CONFIG: MODEL_CONFIG,
        VARIANT_AIRFRAME: render_variant_airframe(),
    }
    for profile in TIMING_PROFILES:
        files.update(profile_files(profile))
    for profile in LONG_OPERATION_PROFILES:
        files.update(long_operation_files(profile))
    for profile in CAMERA_MOUNT_PROFILES:
        files.update(camera_mount_files(profile))
    return files


def write_variant() -> None:
    for path, text in rendered_files().items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        if path.parent.name == "init.d-posix_airframes":
            path.chmod(SOURCE_AIRFRAME.stat().st_mode)
        print(f"Wrote {path.relative_to(ASSET_ROOT)}")


def check_variant() -> None:
    for path, text in rendered_files().items():
        if not path.is_file():
            raise RuntimeError(f"{path.relative_to(ASSET_ROOT)} is missing; run without --check.")
        if path.read_text() != text:
            raise RuntimeError(
                f"{path.relative_to(ASSET_ROOT)} is stale; regenerate it without --check."
            )
    model = ET.parse(VARIANT_SDF).getroot().find(".//model")
    if model is None or model.get("name") != VARIANT_NAME:
        raise RuntimeError("Variant SDF has an unexpected model name.")
    for name in ("RADAR_U.yaml", "RADAR_F.yaml"):
        if not (VARIANT_MODEL / "radar" / name).is_file():
            raise RuntimeError(f"Radar configuration radar/{name} is missing.")
    scene = ASSET_ROOT / "world_models" / "hcaa_pylon_setup" / "radar" / "scene_scatterers_r22_v1.json"
    if not scene.is_file():
        raise RuntimeError("Radar scene world_models/hcaa_pylon_setup/radar is missing.")
    nominal = ET.parse(VARIANT_SDF).getroot()
    cameras = lambda root: [ET.tostring(x) for x in root.iter("sensor") if x.get("name") in ("cable_camera", "pylon_semantic_camera")]  # noqa: E731
    for profile in TIMING_PROFILES:
        root = ET.parse(ASSET_ROOT / "models" / profile_name(profile) / "model.sdf").getroot()
        if cameras(root) != cameras(nominal) or len(cameras(root)) != 2:
            raise RuntimeError(f"timing profile {profile}: the cameras must be those of the nominal model.")
        profile_schedule(profile)
    for profile, spec in LONG_OPERATION_PROFILES.items():
        # a long-operation profile is its base profile without the segmentation sensor: nothing else may differ
        base_text = (ASSET_ROOT / "models" / profile_name(spec["base"]) / "model.sdf").read_text()
        text = (ASSET_ROOT / "models" / profile_name(profile) / "model.sdf").read_text()
        expected = base_text.replace(profile_name(spec["base"]), profile_name(profile)).replace(SEMANTIC_CAMERA, "").replace(
            PYLON_SEMANTIC_SETTING, NO_PYLON_SEMANTIC_SETTING)
        if text != expected or SEMANTIC_CAMERA not in base_text or PYLON_SEMANTIC_SETTING not in base_text:
            raise RuntimeError(f"long-operation profile {profile}: the model must be {spec['base']} without the segmentation sensor only.")
        root = ET.parse(ASSET_ROOT / "models" / profile_name(profile) / "model.sdf").getroot()
        base_root = ET.parse(ASSET_ROOT / "models" / profile_name(spec["base"]) / "model.sdf").getroot()
        sensors = lambda r: {x.get("name"): ET.tostring(x).strip() for x in r.iter("sensor")}  # noqa: E731  (without the tail whitespace)
        kept = {k: v for k, v in sensors(base_root).items() if k != "pylon_semantic_camera"}
        if sensors(root) != kept or "pylon_semantic_camera" in sensors(root):
            raise RuntimeError(f"long-operation profile {profile}: every sensor but pylon_semantic_camera must be the base profile's.")
        for name in ("RADAR_U.yaml", "RADAR_F.yaml"):
            if TIMING_PROFILES[spec["base"]]["locked"] and (ASSET_ROOT / "models" / profile_name(profile) / "radar" / name).read_bytes() != (
                    ASSET_ROOT / "models" / profile_name(spec["base"]) / "radar" / name).read_bytes():
                raise RuntimeError(f"long-operation profile {profile}: radar/{name} must be the base profile's.")
    for profile, spec in CAMERA_MOUNT_PROFILES.items():
        # a camera-mount profile is its base model with the camera pose replaced: nothing else may differ
        base = spec["base"]
        base_text = (ASSET_ROOT / "models" / profile_name(base) / "model.sdf").read_text()
        text = (ASSET_ROOT / "models" / profile_name(profile) / "model.sdf").read_text()
        if base_text.count(CAMERA_POSE) != _camera_pose_count(base) or spec["camera_pose"] in base_text or CAMERA_POSE in text:
            raise RuntimeError(f"camera-mount profile {profile}: unexpected camera poses in the base model or the profile.")
        if text != base_text.replace(profile_name(base), profile_name(profile)).replace(CAMERA_POSE, spec["camera_pose"]):
            raise RuntimeError(f"camera-mount profile {profile}: the model must be {base} with the camera pose replaced only.")
        root = ET.parse(ASSET_ROOT / "models" / profile_name(profile) / "model.sdf").getroot()
        base_root = ET.parse(ASSET_ROOT / "models" / profile_name(base) / "model.sdf").getroot()

        def by_name(r, without_pose):  # noqa: ANN001
            out = {}
            for sensor in r.iter("sensor"):
                body = ET.tostring(sensor).decode().strip()
                out[sensor.get("name")] = body.replace(f"<pose>{without_pose}</pose>", "<pose/>") if sensor.get("name") in (
                    "cable_camera", "pylon_semantic_camera") else body
            return out
        if by_name(root, spec["camera_pose"]) != by_name(base_root, CAMERA_POSE):
            raise RuntimeError(f"camera-mount profile {profile}: every sensor but the camera pose must be the base model's.")
        poses = [x.findtext("pose") for x in root.iter("sensor") if x.get("name") in ("cable_camera", "pylon_semantic_camera")]
        poses += [x.findtext("camera_pose") for x in root.iter("plugin") if x.find("camera_pose") is not None]
        if len(poses) != _camera_pose_count(base) or set(poses) != {spec["camera_pose"]}:
            raise RuntimeError(f"camera-mount profile {profile}: the camera sensors and both radar plugins must carry the one new pose.")
        for path in sorted((ASSET_ROOT / "models" / profile_name(base) / "radar").glob("*.yaml")):
            if (ASSET_ROOT / "models" / profile_name(profile) / "radar" / path.name).read_bytes() != path.read_bytes():
                raise RuntimeError(f"camera-mount profile {profile}: radar/{path.name} must be the base model's.")
    print(f"{VARIANT_NAME}, its timing profiles, its long-operation profiles and its camera-mount profiles match the production model and airframe")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="validate existing generated assets")
    args = parser.parse_args()
    if args.check:
        check_variant()
    else:
        write_variant()


if __name__ == "__main__":
    main()
