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
  plugin;
- PX4's simulated magnetometer (``SENS_EN_MAGSIM``) instead of Gazebo's, as in
  the powerline_slam development runtime: the field of Gazebo's magnetometer
  as PX4 reads it has a declination of -3.1 deg at the world's location, while
  PX4's world magnetic model expects +4.3 deg, which biases PX4's heading by
  several degrees against truth.

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


def remove_magnetometer(source: str) -> str:
    """Drop Gazebo's magnetometer; PX4 simulates the field (SENS_EN_MAGSIM)."""
    start = source.find('      <sensor name="magnetometer_sensor" type="magnetometer">\n')
    if start < 0 or source.count('type="magnetometer"') != 1:
        raise RuntimeError("Expected exactly one magnetometer sensor in the production model.")
    end = source.index("      </sensor>\n", start) + len("      </sensor>\n")
    return source[:start] + source[end:]


def render_variant_sdf() -> str:
    source = (SOURCE_MODEL / "model.sdf").read_text()
    source = replace_once(
        source, "<model name='d4s_dc_drone'>", f"<model name='{VARIANT_NAME}'>"
    )
    source = remove_magnetometer(source)

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
    source = replace_once(
        source,
        "PX4_SIM_MODEL=${PX4_SIM_MODEL:=d4s_dc_drone}\n",
        f"PX4_SIM_MODEL=${{PX4_SIM_MODEL:={VARIANT_NAME}}}\n",
    )
    return replace_once(
        source,
        "param set-default SENS_EN_BAROSIM 1\n",
        "param set-default SENS_EN_BAROSIM 1\n"
        "# The model has no Gazebo magnetometer: PX4 simulates the field from its\n"
        "# world magnetic model, consistent with the declination its EKF assumes.\n"
        "param set-default SENS_EN_MAGSIM 1\n",
    )


def rendered_files() -> dict[Path, str]:
    return {
        VARIANT_SDF: render_variant_sdf(),
        VARIANT_CONFIG: MODEL_CONFIG,
        VARIANT_AIRFRAME: render_variant_airframe(),
    }


def write_variant() -> None:
    VARIANT_MODEL.mkdir(parents=True, exist_ok=True)
    for path, text in rendered_files().items():
        path.write_text(text)
        print(f"Wrote {path.relative_to(ASSET_ROOT)}")
    VARIANT_AIRFRAME.chmod(SOURCE_AIRFRAME.stat().st_mode)


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
    print(f"{VARIANT_NAME} matches the production model and airframe")


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
