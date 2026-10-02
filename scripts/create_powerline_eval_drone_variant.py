#!/usr/bin/env python3
"""Create and validate the D4S powerline SLAM evaluation variant.

The production ``d4s_dc_drone`` model and its ``99999_gz_d4s_dc_drone``
airframe remain the source of truth.  This tool derives
``d4s_dc_drone_powerline_eval`` and ``99997_gz_d4s_dc_drone_powerline_eval``
from them and adds only the powerline SLAM evaluation configuration:

- an evaluator-only semantic segmentation camera (``pylon_semantic_camera``)
  with the pose, rate, FOV, resolution and clipping of ``cable_camera``;
- mmWave sensor plugin settings for the pylon map
  (``model://hcaa_pylon_setup/pylons.yaml``), pylon radar returns, pylon
  camera truth, ``camera_info`` and the finite rectangular radar FOV used for
  powerline SLAM development.

Pylon radar returns are part of ``/sensor/mmwave/points`` in this variant, so
it is selected explicitly (``PX4_SIM_MODEL=gz_d4s_dc_drone_powerline_eval``)
and never replaces the production model.  Dynamics, collision, rotors and the
other sensors stay identical to production.

Run from this asset repository after changing the production model or
airframe:
  python3 scripts/create_powerline_eval_drone_variant.py
  python3 scripts/create_powerline_eval_drone_variant.py --check
"""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


ASSET_ROOT = Path(__file__).resolve().parents[1]
SOURCE_MODEL = ASSET_ROOT / "models" / "d4s_dc_drone"
SOURCE_AIRFRAME = ASSET_ROOT / "init.d-posix_airframes" / "99999_gz_d4s_dc_drone"
VARIANT_NAME = "d4s_dc_drone_powerline_eval"
VARIANT_MODEL = ASSET_ROOT / "models" / VARIANT_NAME
VARIANT_SDF = VARIANT_MODEL / "model.sdf"
VARIANT_CONFIG = VARIANT_MODEL / "model.config"
VARIANT_AIRFRAME = ASSET_ROOT / "init.d-posix_airframes" / f"99997_gz_{VARIANT_NAME}"

SEMANTIC_CAMERA = """
      <!-- Evaluator-only, renderer-exact pylon labels.  Pose, rate, FOV,
           resolution and clipping are intentionally identical to cable_camera.
           The runtime graph must never subscribe to this topic. -->
      <sensor type="segmentation" name="pylon_semantic_camera">
        <pose>0 -0.215 0.3 0 -1.571 0</pose>
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

# (anchor line in the production mmWave plugin block, lines inserted after it)
PLUGIN_INSERTIONS = (
    (
        "      <camera_image_topic>/sensor/cable_camera/image_raw</camera_image_topic>\n",
        "      <camera_info_topic>/sensor/cable_camera/camera_info</camera_info_topic>\n"
        "      <publish_camera_info>true</publish_camera_info>\n"
        "      <pylon_semantic_topic>/simulation/ground_truth/cable_camera/pylon_semantic_raw/labels_map</pylon_semantic_topic>\n"
        "      <pylon_mask_topic>/simulation/ground_truth/cable_camera/pylon_instance_mask</pylon_mask_topic>\n"
        "      <pylon_asset_uri>model://hcaa_pylon_setup/pylons.yaml</pylon_asset_uri>\n"
        "      <pylon_returns_enabled>true</pylon_returns_enabled>\n",
    ),
    (
        "      <update_rate_hz>30</update_rate_hz>\n",
        "      <min_point_dist>0.25</min_point_dist>\n",
    ),
    (
        "      <max_point_dist>18.0</max_point_dist>\n",
        "      <fov_model>FINITE_RECTANGULAR</fov_model>\n"
        "      <azimuth_half_angle_rad>0.6107259643892086</azimuth_half_angle_rad>\n"
        "      <elevation_half_angle_rad>0.6107259643892086</elevation_half_angle_rad>\n",
    ),
)

MODEL_CONFIG = f"""<?xml version="1.0"?>
<model>
  <name>Drones4Safety DC-Drone (powerline SLAM evaluation)</name>
  <version>1.0</version>
  <sdf version='1.9'>model.sdf</sdf>
  <description>D4S model with evaluator-only pylon truth, pylon radar returns and the finite radar FOV used for powerline SLAM evaluation.</description>
</model>
"""


def replace_once(text: str, old: str, new: str) -> str:
    if text.count(old) != 1:
        raise RuntimeError(f"Expected exactly one occurrence of {old!r} in the production asset.")
    return text.replace(old, new, 1)


def render_variant_sdf() -> str:
    source = (SOURCE_MODEL / "model.sdf").read_text()
    source = replace_once(
        source, "<model name='d4s_dc_drone'>", f"<model name='{VARIANT_NAME}'>"
    )

    camera_start = source.find('<sensor type="camera" name="cable_camera">')
    if camera_start < 0:
        raise RuntimeError("Production model has no cable_camera sensor.")
    camera_end = source.index("</sensor>\n", camera_start) + len("</sensor>\n")
    source = source[:camera_end] + SEMANTIC_CAMERA + source[camera_end:]

    for anchor, inserted in PLUGIN_INSERTIONS:
        source = replace_once(source, anchor, anchor + inserted)
    return source


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
