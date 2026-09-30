#!/usr/bin/env python3
"""Generate the power-line conductor collisions of the hcaa_pylon_setup model.

The exported world collision mesh models the conductors as a coarse tube whose
surface lies 0.2-8.5 cm from the conductor centerline, which is wider than the
charger gripper slot and uneven along the line. This script

- writes meshes/V1_world_collisions_no_conductors.dae: the exported collision
  mesh without its conductor geometry (cableAssem), and
- replaces the generated block of model.sdf with one collision cylinder per
  conductors.yaml segment, of radius conductor_radius_m.

conductors.yaml is in the world frame; the cylinders are placed in the model
frame using the pose at which worlds/hca_full_pylon_setup.sdf includes the model.

Run from anywhere; rerun whenever conductors.yaml, the collision mesh or the
model pose changes.
"""

import math
import pathlib
import re
import xml.etree.ElementTree as ET

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "world_models" / "hcaa_pylon_setup"
WORLD = ROOT / "worlds" / "hca_full_pylon_setup.sdf"
SOURCE_MESH = MODEL_DIR / "meshes" / "V1_world_collisions_a.dae"
TARGET_MESH = MODEL_DIR / "meshes" / "V1_world_collisions_no_conductors.dae"
CONDUCTOR_MESH_NAME = "cableAssem"
BEGIN = "<!-- BEGIN generated conductor collisions (scripts/generate_conductor_collisions.py) -->"
END = "<!-- END generated conductor collisions -->"
COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"

SURFACE = """          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
            </friction>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
            </bounce>
            <contact>
              <collide_bitmask>1</collide_bitmask>
            </contact>
          </surface>"""


def model_offset():
    text = WORLD.read_text()
    match = re.search(
        r"<uri>model://hcaa_pylon_setup</uri>\s*<pose[^>]*>([^<]+)</pose>", text)
    if match is None:
        raise SystemExit(f"{WORLD}: no pose for the hcaa_pylon_setup include")
    x, y, z, roll, pitch, yaw = (float(value) for value in match.group(1).split())
    if abs(roll) + abs(pitch) + abs(yaw) > 1e-12:
        raise SystemExit("the hcaa_pylon_setup include is rotated; only translation is supported")
    return x, y, z


def strip_conductor_mesh():
    ET.register_namespace("", COLLADA_NS)
    tree = ET.parse(SOURCE_MESH)
    root = tree.getroot()
    ns = {"c": COLLADA_NS}
    removed = 0
    for parent in root.iter():
        for child in list(parent):
            tag = child.tag.split("}")[-1]
            if tag in ("node", "geometry") and child.get("name") == CONDUCTOR_MESH_NAME:
                parent.remove(child)
                removed += 1
    if removed != 2:
        raise SystemExit(f"expected one {CONDUCTOR_MESH_NAME} node and geometry, removed {removed}")
    if root.findall(".//c:library_geometries/c:geometry", ns) == []:
        raise SystemExit("no collision geometry left")
    tree.write(TARGET_MESH, xml_declaration=True, encoding="utf-8")


def cylinder(name, start, end, radius):
    dx, dy, dz = (end[i] - start[i] for i in range(3))
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # SDF rotates about fixed x, y, z; roll 0, then pitch and yaw point the
    # cylinder's z axis along the segment.
    pitch = math.acos(max(-1.0, min(1.0, dz / length)))
    yaw = math.atan2(dy, dx)
    centre = [(start[i] + end[i]) / 2.0 for i in range(3)]
    # Overlap neighbours by one radius so the joints leave no gap.
    return f"""      <collision name='{name}'>
        <pose>{centre[0]:.6f} {centre[1]:.6f} {centre[2]:.6f} 0 {pitch:.9f} {yaw:.9f}</pose>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length + 2.0 * radius:.6f}</length>
          </cylinder>
        </geometry>
{SURFACE}
      </collision>"""


def main():
    asset = yaml.safe_load((MODEL_DIR / "conductors.yaml").read_text())
    if asset.get("frame_id") != "world":
        raise SystemExit("conductors.yaml must be in the world frame")
    radius = float(asset["conductor_radius_m"])
    ox, oy, oz = model_offset()

    collisions = []
    for conductor in asset["conductors"]:
        samples = [(x - ox, y - oy, z - oz) for x, y, z in conductor["samples"]]
        for index, (start, end) in enumerate(zip(samples, samples[1:])):
            collisions.append(cylinder(f"{conductor['id']}_segment_{index:02d}", start, end, radius))

    sdf_path = MODEL_DIR / "model.sdf"
    sdf = sdf_path.read_text()
    begin, end = sdf.index(BEGIN), sdf.index(END)
    sdf = sdf[:begin + len(BEGIN)] + "\n" + "\n".join(collisions) + "\n      " + sdf[end:]
    sdf_path.write_text(sdf)
    strip_conductor_mesh()
    print(f"{len(collisions)} conductor collision cylinders (r={radius} m); wrote {TARGET_MESH.name}")


if __name__ == "__main__":
    main()
