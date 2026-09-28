#!/usr/bin/env python3
"""Create and validate the display-only D4S paper-model variant.

The production ``d4s_dc_drone`` model remains the source of truth.  This
tool creates ``d4s_dc_drone_paper`` by copying its SDF configuration and
filtering only the disconnected mesh components that make up the side reel,
its guard loops, and their display-only mounting hardware.  Dynamics,
collision, sensors, and rotors deliberately continue to use the original
definitions, so the paper variant is a visual change only.

Run from this asset repository after changing the source mesh:
  python3 scripts/create_paper_drone_variant.py
  python3 scripts/create_paper_drone_variant.py --check
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


ASSET_ROOT = Path(__file__).resolve().parents[1]
SOURCE_MODEL = ASSET_ROOT / "models" / "d4s_dc_drone"
PAPER_MODEL = ASSET_ROOT / "models" / "d4s_dc_drone_paper"
SOURCE_MESH = SOURCE_MODEL / "meshes" / "tarot650_model_v2_smaller.dae"
PAPER_MESH = SOURCE_MODEL / "meshes" / "tarot650_model_v2_smaller_paper.dae"
PAPER_SDF = PAPER_MODEL / "model.sdf"
PAPER_CONFIG = PAPER_MODEL / "model.config"

# This protects the component IDs below from silently being applied to a
# differently exported COLLADA mesh.  Regenerate and review the selection if
# the production mesh ever changes.
SOURCE_MESH_SHA256 = "aeb0c6c2afa4187d5be40c4c7d32e2e189b7a7b9cb149e3fe077c09bef2916c3"

# Disconnected source-mesh roots for the annotated side cable drum, drum
# hardware, and the two long cable guard loops.  The selection was mapped from
# orthographic component renders of the supplied marked Gazebo screenshots.
REMOVED_COMPONENT_ROOT_IDS = frozenset(
    {
        11418,
        11551,
        11702,
        11857,
        11954,
        12101,
        12207,
        12477,
        12597,
        12673,
        12716,
        12793,
        12903,
        13068,
        13148,
        13247,
        13406,
        13771,
        13938,
        14075,
        15697,
        15791,
        15847,
        15929,
        18059,
        30221,
        30139,
    }
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def local_name(element: ET.Element) -> str:
    return element.tag.rsplit("}", 1)[-1]


def position_count(root: ET.Element) -> int:
    positions = next(
        element
        for element in root.iter()
        if local_name(element) == "float_array"
        and "positions-array" in element.attrib.get("id", "")
    )
    return len(np.fromstring(positions.text or "", sep=" ")) // 3


def triangle_blocks(root: ET.Element):
    """Yield (triangles element, vertex-index faces, index stride)."""
    for triangles in root.iter():
        if local_name(triangles) != "triangles":
            continue
        children = list(triangles)
        inputs = [child for child in children if local_name(child) == "input"]
        indices = next((child for child in children if local_name(child) == "p"), None)
        if not inputs or indices is None:
            continue
        stride = max(int(item.attrib["offset"]) for item in inputs) + 1
        vertex_offset = next(
            int(item.attrib["offset"])
            for item in inputs
            if item.attrib["semantic"] == "VERTEX"
        )
        raw = np.fromstring(indices.text or "", sep=" ", dtype=np.int64)
        faces = raw.reshape(-1, 3, stride)
        yield triangles, indices, faces, faces[:, :, vertex_offset]


def component_roots(root: ET.Element) -> np.ndarray:
    parent = np.arange(position_count(root), dtype=np.int64)

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = int(parent[index])
        return index

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    for _triangles, _indices, _faces, vertices in triangle_blocks(root):
        for face in vertices:
            union(int(face[0]), int(face[1]))
            union(int(face[1]), int(face[2]))
    return np.fromiter((find(index) for index in range(len(parent))), dtype=np.int64)


def filtered_mesh_tree() -> tuple[ET.ElementTree, int, int]:
    if sha256(SOURCE_MESH) != SOURCE_MESH_SHA256:
        raise RuntimeError(
            "The production DAE hash changed; component roots must be remapped "
            "before generating the paper variant."
        )

    tree = ET.parse(SOURCE_MESH)
    roots = component_roots(tree.getroot())
    before = removed = 0
    for triangles, indices, faces, vertices in triangle_blocks(tree.getroot()):
        face_roots = roots[vertices[:, 0]]
        if not np.all(face_roots == roots[vertices[:, 1]]) or not np.all(
            face_roots == roots[vertices[:, 2]]
        ):
            raise RuntimeError("A triangle crosses disconnected-component roots.")
        keep = ~np.isin(face_roots, tuple(REMOVED_COMPONENT_ROOT_IDS))
        before += len(faces)
        removed += int((~keep).sum())
        triangles.set("count", str(int(keep.sum())))
        indices.text = " ".join(str(value) for value in faces[keep].reshape(-1))
    if removed == 0:
        raise RuntimeError("No paper-variant faces were removed.")
    return tree, before, removed


def render_variant_sdf() -> str:
    source = (SOURCE_MODEL / "model.sdf").read_text()
    source = source.replace(
        "<model name='d4s_dc_drone'>", "<model name='d4s_dc_drone_paper'>", 1
    )
    source = source.replace(
        "model://d4s_dc_drone/meshes/tarot650_model_v2_smaller.dae",
        "model://d4s_dc_drone/meshes/tarot650_model_v2_smaller_paper.dae",
        1,
    )
    if "d4s_dc_drone_paper" not in source:
        raise RuntimeError("Could not rename the paper model SDF.")
    return source


def write_variant() -> None:
    tree, before, removed = filtered_mesh_tree()
    PAPER_MODEL.mkdir(parents=True, exist_ok=True)
    ET.register_namespace("", "http://www.collada.org/2005/11/COLLADASchema")
    tree.write(PAPER_MESH, encoding="UTF-8", xml_declaration=True)
    PAPER_SDF.write_text(render_variant_sdf())
    PAPER_CONFIG.write_text(
        """<?xml version=\"1.0\"?>
<model>
  <name>Drones4Safety DC-Drone (paper)</name>
  <version>1.0</version>
  <sdf version='1.9'>model.sdf</sdf>
  <description>Display-only D4S model with side cable drum and guard removed.</description>
</model>
"""
    )
    print(f"Wrote {PAPER_MESH.relative_to(ASSET_ROOT)}: removed {removed}/{before} faces")
    print(f"Wrote {PAPER_SDF.relative_to(ASSET_ROOT)}")


def check_variant() -> None:
    if not PAPER_MESH.is_file() or not PAPER_SDF.is_file() or not PAPER_CONFIG.is_file():
        raise RuntimeError("Paper model is missing; run this script without --check.")
    _tree, before, expected_removed = filtered_mesh_tree()
    paper = ET.parse(PAPER_MESH).getroot()
    paper_faces = sum(len(faces) for _t, _i, faces, _v in triangle_blocks(paper))
    source_tree = ET.parse(SOURCE_MESH).getroot()
    source_faces = sum(len(faces) for _t, _i, faces, _v in triangle_blocks(source_tree))
    if source_faces != before or paper_faces != source_faces - expected_removed:
        raise RuntimeError("Paper mesh face count does not match the reviewed removal set.")
    sdf = ET.parse(PAPER_SDF).getroot()
    if sdf.find(".//model").get("name") != "d4s_dc_drone_paper":
        raise RuntimeError("Paper SDF has an unexpected model name.")
    visual_uri = sdf.findtext(".//visual[@name='base_link_inertia_visual']//uri")
    if visual_uri != "model://d4s_dc_drone/meshes/tarot650_model_v2_smaller_paper.dae":
        raise RuntimeError("Paper SDF does not select the filtered airframe mesh.")
    print(f"Paper model is valid: removed {expected_removed}/{source_faces} airframe faces")


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
