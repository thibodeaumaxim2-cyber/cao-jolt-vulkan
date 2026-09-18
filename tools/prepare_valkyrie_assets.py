#!/usr/bin/env python3
"""Convert NASA Valkyrie visual meshes into CAO's binary STL asset format.

The source is the pinned NASA-1.3 ``third_party/val_description`` submodule.
This tool intentionally converts visual geometry only; an articulated MJCF and
controller are separate validation work and must not replace H1 at runtime.
"""
import argparse
import json
from pathlib import Path
import struct

import collada
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "third_party/val_description/model/meshes"
DEFAULT_OUTPUT = ROOT / "assets/valkyrie/meshes"


def write_binary_stl(source: Path, output: Path) -> int:
    """Triangulate a Collada mesh and write a binary STL with computed normals."""
    document = collada.Collada(str(source))
    triangles = []
    # Build a transform table from the scene graph, but iterate every source
    # geometry.  Several Valkyrie DAEs contain auxiliary scene nodes; using
    # only scene.objects() can select a tiny helper geometry and omit the main
    # shell (most visibly the head and neck).
    transforms = {}
    for bound in document.scene.objects("geometry"):
        transforms.setdefault(bound.original.id, np.asarray(bound.matrix, dtype=np.float32))
    for geometry in document.geometries:
        transform = transforms.get(geometry.id, np.eye(4, dtype=np.float32))
        for primitive in geometry.primitives:
            # Some Valkyrie meshes include decorative line sets (joint rings
            # and guide marks). Vulkan's surface renderer does not consume
            # them, so retain only triangulatable surface primitives.
            if not hasattr(primitive, "triangleset"):
                continue
            triangle_set = primitive.triangleset()
            vertices = triangle_set.vertex[triangle_set.vertex_index]
            homogeneous = np.concatenate(
                (vertices, np.ones((*vertices.shape[:-1], 1), dtype=np.float32)), axis=-1
            )
            vertices = np.einsum("ij,...j->...i", transform, homogeneous)[..., :3]
            for triangle in vertices:
                normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
                length = float(np.linalg.norm(normal))
                if length:
                    normal /= length
                triangles.append((normal, triangle))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as stream:
        stream.write(b"CAO Valkyrie mesh conversion".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(triangles)))
        for normal, triangle in triangles:
            stream.write(struct.pack("<12fH", *normal, *triangle[0], *triangle[1], *triangle[2], 0))
    return len(triangles)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    if not args.source.is_dir():
        raise SystemExit(f"Missing Valkyrie mesh source: {args.source}")
    converted = []
    for dae in sorted(args.source.rglob("*.dae")):
        relative = dae.relative_to(args.source).with_suffix(".stl")
        triangle_count = write_binary_stl(dae, args.output / relative)
        converted.append({"source": dae.relative_to(ROOT).as_posix(),
                          "output": relative.as_posix(), "triangles": triangle_count})
    manifest = {"format": "cao_valkyrie_mesh_manifest_v1", "license": "NASA-1.3",
                "source_repository": "openhumanoids/val_description", "meshes": converted}
    (args.output.parent / "mesh_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"converted {len(converted)} Valkyrie meshes to {args.output}")


if __name__ == "__main__":
    main()
