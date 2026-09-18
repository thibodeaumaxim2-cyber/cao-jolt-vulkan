#!/usr/bin/env python3
"""Expand the vendored NASA Valkyrie Xacro without a full ROS install."""
import argparse
from pathlib import Path
import sys
import types

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "third_party/val_description/model/robots/valkyrie_sim.xacro"
OUTPUT = ROOT / "assets/valkyrie/valkyrie_sim.urdf"
MUJOCO_OUTPUT = ROOT / "assets/valkyrie/valkyrie_mujoco.urdf"
MESH_ROOT = ROOT / "assets/valkyrie/meshes"

# The upstream description predates lexical Xacro property scope.  Its link
# name fragments expose parent-link aliases whose value is a macro argument
# (for example ``TorsoYawParentLinkName=${waist_root_link}``).  Current Xacro
# evaluates those aliases in the include's global scope, where the macro
# argument does not exist.  Replace only these six aliases at their uses; the
# source package itself stays pristine and the generated URDF has the same
# intended parent links.
LEGACY_PARENT_LINK_ALIASES = {
    "${TorsoYawParentLinkName}": "${waist_root_link}",
    "${LowerNeckPitchParentLinkName}": "${head_root_link}",
    "${HipYawParentLinkName}": "${leg_root_link}",
    "${ShoulderPitchParentLinkName}": "${arm_root_link}",
    "${ThumbRollParentLinkName}": "${thumb_root_link}",
    "${IndexFingerPitch1ParentLinkName}": "${index_finger_root_link}",
    "${MiddleFingerPitch1ParentLinkName}": "${middle_finger_root_link}",
    "${PinkyPitch1ParentLinkName}": "${pinky_finger_root_link}",
}
LEGACY_DYNAMIC_ARGUMENTS = (
    "prefix",
    "reflect",
    "waist_root_link",
    "head_root_link",
    "leg_root_link",
    "arm_root_link",
    "thumb_root_link",
    "index_finger_root_link",
    "middle_finger_root_link",
    "pinky_finger_root_link",
)


def install_package_lookup() -> None:
    """Provide the one ROS package lookup used by the source Xacro files."""
    package_module = types.ModuleType("ament_index_python.packages")

    def share_directory(name: str) -> str:
        if name == "val_description":
            return str(ROOT / "third_party/val_description")
        raise RuntimeError(f"Unsupported Xacro package: {name}")

    package_module.get_package_share_directory = share_directory
    index_module = types.ModuleType("ament_index_python")
    index_module.packages = package_module
    sys.modules["ament_index_python"] = index_module
    sys.modules["ament_index_python.packages"] = package_module


def normalize_legacy_document(document) -> None:
    """Apply the narrow Xacro-scope compatibility substitutions in-place."""
    for node in document.getElementsByTagName("*"):
        if not node.attributes:
            continue
        for attribute_index in range(node.attributes.length):
            attribute = node.attributes.item(attribute_index)
            value = attribute.value
            for legacy_name, macro_argument in LEGACY_PARENT_LINK_ALIASES.items():
                value = value.replace(legacy_name, macro_argument)
            attribute.value = value

    # The simulation entry point consumes the simulation ATI serial while
    # expanding ``valkyrie_base_sim.xacro``, but declares the serial include
    # afterwards.  ROS's historical launch sequence happened to pre-load it;
    # make that dependency explicit in the generated-only document.
    root = document.documentElement
    serial_includes = [
        child for child in root.childNodes
        if getattr(child, "tagName", None) == "xacro:include"
        and "serial_numbers/" in child.getAttribute("filename")
    ]
    for include in serial_includes:
        root.removeChild(include)
        root.insertBefore(include, root.firstChild)


def install_legacy_scope_compatibility(xacro) -> None:
    """Evaluate old parameter-dependent global properties in the call scope.

    The 2015 source puts per-limb properties in included files and expects them
    to see the ``prefix``/``reflect`` arguments of the macro that consumes
    them.  Modern Xacro correctly keeps includes lexical, so those properties
    otherwise resolve from the document root.  This compatibility layer leaves
    normal properties alone and evaluates just parameter-dependent values at
    their point of use without caching a left/right-specific result globally.
    """
    active_scopes = []
    eval_all = xacro.eval_all
    resolve_property = xacro.Table._resolve_

    def compatible_eval_all(node, macros, symbols):
        active_scopes.append(symbols)
        try:
            return eval_all(node, macros, symbols)
        finally:
            active_scopes.pop()

    def compatible_resolve_property(symbols, key):
        raw_value = dict.get(symbols, key)
        if (
            active_scopes
            and key in symbols.unevaluated
            and isinstance(raw_value, str)
            and any(f"${{{argument}" in raw_value for argument in LEGACY_DYNAMIC_ARGUMENTS)
        ):
            return symbols._eval_literal(xacro.eval_text(raw_value, active_scopes[-1]))
        return resolve_property(symbols, key)

    xacro.eval_all = compatible_eval_all
    xacro.Table._resolve_ = compatible_resolve_property


def redirect_meshes_to_converted_assets(document) -> None:
    """Point visual and collision meshes to the locally validated STL assets."""
    source_prefix = "package://val_description/model/meshes/"
    missing = []
    for mesh in document.getElementsByTagName("mesh"):
        filename = mesh.getAttribute("filename")
        if not filename.startswith(source_prefix):
            continue
        relative = Path(filename.removeprefix(source_prefix)).with_suffix(".stl")
        output = MESH_ROOT / relative
        if not output.is_file():
            missing.append(str(relative))
            continue
        mesh.setAttribute("filename", str(Path("meshes") / relative))
    if missing:
        raise RuntimeError(
            "Converted Valkyrie meshes are missing: " + ", ".join(sorted(set(missing)))
        )


def remove_massless_sensor_frames(document) -> None:
    """Remove fixed sensor frames whose zero inertia is invalid in MuJoCo.

    These frames model camera/IMU mounting coordinates rather than articulated
    robot mass.  Physical links remain untouched; the resulting URDF is a
    dedicated import artifact, not a replacement for the canonical export.
    """
    removed = set()
    for link in document.getElementsByTagName("link"):
        inertial = next(
            (child for child in link.childNodes if getattr(child, "tagName", None) == "inertial"),
            None,
        )
        if inertial is None:
            continue
        mass = next(
            (child for child in inertial.childNodes if getattr(child, "tagName", None) == "mass"),
            None,
        )
        # The source represents sensor frames as 1e-7 kg placeholders with
        # near-zero inertia.  They are non-physical and below MuJoCo's valid
        # inertia range, whereas the smallest robot link is substantially
        # heavier.
        if mass is not None and float(mass.getAttribute("value")) <= 1e-6:
            removed.add(link.getAttribute("name"))

    if not removed:
        return
    root = document.documentElement
    for link in list(document.getElementsByTagName("link")):
        if link.getAttribute("name") in removed:
            root.removeChild(link)
    for joint in list(document.getElementsByTagName("joint")):
        connected = {
            child.getAttribute("link")
            for child in joint.childNodes
            if getattr(child, "tagName", None) in {"parent", "child"}
        }
        if connected & removed:
            root.removeChild(joint)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--mujoco-output", type=Path, default=MUJOCO_OUTPUT)
    args = parser.parse_args()
    install_package_lookup()
    import xacro

    # Xacro 1.x delegates substitution arguments to ROS 1's roslaunch.  The
    # description only uses one deterministic lookup, so provide it directly
    # instead of pulling ROS into this project.
    legacy_extension = getattr(xacro, "eval_extension", None)

    if legacy_extension:
        def compatible_extension(expression: str) -> str:
            if expression == "$(find val_description)":
                return str(ROOT / "third_party/val_description")
            return legacy_extension(expression)

        xacro.eval_extension = compatible_extension
    install_legacy_scope_compatibility(xacro)

    # This legacy package expects includes below model/robots/ to be resolved
    # from that directory. New standalone Xacro resolves nested includes from
    # their own file, duplicating ``common/xacro``. Normalize only that known
    # legacy path while leaving the vendored source untouched.
    parse = xacro.parse

    def compatible_parse(input_file, filename=None):
        if filename:
            filename = filename.replace("/common/xacro/common/xacro/", "/common/xacro/")
        document = parse(input_file, filename)
        normalize_legacy_document(document)
        return document

    xacro.parse = compatible_parse

    document = xacro.process_file(str(args.source))
    redirect_meshes_to_converted_assets(document)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(document.toprettyxml(indent="  "))
    print(f"wrote {args.output}")
    remove_massless_sensor_frames(document)
    args.mujoco_output.write_text(document.toprettyxml(indent="  "))
    print(f"wrote {args.mujoco_output}")


if __name__ == "__main__":
    main()
