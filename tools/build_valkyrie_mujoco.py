#!/usr/bin/env python3
"""Build the isolated, torque-actuated MuJoCo Valkyrie candidate model."""
import argparse
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets/valkyrie/valkyrie_mujoco.urdf"
OUTPUT = ROOT / "assets/valkyrie/valkyrie.xml"


def add_free_base_and_actuators(tree: ET.ElementTree) -> None:
    root = tree.getroot()
    worldbody = root.find("worldbody")
    # MuJoCo's URDF importer treats the root pelvis link as world geometry.
    # Wrap the imported root geoms and child bodies in a free pelvis body,
    # restoring the physical floating base from the canonical URDF inertia.
    pelvis = ET.Element("body", {"name": "pelvis", "pos": "0 0 1.18"})
    pelvis.append(ET.Element("freejoint", {"name": "valkyrie_root"}))
    pelvis.append(ET.Element(
        "inertial",
        {
            "pos": "-0.00532 -0.003512 -0.0036",
            "mass": "8.22",
            "fullinertia": "0.11866378 0.097963425 0.083854638 -0.000143482 0.003271293 0.002159545",
        },
    ))
    for child in list(worldbody):
        worldbody.remove(child)
        pelvis.append(child)
    worldbody.append(pelvis)
    # This first controller candidate uses only the source foot boxes for
    # collision.  Detailed visual meshes and sensor housing primitives are
    # visual-only: allowing them to self-collide creates hundreds of expensive,
    # non-physical contacts before a balance controller exists.
    for geom in pelvis.findall(".//geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
        if geom.get("type") == "mesh":
            geom.set("group", "1")
    for foot_name in ("leftFoot", "rightFoot"):
        foot = next(body for body in pelvis.findall(".//body") if body.get("name") == foot_name)
        foot_box = next(geom for geom in foot.findall("geom") if geom.get("type") == "box")
        foot_box.set("contype", "1")
        foot_box.set("conaffinity", "1")
        sole_name = "left_sole" if foot_name == "leftFoot" else "right_sole"
        ET.SubElement(foot, "site", {
            "name": sole_name,
            "type": "box",
            "pos": "0.066 0 -0.088",
            "size": "0.12 0.07 0.003",
        })
    ET.SubElement(pelvis, "site", {"name": "pelvis_imu", "size": "0.01"})
    worldbody.insert(0, ET.Element(
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "size": "20 20 0.1",
            "material": "floor",
            "friction": "0.9 0.02 0.001",
        },
    ))

    asset = root.find("asset")
    ET.SubElement(asset, "texture", {
        "name": "checker", "type": "2d", "builtin": "checker", "width": "512", "height": "512",
        "rgb1": "0.12 0.14 0.18", "rgb2": "0.24 0.27 0.32",
    })
    ET.SubElement(asset, "material", {
        "name": "floor", "texture": "checker", "texrepeat": "8 8", "reflectance": "0.15",
    })
    actuator = ET.SubElement(root, "actuator")
    for joint in root.findall(".//joint"):
        name = joint.get("name")
        if not name or name == "hokuyo_joint":
            continue
        force_range = joint.get("actuatorfrcrange", "-10 10")
        ET.SubElement(actuator, "motor", {
            "name": name,
            "joint": name,
            "gear": "1",
            "ctrllimited": "true",
            "ctrlrange": force_range,
        })
    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "framequat", {"name": "pelvis_orientation", "objtype": "site", "objname": "pelvis_imu"})
    ET.SubElement(sensor, "gyro", {"name": "pelvis_angular_velocity", "site": "pelvis_imu"})
    ET.SubElement(sensor, "accelerometer", {"name": "pelvis_linear_acceleration", "site": "pelvis_imu"})
    ET.SubElement(sensor, "touch", {"name": "left_foot_contact", "site": "left_sole"})
    ET.SubElement(sensor, "touch", {"name": "right_foot_contact", "site": "right_sole"})


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=URDF)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()

    imported = mujoco.MjModel.from_xml_path(str(args.urdf))
    with tempfile.NamedTemporaryFile(
        suffix=".xml", prefix=".valkyrie_import_", dir=args.output.parent, delete=False
    ) as temporary:
        imported_xml = Path(temporary.name)
    mujoco.mj_saveLastXML(str(imported_xml), imported)

    try:
        tree = ET.parse(imported_xml)
    finally:
        imported_xml.unlink(missing_ok=True)
    add_free_base_and_actuators(tree)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="unicode", xml_declaration=True)

    model = mujoco.MjModel.from_xml_path(str(args.output))
    print(
        f"wrote {args.output} nq={model.nq} nv={model.nv} "
        f"nu={model.nu} nbody={model.nbody} njnt={model.njnt}"
    )


if __name__ == "__main__":
    main()
