#!/usr/bin/env python3
"""One-time migration tool: extract per-model parameter YAMLs from the vendored
Fairino URDFs (references/fairino_vendor/fairino_description).

The generated YAMLs under config/<model>/ are the living source of
fr_description; this script only documents how they were derived.

Usage (from the repository root):
    python3 src/fairino_ros2_driver/fr_description/scripts/generate_from_vendor.py \
        --vendor-urdf references/fairino_vendor/fairino_description/urdf \
        --out src/fairino_ros2_driver/fr_description/config

Naming normalization applied during extraction:
  links : wrist3_link / wrist3_Link  ->  wrist_3_link      (ur_description style)
  joints: j1..j6                     ->  shoulder_pan_joint, shoulder_lift_joint,
                                        elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
  mesh  : package://fairino_description/... and ../meshes/...
          -> package://fr_description/meshes/...

Dropped vendor artifacts (do not carry into the new description):
  - dangling `tool` fixed joints whose child link definition is commented out
    (FR3WML/FR3WMS/FR5WM); flange + tool0 supersede them
  - zero-valued <dynamics>, <calibration>, <safety_controller> (SolidWorks exporter noise)
"""

import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

# (model key, vendor urdf file, nominal DH d6 flange offset [m], source remark)
MODELS = [
    ("fairino3_v6", "fairino3_v6.urdf", 0.100),
    ("fairino3_mt_v6", "fairino3_mt_v6.urdf", 0.100),
    ("fairino5_v6", "fairino5_v6.urdf", 0.100),
    ("fairino10_v6", "fairino10_v6.urdf", 0.106),
    ("fairino16_v6", "fairino16_v6.urdf", 0.106),
    ("fairino20_v6", "fairino20_v6.urdf", 0.120),
    ("fairino30_v6", "fairino30_v6.urdf", 0.120),
    ("fr3wml", "FR3WML.urdf", 0.100),
    ("fr3wms", "FR3WMS.urdf", 0.100),
    ("fr5wm", "FR5WM.urdf", 0.100),
]

LINK_MAP = {
    "base_link": "base_link",
    "shoulder_link": "shoulder_link", "shoulder_Link": "shoulder_link",
    "upperarm_link": "upper_arm_link", "upperarm_Link": "upper_arm_link",
    "uppperarm_link": "upper_arm_link",  # vendor typo in fairino30_v6.urdf
    "forearm_link": "forearm_link", "forearm_Link": "forearm_link",
    "wrist1_link": "wrist_1_link", "wrist1_Link": "wrist_1_link",
    "wrist2_link": "wrist_2_link", "wrist2_Link": "wrist_2_link",
    "wrist3_link": "wrist_3_link", "wrist3_Link": "wrist_3_link",
}
JOINT_MAP = {
    "j1": "shoulder_pan_joint", "joint1": "shoulder_pan_joint",
    "j2": "shoulder_lift_joint", "joint2": "shoulder_lift_joint",
    "j3": "elbow_joint", "joint3": "elbow_joint",
    "j4": "wrist_1_joint", "joint4": "wrist_1_joint",
    "j5": "wrist_2_joint", "joint5": "wrist_2_joint",
    "j6": "wrist_3_joint", "joint6": "wrist_3_joint",
}


def norm_mesh(path: str) -> str:
    if path.startswith("package://"):
        return path.replace(
            "package://fairino_description/", "package://fr_description/", 1
        )
    m = re.match(r"\.\./meshes/(.+)$", path)
    if m:
        return "package://fr_description/meshes/" + m.group(1)
    raise ValueError(f"unrecognized mesh path: {path}")


def origin_of(elem):
    if elem is None:
        return "0 0 0", "0 0 0"
    o = elem.find("origin")
    if o is None:
        return "0 0 0", "0 0 0"
    return o.get("xyz", "0 0 0"), o.get("rpy", "0 0 0")


def extract(vendor_urdf: Path) -> dict:
    root = ET.parse(vendor_urdf).getroot()
    kin, lim, phys, vis, col = {}, {}, {}, {}, {}

    for joint in root.iter("joint"):
        name = joint.get("name")
        if name not in JOINT_MAP:
            continue
        xyz, rpy = origin_of(joint)
        axis = joint.find("axis")
        limit = joint.find("limit")
        kin[JOINT_MAP[name]] = {
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis.get("xyz") if axis is not None else "0 0 1",
        }
        lim[JOINT_MAP[name]] = {
            k: float(limit.get(k)) for k in ("lower", "upper", "effort", "velocity")
        }

    for link in root.iter("link"):
        raw = link.get("name")
        if raw not in LINK_MAP:
            continue
        name = LINK_MAP[raw]

        inertial = link.find("inertial")
        ixyz, irpy = origin_of(inertial)
        assert irpy == "0 0 0", f"{vendor_urdf.name}: nonzero inertial rpy on {raw}"
        inertia = inertial.find("inertia")
        phys[name] = {
            "mass": float(inertial.find("mass").get("value")),
            "com": ixyz,
            "inertia": {k: float(inertia.get(k)) for k in
                        ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")},
        }

        visual = link.find("visual")
        if visual is None:
            vis[name] = {"xyz": "0 0 0", "rpy": "0 0 0", "mesh": None,
                         "rgba": "0.8 0.8 0.8 1"}
        else:
            vxyz, vrpy = origin_of(visual)
            color = visual.find(".//color")
            vis[name] = {
                "xyz": vxyz,
                "rpy": vrpy,
                "mesh": norm_mesh(visual.find(".//mesh").get("filename")),
                "rgba": color.get("rgba") if color is not None else "0.8 0.8 0.8 1",
            }

        collision = link.find("collision")
        if collision is None:
            col[name] = {"xyz": "0 0 0", "rpy": "0 0 0", "mesh": None}
        else:
            cxyz, crpy = origin_of(collision)
            col[name] = {
                "xyz": cxyz,
                "rpy": crpy,
                "mesh": norm_mesh(collision.find(".//mesh").get("filename")),
            }

    assert len(kin) == 6, f"{vendor_urdf.name}: expected 6 joints, got {list(kin)}"
    assert len(phys) == 7, f"{vendor_urdf.name}: expected 7 links, got {list(phys)}"
    return {"kinematics": kin, "joint_limits": lim, "physical": phys,
            "visual": vis, "collision": col}


def dump(path: Path, header: str, data: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        header + yaml.safe_dump(data, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vendor-urdf", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    for model, urdf_file, flange_offset in MODELS:
        data = extract(args.vendor_urdf / urdf_file)
        base = args.out / model
        src = f"references/fairino_vendor/fairino_description/urdf/{urdf_file}"

        dump(base / "default_kinematics.yaml",
             f"# Nominal kinematics for {model}, extracted from {src}.\n"
             "# Joint origins are relative to the parent link frame.\n",
             {"kinematics": data["kinematics"], "flange_offset": flange_offset})
        dump(base / "joint_limits.yaml",
             f"# Joint limits for {model}, extracted from {src}.\n",
             {"joint_limits": data["joint_limits"]})
        dump(base / "physical_parameters.yaml",
             f"# Inertial parameters for {model}, extracted from {src}.\n",
             {"physical": data["physical"]})
        dump(base / "visual_parameters.yaml",
             f"# Visual/collision geometry for {model}, extracted from {src}.\n",
             {"visual": data["visual"], "collision": data["collision"]})
        print(f"{model}: 4 files written")


if __name__ == "__main__":
    main()
