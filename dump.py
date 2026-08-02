from isaacsim import SimulationApp

# Start Isaac Sim first
simulation_app = SimulationApp({
    "headless": True
})

# IMPORTANT: import pxr AFTER SimulationApp starts
from pxr import Usd, UsdGeom, UsdPhysics

import os

USD_PATH = "/home/ubuntu/isaac_assets/test_forklift/forklift_c_camera.usd"
OUTPUT_PATH = "/home/ubuntu/isaac_assets/test_forklift/forklift_usd_inspection.txt"


def section(out, title):
    out.write("\n")
    out.write("=" * 80 + "\n")
    out.write(title + "\n")
    out.write("=" * 80 + "\n")


try:
    print(f"Opening USD: {USD_PATH}")

    stage = Usd.Stage.Open(USD_PATH)

    if stage is None:
        raise RuntimeError(f"Could not open USD: {USD_PATH}")

    with open(OUTPUT_PATH, "w", encoding="utf-8") as out:

        # --------------------------------------------------
        # USD information
        # --------------------------------------------------

        section(out, "USD INFORMATION")

        out.write(f"USD file: {USD_PATH}\n")
        out.write(
            f"Root layer: {stage.GetRootLayer().identifier}\n"
        )

        out.write(
            f"Up axis: {UsdGeom.GetStageUpAxis(stage)}\n"
        )

        out.write(
            f"Meters per unit: "
            f"{UsdGeom.GetStageMetersPerUnit(stage)}\n"
        )

        # --------------------------------------------------
        # Stage metadata
        # --------------------------------------------------

        section(out, "STAGE METADATA")

        out.write(
            f"Time codes per second: "
            f"{stage.GetTimeCodesPerSecond()}\n"
        )

        out.write(
            f"Start time code: "
            f"{stage.GetStartTimeCode()}\n"
        )

        out.write(
            f"End time code: "
            f"{stage.GetEndTimeCode()}\n"
        )

        # --------------------------------------------------
        # Prim hierarchy
        # --------------------------------------------------

        section(out, "PRIM HIERARCHY")

        for prim in stage.Traverse():

            depth = len(
                prim.GetPath().pathString.split("/")
            ) - 2

            indent = "  " * max(depth, 0)

            out.write(
                f"{indent}{prim.GetPath()} "
                f"[{prim.GetTypeName()}]\n"
            )

        # --------------------------------------------------
        # Cameras
        # --------------------------------------------------

        section(out, "CAMERAS")

        camera_count = 0

        for prim in stage.Traverse():

            if prim.IsA(UsdGeom.Camera):

                camera_count += 1

                camera = UsdGeom.Camera(prim)

                out.write(
                    f"\nCamera: {prim.GetPath()}\n"
                )

                out.write(
                    f"  Projection: "
                    f"{camera.GetProjectionAttr().Get()}\n"
                )

                out.write(
                    f"  Focal length: "
                    f"{camera.GetFocalLengthAttr().Get()}\n"
                )

                out.write(
                    f"  Horizontal aperture: "
                    f"{camera.GetHorizontalApertureAttr().Get()}\n"
                )

                out.write(
                    f"  Vertical aperture: "
                    f"{camera.GetVerticalApertureAttr().Get()}\n"
                )

        if camera_count == 0:
            out.write("No cameras found.\n")

        # --------------------------------------------------
        # Articulation roots
        # --------------------------------------------------

        section(out, "ARTICULATION ROOTS")

        found = False

        for prim in stage.Traverse():

            if prim.HasAPI(
                UsdPhysics.ArticulationRootAPI
            ):

                found = True

                out.write(
                    f"{prim.GetPath()}\n"
                )

        if not found:
            out.write(
                "No ArticulationRootAPI found.\n"
            )

        # --------------------------------------------------
        # Physics joints
        # --------------------------------------------------

        section(out, "PHYSICS JOINTS")

        joint_count = 0

        for prim in stage.Traverse():

            if prim.IsA(UsdPhysics.Joint):

                joint_count += 1

                out.write(
                    f"\nJoint: {prim.GetPath()}\n"
                )

                out.write(
                    f"  Type: {prim.GetTypeName()}\n"
                )

                body0 = prim.GetRelationship(
                    "body0"
                )

                if body0:
                    out.write(
                        f"  body0: "
                        f"{body0.GetTargets()}\n"
                    )

                body1 = prim.GetRelationship(
                    "body1"
                )

                if body1:
                    out.write(
                        f"  body1: "
                        f"{body1.GetTargets()}\n"
                    )

        if joint_count == 0:
            out.write("No physics joints found.\n")

        # --------------------------------------------------
        # Potential forklift-related prims
        # --------------------------------------------------

        section(
            out,
            "FORKLIFT / VEHICLE RELATED PRIMS"
        )

        keywords = [
            "forklift",
            "vehicle",
            "chassis",
            "base",
            "body",
            "wheel",
            "steer",
            "steering",
            "fork",
            "lift",
        ]

        for prim in stage.Traverse():

            name = prim.GetName().lower()

            if any(
                keyword in name
                for keyword in keywords
            ):

                out.write(
                    f"{prim.GetPath()} "
                    f"[{prim.GetTypeName()}]\n"
                )

        # --------------------------------------------------
        # All attributes containing ROS-related information
        # --------------------------------------------------

        section(
            out,
            "ROS RELATED ATTRIBUTES"
        )

        ros_keywords = [
            "ros",
            "ros2",
            "topic",
            "publisher",
            "subscriber",
            "ackermann",
            "clock",
            "odom",
            "tf",
            "cmd_vel",
            "camera",
        ]

        for prim in stage.Traverse():

            for attr in prim.GetAttributes():

                attr_name = attr.GetName()

                try:
                    value = attr.Get()
                except Exception:
                    continue

                text = str(value)

                combined = (
                    str(prim.GetPath())
                    + " "
                    + attr_name
                    + " "
                    + text
                ).lower()

                if any(
                    keyword in combined
                    for keyword in ros_keywords
                ):

                    out.write(
                        f"\nPrim: {prim.GetPath()}\n"
                    )

                    out.write(
                        f"  Attribute: "
                        f"{attr_name}\n"
                    )

                    out.write(
                        f"  Type: "
                        f"{attr.GetTypeName()}\n"
                    )

                    out.write(
                        f"  Value: {text}\n"
                    )

    print()
    print("=" * 60)
    print("Inspection complete")
    print(f"Output: {OUTPUT_PATH}")
    print("=" * 60)

finally:

    simulation_app.close()
