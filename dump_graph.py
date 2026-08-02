from isaacsim import SimulationApp

# ------------------------------------------------------------
# Start Isaac Sim FIRST
# ------------------------------------------------------------

simulation_app = SimulationApp({
    "headless": True
})

# IMPORTANT:
# Isaac Sim / OmniGraph imports must happen AFTER SimulationApp
# is initialized.
from pxr import Usd
import omni.graph.core as og

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

USD_PATH = "/home/ubuntu/isaac_assets/test_forklift/forklift_c_camera.usd"

OUTPUT_PATH = (
    "/home/ubuntu/isaac_assets/test_forklift/"
    "omnigraph_connections.txt"
)


def write(out, text=""):
    out.write(str(text) + "\n")


def section(out, title):
    write(out)
    write(out, "=" * 100)
    write(out, title)
    write(out, "=" * 100)


try:

    print(f"Opening: {USD_PATH}")

    # --------------------------------------------------------
    # Open USD
    # --------------------------------------------------------

    stage = Usd.Stage.Open(USD_PATH)

    if stage is None:
        raise RuntimeError(
            f"Failed to open USD: {USD_PATH}"
        )

    # Let Isaac Sim process the stage
    for _ in range(10):
        simulation_app.update()

    with open(
        OUTPUT_PATH,
        "w",
        encoding="utf-8"
    ) as out:

        # ====================================================
        # BASIC INFORMATION
        # ====================================================

        section(out, "USD INFORMATION")

        write(
            out,
            f"USD: {USD_PATH}"
        )

        write(
            out,
            f"Root layer: "
            f"{stage.GetRootLayer().identifier}"
        )

        # ====================================================
        # FIND OMNIGRAPH PRIMS
        # ====================================================

        section(
            out,
            "OMNIGRAPH / ACTION GRAPH PRIMS"
        )

        graph_paths = []

        for prim in stage.Traverse():

            type_name = prim.GetTypeName()

            if (
                "ActionGraph" in type_name
                or "OmniGraph" in type_name
                or type_name == "omni.graph.nodes"
            ):

                graph_paths.append(
                    str(prim.GetPath())
                )

                write(
                    out,
                    f"{prim.GetPath()} "
                    f"[{type_name}]"
                )

        # ====================================================
        # FIND PRIMS THAT LOOK LIKE GRAPH NODES
        # ====================================================

        section(
            out,
            "POSSIBLE OMNIGRAPH NODES"
        )

        node_paths = []

        for prim in stage.Traverse():

            path = str(prim.GetPath())
            type_name = prim.GetTypeName()

            # We are deliberately broad here.
            if (
                "node" in type_name.lower()
                or "graph" in type_name.lower()
                or "ros" in path.lower()
                or "ackermann" in path.lower()
                or "camera" in path.lower()
                or "imu" in path.lower()
                or "clock" in path.lower()
                or "joint" in path.lower()
            ):

                node_paths.append(path)

                write(
                    out,
                    f"{path} "
                    f"[{type_name}]"
                )

        # ====================================================
        # ATTRIBUTE DETAILS
        # ====================================================

        section(
            out,
            "NODE ATTRIBUTES"
        )

        for path in node_paths:

            prim = stage.GetPrimAtPath(path)

            if not prim.IsValid():
                continue

            write(out)
            write(
                out,
                f"NODE: {path}"
            )

            write(
                out,
                f"TYPE: {prim.GetTypeName()}"
            )

            for attr in prim.GetAttributes():

                try:
                    value = attr.Get()
                except Exception:
                    value = "<ERROR>"

                write(
                    out,
                    f"  ATTRIBUTE: {attr.GetName()}"
                )

                write(
                    out,
                    f"    TYPE: {attr.GetTypeName()}"
                )

                write(
                    out,
                    f"    VALUE: {value}"
                )

                # ------------------------------------------------
                # Relationship information
                # ------------------------------------------------

                try:
                    connections = (
                        attr.GetConnections()
                    )
                except Exception:
                    connections = []

                if connections:

                    write(
                        out,
                        "    CONNECTIONS:"
                    )

                    for connection in connections:

                        write(
                            out,
                            f"      -> {connection}"
                        )

        # ====================================================
        # USD ATTRIBUTE CONNECTIONS
        # ====================================================

        section(
            out,
            "ALL USD ATTRIBUTE CONNECTIONS"
        )

        connection_count = 0

        for prim in stage.Traverse():

            for attr in prim.GetAttributes():

                try:
                    connections = (
                        attr.GetConnections()
                    )
                except Exception:
                    continue

                if not connections:
                    continue

                connection_count += 1

                write(out)
                write(
                    out,
                    f"SOURCE: "
                    f"{prim.GetPath()}"
                )

                write(
                    out,
                    f"ATTRIBUTE: "
                    f"{attr.GetName()}"
                )

                write(
                    out,
                    f"TYPE: "
                    f"{attr.GetTypeName()}"
                )

                for connection in connections:

                    write(
                        out,
                        f"    --> {connection}"
                    )

        write(out)

        write(
            out,
            f"Total connected attributes: "
            f"{connection_count}"
        )

        # ====================================================
        # ROS-SPECIFIC CONNECTIONS
        # ====================================================

        section(
            out,
            "ROS / ACKERMANN / CAMERA / IMU CONNECTIONS"
        )

        keywords = [
            "ros",
            "ros2",
            "ackermann",
            "clock",
            "odom",
            "tf",
            "cmd_vel",
            "camera",
            "imu",
            "joint",
            "image",
            "info",
        ]

        for prim in stage.Traverse():

            prim_text = (
                str(prim.GetPath())
                + " "
                + prim.GetTypeName()
            ).lower()

            if not any(
                k in prim_text
                for k in keywords
            ):
                continue

            for attr in prim.GetAttributes():

                try:
                    connections = (
                        attr.GetConnections()
                    )
                except Exception:
                    continue

                if not connections:
                    continue

                write(out)
                write(
                    out,
                    f"{prim.GetPath()}"
                )

                write(
                    out,
                    f"  ATTRIBUTE: "
                    f"{attr.GetName()}"
                )

                try:
                    value = attr.Get()
                except Exception:
                    value = "<ERROR>"

                write(
                    out,
                    f"  VALUE: {value}"
                )

                write(
                    out,
                    "  CONNECTIONS:"
                )

                for connection in connections:

                    write(
                        out,
                        f"    --> {connection}"
                    )

        # ====================================================
        # LOOK FOR ROS TOPIC NAMES IN ALL VALUES
        # ====================================================

        section(
            out,
            "ROS TOPIC / MESSAGE STRING SEARCH"
        )

        search_terms = [
            "/clock",
            "/odom",
            "/tf",
            "/tf_static",
            "/cmd_vel",
            "/ackermann_cmd",
            "AckermannDrive",
            "AckermannDriveStamped",
            "Odometry",
            "Twist",
            "Image",
            "CameraInfo",
            "JointState",
            "ROS2",
        ]

        found = set()

        for prim in stage.Traverse():

            for attr in prim.GetAttributes():

                try:
                    value = attr.Get()
                except Exception:
                    continue

                text = str(value)

                for term in search_terms:

                    if term.lower() in text.lower():

                        key = (
                            str(prim.GetPath()),
                            attr.GetName(),
                            text,
                        )

                        if key in found:
                            continue

                        found.add(key)

                        write(out)
                        write(
                            out,
                            f"PRIM: "
                            f"{prim.GetPath()}"
                        )

                        write(
                            out,
                            f"ATTRIBUTE: "
                            f"{attr.GetName()}"
                        )

                        write(
                            out,
                            f"TYPE: "
                            f"{attr.GetTypeName()}"
                        )

                        write(
                            out,
                            f"VALUE: {text}"
                        )

        # ====================================================
        # END
        # ====================================================

        section(
            out,
            "END OF INSPECTION"
        )

    print()
    print("=" * 70)
    print("OmniGraph inspection complete")
    print(f"Output: {OUTPUT_PATH}")
    print("=" * 70)

finally:

    simulation_app.close()