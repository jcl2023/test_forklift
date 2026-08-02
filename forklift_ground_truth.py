from isaacsim import SimulationApp

# ============================================================
# Start Isaac Sim
# ============================================================

simulation_app = SimulationApp({
    "headless": True
})

# ============================================================
# Imports AFTER SimulationApp
# ============================================================

import omni.graph.core as og
import omni.usd
import omni.kit.app
import omni.timeline

from pxr import Sdf


# ============================================================
# Configuration
# ============================================================

USD_PATH = (
    "/home/ubuntu/isaac_assets/test_forklift/"
    "forklift_c_center_stereo_clean.usd"
)

ROBOT_PRIM = "/World/forklift_c"

GRAPH_PATH = (
    "/World/forklift_c/ground_truth_ros"
)

ODOM_TOPIC = "/odom"
TF_TOPIC = "/tf"
CLOCK_TOPIC = "/clock"

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"


# ============================================================
# Main
# ============================================================

try:

    print("=" * 70)
    print("Starting Isaac Sim")
    print("=" * 70)

    # --------------------------------------------------------
    # Enable ROS 2 bridge
    # --------------------------------------------------------

    app = omni.kit.app.get_app()

    extension_manager = (
        app.get_extension_manager()
    )

    extension_manager.set_extension_enabled_immediate(
        "isaacsim.ros2.bridge",
        True
    )

    # Give ROS2 bridge time to initialize
    for _ in range(30):
        simulation_app.update()

    print("ROS 2 bridge enabled")

    # --------------------------------------------------------
    # Open USD
    # --------------------------------------------------------

    print()
    print("Opening USD:")
    print(f"  {USD_PATH}")

    usd_context = omni.usd.get_context()

    result = usd_context.open_stage(USD_PATH)

    if not result:
        raise RuntimeError(
            f"Failed to open USD: {USD_PATH}"
        )

    # Wait for stage
    stage = None

    for _ in range(100):

        simulation_app.update()

        stage = usd_context.get_stage()

        if stage is not None:
            break

    if stage is None:
        raise RuntimeError(
            "USD stage was not created."
        )

    robot = stage.GetPrimAtPath(ROBOT_PRIM)

    if not robot.IsValid():
        raise RuntimeError(
            f"Robot prim not found: {ROBOT_PRIM}"
        )

    print(
        f"Robot prim found: {ROBOT_PRIM}"
    )

    # --------------------------------------------------------
    # Create ROS2 Action Graph
    # --------------------------------------------------------

    print()
    print("=" * 70)
    print("Creating Ground Truth ROS 2 Action Graph")
    print("=" * 70)

    print(
        f"Graph path: {GRAPH_PATH}"
    )

    keys = og.Controller.Keys

    og.Controller.edit(
        {
            "graph_path": GRAPH_PATH,
            "evaluator_name": "execution",
        },
        {

            # =================================================
            # CREATE NODES
            # =================================================

            keys.CREATE_NODES: [

                # Simulation tick
                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick"
                ),

                # Isaac simulation time
                (
                    "ReadSimTime",
                    "isaacsim.core.nodes.IsaacReadSimulationTime"
                ),

                # ROS 2 context
                (
                    "ROS2Context",
                    "isaacsim.ros2.bridge.ROS2Context"
                ),

                # Ground truth odometry
                (
                    "ComputeOdom",
                    "isaacsim.core.nodes.IsaacComputeOdometry"
                ),

                # Odometry publisher
                (
                    "PublishOdom",
                    "isaacsim.ros2.bridge.ROS2PublishOdometry"
                ),

                # TF publisher
                (
                    "PublishTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"
                ),

                # =================================================
                # NEW: ROS2 CLOCK PUBLISHER
                # =================================================

                (
                    "PublishClock",
                    "isaacsim.ros2.bridge.ROS2PublishClock"
                ),
            ],

            # =================================================
            # SET VALUES
            # =================================================

            keys.SET_VALUES: [

                # ------------------------------------------------
                # Ground truth robot
                # ------------------------------------------------

                (
                    "ComputeOdom.inputs:chassisPrim",
                    [
                        Sdf.Path(ROBOT_PRIM)
                    ]
                ),

                # ------------------------------------------------
                # Odometry
                # ------------------------------------------------

                (
                    "PublishOdom.inputs:topicName",
                    ODOM_TOPIC
                ),

                (
                    "PublishOdom.inputs:odomFrameId",
                    ODOM_FRAME
                ),

                (
                    "PublishOdom.inputs:chassisFrameId",
                    BASE_FRAME
                ),

                (
                    "PublishOdom.inputs:queueSize",
                    10
                ),

                # ------------------------------------------------
                # TF
                # ------------------------------------------------

                (
                    "PublishTF.inputs:topicName",
                    TF_TOPIC
                ),

                (
                    "PublishTF.inputs:parentFrameId",
                    ODOM_FRAME
                ),

                (
                    "PublishTF.inputs:childFrameId",
                    BASE_FRAME
                ),

                (
                    "PublishTF.inputs:queueSize",
                    10
                ),

                (
                    "PublishTF.inputs:staticPublisher",
                    False
                ),

                # ------------------------------------------------
                # CLOCK
                # ------------------------------------------------

                (
                    "PublishClock.inputs:topicName",
                    CLOCK_TOPIC
                ),

                (
                    "PublishClock.inputs:queueSize",
                    10
                ),
            ],

            # =================================================
            # CONNECTIONS
            # =================================================

            keys.CONNECT: [

                # =================================================
                # SIMULATION TICK
                # =================================================

                (
                    "OnPlaybackTick.outputs:tick",
                    "ComputeOdom.inputs:execIn"
                ),

                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishTF.inputs:execIn"
                ),

                # =================================================
                # CLOCK
                #
                # IMPORTANT:
                # ReadSimTime does NOT need execIn.
                #
                # The simulation tick directly executes
                # PublishClock.
                # =================================================

                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishClock.inputs:execIn"
                ),

                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishClock.inputs:timeStamp"
                ),

                # =================================================
                # ODOMETRY
                # =================================================

                (
                    "ComputeOdom.outputs:execOut",
                    "PublishOdom.inputs:execIn"
                ),

                (
                    "ComputeOdom.outputs:position",
                    "PublishOdom.inputs:position"
                ),

                (
                    "ComputeOdom.outputs:orientation",
                    "PublishOdom.inputs:orientation"
                ),

                (
                    "ComputeOdom.outputs:linearVelocity",
                    "PublishOdom.inputs:linearVelocity"
                ),

                (
                    "ComputeOdom.outputs:angularVelocity",
                    "PublishOdom.inputs:angularVelocity"
                ),

                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdom.inputs:timeStamp"
                ),

                # =================================================
                # TF
                # =================================================

                (
                    "ComputeOdom.outputs:position",
                    "PublishTF.inputs:translation"
                ),

                (
                    "ComputeOdom.outputs:orientation",
                    "PublishTF.inputs:rotation"
                ),

                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishTF.inputs:timeStamp"
                ),

                # =================================================
                # ROS2 CONTEXT
                # =================================================

                (
                    "ROS2Context.outputs:context",
                    "PublishOdom.inputs:context"
                ),

                (
                    "ROS2Context.outputs:context",
                    "PublishTF.inputs:context"
                ),

                (
                    "ROS2Context.outputs:context",
                    "PublishClock.inputs:context"
                ),
            ],
        },
    )

    print()
    print("Action Graph created successfully.")

    # --------------------------------------------------------
    # Start simulation
    # --------------------------------------------------------

    timeline = (
        omni.timeline.get_timeline_interface()
    )

    timeline.play()

    print()
    print("=" * 70)
    print("GROUND TRUTH ROS 2 RUNNING")
    print("=" * 70)

    print()
    print("Robot:")
    print(f"  {ROBOT_PRIM}")

    print()
    print("Odometry:")
    print(f"  Topic:       {ODOM_TOPIC}")
    print(f"  Frame:       {ODOM_FRAME}")
    print(f"  Child frame: {BASE_FRAME}")

    print()
    print("TF:")
    print(f"  Topic:  {TF_TOPIC}")
    print(f"  Parent: {ODOM_FRAME}")
    print(f"  Child:  {BASE_FRAME}")

    print()
    print("Clock:")
    print(f"  Topic: {CLOCK_TOPIC}")
    print("  Source: IsaacReadSimulationTime")

    print()
    print("Press Ctrl+C to stop.")
    print()

    # --------------------------------------------------------
    # Simulation loop
    # --------------------------------------------------------

    while simulation_app.is_running():

        simulation_app.update()


except KeyboardInterrupt:

    print()
    print("Interrupted by user.")


except Exception as e:

    print()
    print("=" * 70)
    print("ERROR")
    print("=" * 70)

    print(type(e).__name__)
    print(str(e))

    raise


finally:

    print()
    print("Closing Isaac Sim...")

    simulation_app.close()
