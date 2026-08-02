from isaacsim import SimulationApp

# ============================================================
# START ISAAC SIM
# ============================================================

simulation_app = SimulationApp({
    "headless": True,
	
	    # Rendering
    "renderer": "RaytracedLighting",
	
	    # Disable expensive visual features
    "anti_aliasing": 0,
	
	    # Lower viewport/render resolution
    "width": 960,
    "height": 600,
})
# ============================================================
# IMPORTS AFTER SimulationApp
# ============================================================

import omni.graph.core as og
import omni.usd
import omni.kit.app
import omni.timeline


# ============================================================
# CONFIGURATION
# ============================================================

USD_PATH = (
    "/home/ubuntu/isaac_assets/test_forklift/"
    "forklift_c_center_stereo_clean.usd"
)

ROBOT_PRIM = "/World/forklift_c"

GRAPH_PATH = (
    "/World/forklift_c/ground_truth_ros"
)

# ------------------------------------------------------------
# Ground truth topics
# ------------------------------------------------------------

ODOM_TOPIC = "/odom"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
CLOCK_TOPIC = "/clock"

# ------------------------------------------------------------
# Frames
# ------------------------------------------------------------

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

LEFT_CAMERA_FRAME = (
    "front_stereo_camera_left_optical"
)

RIGHT_CAMERA_FRAME = (
    "front_stereo_camera_right_optical"
)

# ============================================================
# CAMERA TF
#
# These are the values already verified for the LEFT camera.
#
# Units: meters
#
# DO NOT change these to millimeters.
# ============================================================

LEFT_CAMERA_TRANSLATION = (
    -0.5187423383485059,
     0.5217299212803488,
    -0.21939674849026913
)

RIGHT_CAMERA_TRANSLATION = (
    -0.5187423383485059,
     0.5202299212803488,
    -0.21939674849026913
)

# Camera optical-frame rotation.
#
# This is the rotation that was already verified with:
#
# ros2 run tf2_ros tf2_echo \
#     base_link front_stereo_camera_left_optical
#
# Quaternion order required by Isaac Sim:
#
# (x, y, z, w)
# ============================================================

CAMERA_ROTATION = (
    -0.5,
     0.5,
     0.5,
    -0.5
)


# ============================================================
# HELPER
# ============================================================

def print_separator(title):
    print()
    print("=" * 70)
    print(title)
    print("=" * 70)


# ============================================================
# MAIN
# ============================================================

try:

    print_separator("STARTING ISAAC SIM")

    print("USD:")
    print(USD_PATH)

    # --------------------------------------------------------
    # Enable ROS2 bridge
    # --------------------------------------------------------

    app = omni.kit.app.get_app()

    extension_manager = (
        app.get_extension_manager()
    )

    extension_manager.set_extension_enabled_immediate(
        "isaacsim.ros2.bridge",
        True
    )

    for _ in range(30):
        simulation_app.update()

    print("ROS 2 bridge enabled.")

    # --------------------------------------------------------
    # Open USD
    # --------------------------------------------------------

    print_separator("OPENING USD")

    usd_context = omni.usd.get_context()

    result = usd_context.open_stage(
        USD_PATH
    )

    if not result:
        raise RuntimeError(
            "Failed to open USD: "
            + USD_PATH
        )

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

    robot = stage.GetPrimAtPath(
        ROBOT_PRIM
    )

    if not robot.IsValid():
        raise RuntimeError(
            "Robot prim not found: "
            + ROBOT_PRIM
        )

    print("Robot prim found:")
    print("  " + ROBOT_PRIM)

    # --------------------------------------------------------
    # Verify camera prims
    # --------------------------------------------------------

    print_separator("VERIFYING CAMERA PRIMS")

    left_camera_path = (
        "/World/forklift_c/"
        "body/sensors/front_stereo_camera/"
        "left/camera_left"
    )

    right_camera_path = (
        "/World/forklift_c/"
        "body/sensors/front_stereo_camera/"
        "right/camera_right"
    )

    left_camera = stage.GetPrimAtPath(
        left_camera_path
    )

    right_camera = stage.GetPrimAtPath(
        right_camera_path
    )

    if not left_camera.IsValid():
        raise RuntimeError(
            "Left camera not found: "
            + left_camera_path
        )

    if not right_camera.IsValid():
        raise RuntimeError(
            "Right camera not found: "
            + right_camera_path
        )

    print("Left:")
    print("  " + left_camera_path)

    print("Right:")
    print("  " + right_camera_path)

    # --------------------------------------------------------
    # Print TF values
    # --------------------------------------------------------

    print_separator("CAMERA TF VALUES")

    print()
    print("base_link -> " + LEFT_CAMERA_FRAME)

    print(
        "  Translation:"
        " x={:.15f}"
        " y={:.15f}"
        " z={:.15f}".format(
            LEFT_CAMERA_TRANSLATION[0],
            LEFT_CAMERA_TRANSLATION[1],
            LEFT_CAMERA_TRANSLATION[2]
        )
    )

    print(
        "  Quaternion:"
        " x={:.15f}"
        " y={:.15f}"
        " z={:.15f}"
        " w={:.15f}".format(
            CAMERA_ROTATION[0],
            CAMERA_ROTATION[1],
            CAMERA_ROTATION[2],
            CAMERA_ROTATION[3]
        )
    )

    print()
    print("base_link -> " + RIGHT_CAMERA_FRAME)

    print(
        "  Translation:"
        " x={:.15f}"
        " y={:.15f}"
        " z={:.15f}".format(
            RIGHT_CAMERA_TRANSLATION[0],
            RIGHT_CAMERA_TRANSLATION[1],
            RIGHT_CAMERA_TRANSLATION[2]
        )
    )

    print(
        "  Quaternion:"
        " x={:.15f}"
        " y={:.15f}"
        " z={:.15f}"
        " w={:.15f}".format(
            CAMERA_ROTATION[0],
            CAMERA_ROTATION[1],
            CAMERA_ROTATION[2],
            CAMERA_ROTATION[3]
        )
    )

    # --------------------------------------------------------
    # Create ROS2 graph
    # --------------------------------------------------------

    print_separator(
        "CREATING GROUND TRUTH ROS2 ACTION GRAPH"
    )

    print("Graph:")
    print("  " + GRAPH_PATH)

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

                # ---------------------------------------------
                # Simulation tick
                # ---------------------------------------------

                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick"
                ),

                # ---------------------------------------------
                # Simulation time
                # ---------------------------------------------

                (
                    "ReadSimTime",
                    "isaacsim.core.nodes.IsaacReadSimulationTime"
                ),

                # ---------------------------------------------
                # ROS2 context
                # ---------------------------------------------

                (
                    "ROS2Context",
                    "isaacsim.ros2.bridge.ROS2Context"
                ),

                # ---------------------------------------------
                # Ground truth odometry
                # ---------------------------------------------

                (
                    "ComputeOdom",
                    "isaacsim.core.nodes.IsaacComputeOdometry"
                ),

                # ---------------------------------------------
                # Odom publisher
                # ---------------------------------------------

                (
                    "PublishOdom",
                    "isaacsim.ros2.bridge.ROS2PublishOdometry"
                ),

                # ---------------------------------------------
                # odom -> base_link TF
                # ---------------------------------------------

                (
                    "PublishOdomTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"
                ),

                # ---------------------------------------------
                # base_link -> LEFT camera
                # ---------------------------------------------

                (
                    "PublishLeftCameraTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"
                ),

                # ---------------------------------------------
                # base_link -> RIGHT camera
                # ---------------------------------------------

                (
                    "PublishRightCameraTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"
                ),

                # ---------------------------------------------
                # ROS2 clock
                # ---------------------------------------------

                (
                    "PublishClock",
                    "isaacsim.ros2.bridge.ROS2PublishClock"
                ),
            ],

            # =================================================
            # SET VALUES
            # =================================================

            keys.SET_VALUES: [

                # =================================================
                # COMPUTE ODOM
                # =================================================

                (
                    "ComputeOdom.inputs:chassisPrim",
                    [
                        ROBOT_PRIM
                    ]
                ),

                # =================================================
                # ODOMETRY
                # =================================================

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

                # =================================================
                # ODOM -> BASE_LINK TF
                # =================================================

                (
                    "PublishOdomTF.inputs:topicName",
                    TF_TOPIC
                ),

                (
                    "PublishOdomTF.inputs:parentFrameId",
                    ODOM_FRAME
                ),

                (
                    "PublishOdomTF.inputs:childFrameId",
                    BASE_FRAME
                ),

                (
                    "PublishOdomTF.inputs:queueSize",
                    10
                ),

                (
                    "PublishOdomTF.inputs:staticPublisher",
                    False
                ),

                # =================================================
                # LEFT CAMERA TF
                # =================================================

                (
                    "PublishLeftCameraTF.inputs:topicName",
                    TF_STATIC_TOPIC
                ),

                (
                    "PublishLeftCameraTF.inputs:parentFrameId",
                    BASE_FRAME
                ),

                (
                    "PublishLeftCameraTF.inputs:childFrameId",
                    LEFT_CAMERA_FRAME
                ),

                (
                    "PublishLeftCameraTF.inputs:translation",
                    LEFT_CAMERA_TRANSLATION
                ),

                (
                    "PublishLeftCameraTF.inputs:rotation",
                    CAMERA_ROTATION
                ),

                (
                    "PublishLeftCameraTF.inputs:queueSize",
                    10
                ),

                (
                    "PublishLeftCameraTF.inputs:staticPublisher",
                    True
                ),

                # =================================================
                # RIGHT CAMERA TF
                # =================================================

                (
                    "PublishRightCameraTF.inputs:topicName",
                    TF_STATIC_TOPIC
                ),

                (
                    "PublishRightCameraTF.inputs:parentFrameId",
                    BASE_FRAME
                ),

                (
                    "PublishRightCameraTF.inputs:childFrameId",
                    RIGHT_CAMERA_FRAME
                ),

                (
                    "PublishRightCameraTF.inputs:translation",
                    RIGHT_CAMERA_TRANSLATION
                ),

                (
                    "PublishRightCameraTF.inputs:rotation",
                    CAMERA_ROTATION
                ),

                (
                    "PublishRightCameraTF.inputs:queueSize",
                    10
                ),

                (
                    "PublishRightCameraTF.inputs:staticPublisher",
                    True
                ),

                # =================================================
                # CLOCK
                # =================================================

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
                # TICK -> ODOM COMPUTATION
                # =================================================

                (
                    "OnPlaybackTick.outputs:tick",
                    "ComputeOdom.inputs:execIn"
                ),

                # =================================================
                # TICK -> ODOM TF
                # =================================================

                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishOdomTF.inputs:execIn"
                ),

                # =================================================
                # TICK -> LEFT CAMERA STATIC TF
                # =================================================

                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishLeftCameraTF.inputs:execIn"
                ),

                # =================================================
                # TICK -> RIGHT CAMERA STATIC TF
                # =================================================

                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishRightCameraTF.inputs:execIn"
                ),

                # =================================================
                # TICK -> CLOCK
                # =================================================

                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishClock.inputs:execIn"
                ),

                # =================================================
                # ODOM COMPUTATION -> ODOM PUBLISHER
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

                # =================================================
                # SIM TIME -> ODOM
                # =================================================

                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdom.inputs:timeStamp"
                ),

                # =================================================
                # ODOM COMPUTATION -> ODOM TF
                # =================================================

                (
                    "ComputeOdom.outputs:position",
                    "PublishOdomTF.inputs:translation"
                ),

                (
                    "ComputeOdom.outputs:orientation",
                    "PublishOdomTF.inputs:rotation"
                ),

                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdomTF.inputs:timeStamp"
                ),

                # =================================================
                # SIM TIME -> CLOCK
                # =================================================

                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishClock.inputs:timeStamp"
                ),

                # =================================================
                # ROS2 CONTEXT -> ODOM
                # =================================================

                (
                    "ROS2Context.outputs:context",
                    "PublishOdom.inputs:context"
                ),

                # =================================================
                # ROS2 CONTEXT -> ODOM TF
                # =================================================

                (
                    "ROS2Context.outputs:context",
                    "PublishOdomTF.inputs:context"
                ),

                # =================================================
                # ROS2 CONTEXT -> LEFT CAMERA TF
                # =================================================

                (
                    "ROS2Context.outputs:context",
                    "PublishLeftCameraTF.inputs:context"
                ),

                # =================================================
                # ROS2 CONTEXT -> RIGHT CAMERA TF
                # =================================================

                (
                    "ROS2Context.outputs:context",
                    "PublishRightCameraTF.inputs:context"
                ),

                # =================================================
                # ROS2 CONTEXT -> CLOCK
                # =================================================

                (
                    "ROS2Context.outputs:context",
                    "PublishClock.inputs:context"
                ),
            ],
        },
    )

    print()
    print("ROS2 graph created successfully.")

    # --------------------------------------------------------
    # Start simulation
    # --------------------------------------------------------

    timeline = (
        omni.timeline.get_timeline_interface()
    )

    timeline.play()

    # Give graph enough time to initialize.
    for _ in range(30):
        simulation_app.update()

    # --------------------------------------------------------
    # STATUS
    # --------------------------------------------------------

    print_separator(
        "GROUND TRUTH ROS2 RUNNING"
    )

    print()
    print("ODOMETRY")
    print("  Topic:")
    print("    " + ODOM_TOPIC)
    print("  TF:")
    print(
        "    " + ODOM_FRAME
        + " -> "
        + BASE_FRAME
    )

    print()
    print("CAMERA TF")

    print(
        "  "
        + BASE_FRAME
        + " -> "
        + LEFT_CAMERA_FRAME
    )

    print(
        "  "
        + BASE_FRAME
        + " -> "
        + RIGHT_CAMERA_FRAME
    )

    print()
    print("Static TF topic:")
    print("  " + TF_STATIC_TOPIC)

    print()
    print("Clock:")
    print("  " + CLOCK_TOPIC)

    print()
    print("Expected TF tree:")
    print()
    print("             odom")
    print("               |")
    print("               |")
    print("           base_link")
    print("            /      \\")
    print("           /        \\")
    print("          /          \\")
    print("     left_optical   right_optical")

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
    print_separator("ERROR")

    print(
        type(e).__name__
    )

    print(
        str(e)
    )

    raise


finally:

    print()
    print("Closing Isaac Sim...")

    simulation_app.close()
