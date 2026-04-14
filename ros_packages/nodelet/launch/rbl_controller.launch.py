import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = "rbl_controller_node"
    pkg_path = get_package_share_directory(pkg_name)

    ld = LaunchDescription()

    # -------------------------------------------------
    # UAV name
    # -------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        "uav_name",
        default_value=EnvironmentVariable("UAV_NAME", default_value="uav1")
    ))
    uav_name = LaunchConfiguration("uav_name")

    # -------------------------------------------------
    # run type
    # -------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        "run_type",
        default_value="simulation"
    ))
    run_type = LaunchConfiguration("run_type")

    # -------------------------------------------------
    # config file (DEFAULT + OVERRIDE SUPPORT)
    # -------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        "config",
        default_value=os.path.join(pkg_path, "config", "default.yaml")
    ))
    config = LaunchConfiguration("config")

    # -------------------------------------------------
    # standalone mode
    # -------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        "standalone",
        default_value="true"
    ))
    standalone = LaunchConfiguration("standalone")

    # -------------------------------------------------
    # container name (if attaching externally)
    # -------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        "container_name",
        default_value=""
    ))
    container_name = LaunchConfiguration("container_name")

    # -------------------------------------------------
    # RBL Controller component
    # -------------------------------------------------
    rbl_controller_node = ComposableNode(

        package=pkg_name,
        plugin="rbl_controller::WrapperRosRBL",
        name="rbl_controller",
        namespace=uav_name,

        parameters=[
            {"config": config},
            {"simulation": True},
            {"uav_name": uav_name},
            {"control_frame": [uav_name, "/world_origin"]},
        ],

        remappings=[
            ("~/odom_in", "estimation_manager/odom_main"),
            ("~/alt_in", "estimation_manager/garmin_agl/agl_height"),
            ("~/pcl_in", "losos_server/current_submap_pc"),
            ("~/octomap_in", "octomap_server/octomap_local_binary"),
            ("~/group_states_in", "filter_reflective_uavs/pose_vel"),
            ("~/tracker_cmd_in", "control_manager/tracker_cmd"),
            ("~/ref_out", "control_manager/reference"),
            ("~/goto_out", "~/goto"),
            ("~/control_activation_in", "~/activation"),
            ("~/control_activation_params_in", "~/activation_params"),
            ("~/control_deactivation_in", "~/deactivation"),
            ("~/fly_to_start_in", "~/fly_to_start"),
            ("~/destination_out", "~/destination_vis"),
            ("~/centroid_out", "~/centroid_vis"),
            ("~/position_out", "~/position_vis"),
            ("~/obstacle_markers_out", "~/obstacles_vis"),
            ("~/neighbors_markers_out", "~/neighbors_vis"),
            ("~/hull_markers_out", "~/hull_vis"),
            ("~/line_out", "~/line_vis"),
        ],
    )

    # -------------------------------------------------
    # load into external container (if used)
    # -------------------------------------------------
    ld.add_action(
        LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[rbl_controller_node],
            condition=UnlessCondition(standalone)
        )
    )

    # -------------------------------------------------
    # standalone container (default mode)
    # -------------------------------------------------
    ld.add_action(
        ComposableNodeContainer(
            namespace=uav_name,
            name="rbl_controller_container",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[rbl_controller_node],
            output="screen",
            condition=IfCondition(standalone)
        )
    )

    return ld
