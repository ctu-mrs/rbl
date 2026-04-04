import os
import launch

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
    # standalone container
    # -------------------------------------------------

    ld.add_action(DeclareLaunchArgument(
        "standalone",
        default_value="true"
    ))

    standalone = LaunchConfiguration("standalone")

    # -------------------------------------------------
    # config
    # -------------------------------------------------

    ld.add_action(DeclareLaunchArgument(
        "custom_config",
        default_value=""
    ))

    custom_config = LaunchConfiguration("custom_config")

    # -------------------------------------------------
    # RBL Controller component
    # -------------------------------------------------

    rbl_controller_node = ComposableNode(

        package=pkg_name,
        plugin="rbl_controller::WrapperRosRBL",
        name="rbl_controller",
        namespace=uav_name,

        parameters=[
            {"config": os.path.join(pkg_path, "config", "default.yaml")},
            {"custom_config": custom_config},
            {"simulation": True},
            {"uav_name": uav_name},
            {"control_frame": [uav_name, "/world_origin"]},
        ],

        remappings=[
            ("~/odom_in","estimation_manager/odom_main"),
            ("~/alt_in","estimation_manager/garmin_agl/agl_height"), 
            ("~/pcl_in","losos_server/current_submap_pc"), 
            ("~/pcl_in_raw","livox/points"), 
            ("~/octomap_in","octomap_server/octomap_local_binary"), 
            ("~/group_states_in", "filter_reflective_uavs/reflective_centroids_out"),
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
    # container name
    # -------------------------------------------------

    ld.add_action(DeclareLaunchArgument(
        "container_name",
        default_value=""
    ))

    container_name = LaunchConfiguration("container_name")

    # -------------------------------------------------
    # load into existing container
    # -------------------------------------------------

    ld.add_action(

        LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[rbl_controller_node],
            condition=UnlessCondition(standalone)
        )
    )

    # -------------------------------------------------
    # standalone container
    # -------------------------------------------------

    ld.add_action(

        ComposableNodeContainer(
            namespace=uav_name,
            name="rbl_controller_container",
            package="rclcpp_components",
            executable="component_container_mt",
            # prefix=['gdb -q -ex "set pagination off" -ex "handle SIGABRT stop print nopass" -ex run -ex bt -ex "thread apply all bt" --args'],
            composable_node_descriptions=[rbl_controller_node],
            output="screen",
            condition=IfCondition(standalone)
        )
    )

    return ld


# old launch file:

# <launch>

#     <!-- defines name of the namespace of the drone -->
#     <arg name="UAV_NAME" default="$(optenv UAV_NAME)" />
#     <arg name="RUN_TYPE" default="simulation" />

#     <arg name="custom_config" default="" />

#     <!-- will it run using GNU debugger? -->
#     <arg name="DEBUG" default="false" />
#     <arg unless="$(arg DEBUG)" name="launch_prefix_debug" value="" />
#     <arg if="$(arg DEBUG)" name="launch_prefix_debug" value="debug_roslaunch" />

#     <!-- will it run as standalone nodelet or using a nodelet manager? -->
#     <arg name="standalone" default="true" />
#     <arg name="manager" default="$(arg UAV_NAME)_waypointflier_manager" />
#     <arg name="n_threads" default="8" />
#     <arg unless="$(arg standalone)" name="nodelet" value="load" />
#     <arg if="$(arg standalone)" name="nodelet" value="standalone" />
#     <arg unless="$(arg standalone)" name="nodelet_manager" value="$(arg manager)" />
#     <arg if="$(arg standalone)" name="nodelet_manager" value="" />

#     <!-- Namespace - all topics, services and parameters will be remapped using this namespace as a prefix (eg.: "/waypoint_flier/odom_uav_in" to "/uav1/waypoint_flier/odom_uav_in") -->
#     <group ns="$(arg UAV_NAME)">

#         <!-- FormationPlanning nodelet -->
#         <node pkg="nodelet" type="nodelet" name="rbl_controller"
#             args="$(arg nodelet) formation_control/RBLController $(arg nodelet_manager)"
#             launch-prefix="$(arg launch_prefix_debug)" output="screen">

#         <!-- lidar clustering to measure distance to leader -->
#         <!-- <node name="lidar_clusters$(arg UAV_NAME)" pkg="rbl_controller" type="lidar_clusters" output="screen"/> -->
#             <!-- Parameters loaded from launch file -->
#             <param name="simulation" value="$(eval arg('RUN_TYPE') == 'simulation')" />
#             <param name="uav_name" value="$(arg UAV_NAME)" />
#             <!-- TODO: move this to config .yaml so you can change if simulation or experiment -->
#             <param name="control_frame" value="$(arg UAV_NAME)/world_origin" />
#             <!-- <param name="control_frame" value="$(arg UAV_NAME)/hector_origin" /> -->

#             <!-- ROS parameters config file -->
#             <rosparam file="$(find rbl_controller)/config/default.yaml" />
#             <rosparam file="$(arg custom_config)" />
#             <!-- <rosparam file="$(find formation_controller_susd)/config/$(arg RUN_TYPE).yaml" /> -->

#             <!-- Subscribers (note that the topic names will also be remapped using the namespace as a prefix) -->
#             <remap from="~tracker_cmd_in" to="control_manager/tracker_cmd" />

#             <!-- Service clients (note that the service names will also be remapped using the namespace as a prefix) -->
#             <remap from="~ref_pos_out" to="control_manager/reference" />
#             <remap from="~goto_out" to="control_manager/goto" />
#             <!-- Service servers (note that the service names will also be remapped using the namespace as a prefix) -->
#             <remap from="~control_activation_in" to="~activation" />
#             <remap from="~control_activation_params_in" to="~activation_params" />
#             <remap from="~control_deactivation_in" to="~deactivation" />
#             <remap from="~fly_to_start_in" to="~fly_to_start" />

#             <!-- Publishers (note that the service names will also be remapped using the namespace as a prefix) -->
#             <remap from="~destination_out" to="~destination_vis" />
#             <remap from="~centroid_out" to="~centroid_vis" />
#             <remap from="~position_out" to="~position_vis" />
#             <remap from="~obstacle_markers_out" to="~obstacles_vis" />
#             <remap from="~neighbors_markers_out" to="~neighbors_vis" />
#             <remap from="~hull_markers_out" to="~hull_vis" />
#             <remap from="~line_out" to="~line_vis" />

            

            

#         </node>

#     </group>

# </launch>
