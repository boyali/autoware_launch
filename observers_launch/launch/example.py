# Copyright 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    lat_controller_param_path = LaunchConfiguration("lat_controller_param_path").perform(context)
    with open(lat_controller_param_path, "r") as f:
        lat_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # lateral controller
    mpc_follower_component = ComposableNode(
        package="trajectory_follower_nodes",
        plugin="autoware::motion::control::trajectory_follower_nodes::LateralController",
        name="lateral_controller_node_exe",
        namespace="trajectory_follower",
        remappings=[
            ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
            ("~/input/current_odometry", "/localization/kinematic_state"),
            ("~/input/current_steering", "/vehicle/status/steering_status"),
            ("~/output/control_cmd", "lateral/control_cmd"),
            ("~/output/predicted_trajectory", "lateral/predicted_trajectory"),
            ("~/output/diagnostic", "lateral/diagnostic"),
        ],
        parameters=[
            lat_controller_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # set container to run all required components in the same process
    mpc_follower_container = ComposableNodeContainer(
        name="control_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            lon_controller_component,
            latlon_muxer_component,
            lane_departure_component,
            shift_decider_component,
            vehicle_cmd_gate_component,
        ],
        condition=LaunchConfigurationEquals("lateral_controller_mode", "mpc_follower"),
    )

    # lateral controller is separated since it may be another controller (e.g. pure pursuit)
    mpc_follower_loader = LoadComposableNodes(
        composable_node_descriptions=[mpc_follower_component],
        target_container=mpc_follower_container,
        condition=LaunchConfigurationEquals("lateral_controller_mode", "mpc_follower"),
    )

    group = GroupAction(
        [
            PushRosNamespace("control"),
            mpc_follower_container,
            pure_pursuit_container,
            external_cmd_selector_loader,
            external_cmd_converter_loader,
            mpc_follower_loader,
            pure_pursuit_loader,
        ]
    )

    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # lateral controller mode
    add_launch_arg(
        "lateral_controller_mode",
        "mpc_follower",
        "lateral controller mode: `mpc_follower` or `pure_pursuit`",
    )

    return launch.LaunchDescription(
        launch_arguments
        + [
            set_container_executable,
            set_container_mt_executable,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
