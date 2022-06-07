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
from launch.conditions import IfCondition, LaunchConfigurationEquals
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
    communication_delay_compensator_path = LaunchConfiguration("communication_delay_compensator_path").perform(context)
    with open(communication_delay_compensator_path, "r") as f:
        communication_delay_compensator_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # Communication Delay Compensator
    communication_delay_component = ComposableNode(
        package="communication_delay_compensator",
        plugin="autoware::motion::control::observer::CommunicationDelayCompensatorNode",
        name="communication_delay_compensation_node_exe",
        namespace="observer",
        # remappings=[
        #     ("~/input/current_trajectory", "/planning/scenario_planning/trajectory"),
        #     ("~/input/current_odometry", "/localization/kinematic_state"),
        #     ("~/output/control_cmd", "longitudinal/control_cmd"),
        #     ("~/output/slope_angle", "longitudinal/slope_angle"),
        #     ("~/output/diagnostic", "longitudinal/diagnostic"),
        # ],
        parameters=[
            communication_delay_compensator_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],

    )

    communication_delay_container = ComposableNodeContainer(
        name="observer_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[],
        condition=LaunchConfigurationEquals("communication_delay_compensation_mode", "CDOB"),
    )

    communication_delay_compensator_loader = LoadComposableNodes(
        composable_node_descriptions=[communication_delay_component],
        target_container=communication_delay_container,
        condition=LaunchConfigurationEquals("communication_delay_compensation_mode", "CDOB"),
    )

    group = GroupAction(
        [
            PushRosNamespace("observer"),
            communication_delay_container,
            communication_delay_compensator_loader,

        ]
    )

    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # communication delay compensation mode
    add_launch_arg(
        "communication_delay_compensation_mode",
        "CDOB",
        "lateral communication_delay_compensation_mode: `CDOB` or `...`",
    )

    # parameter file path
    add_launch_arg(
        "communication_delay_compensator_path",
        [
            FindPackageShare("observers_launch"),
            "/config/communication_delay_compensator/communication_delay_compensator.param.yaml",
        ],
        "path to the parameter file of communication_delay_compensator. default is `CDOB`",
    )

    # component
    add_launch_arg("use_intra_process", "false", "use ROS2 component container communication")
    add_launch_arg("use_multithread", "false", "use multithread")
    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )
    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )
    return launch.LaunchDescription(
        launch_arguments
        + [
            set_container_executable,
            set_container_mt_executable,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
