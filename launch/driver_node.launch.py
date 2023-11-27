# -----------------------------------------------------------------------------
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
#

import launch
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Create simple node."""
    cam_name = LaunchConfig("camera_name")
    cam_str = cam_name.perform(context)
    node = Node(
        package="libcaer_driver",
        executable="driver_node",
        output="screen",
        # prefix=["xterm -e gdb -ex run --args"],
        name=cam_name,
        parameters=[
            {
                "device_type": "dvxplorer",  # "davis",
                "device_id": 1,
                "serial": "",
                "statistics_print_interval": 2.0,
                "camerainfo_url": "",
                "frame_id": "",
                "event_message_time_threshold": 1.0e-3,
            },
        ],
        remappings=[("~/events", cam_str + "/events")],
    )
    return [node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg(
                "camera_name", default_value=["event_camera"], description="camera name"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
