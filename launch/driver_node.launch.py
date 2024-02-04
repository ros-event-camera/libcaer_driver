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
    node = Node(
        package='libcaer_driver',
        executable='driver_node',
        output='screen',
        # prefix=["xterm -e gdb -ex run --args"],
        name=LaunchConfig('camera_name'),
        parameters=[
            {
                'device_type': LaunchConfig('device_type'),
                'device_id': 1,
                'master': LaunchConfig('master'),
                'serial': '',
                'statistics_print_interval': 2.0,
                'camerainfo_url': '',
                'frame_id': '',
                'event_message_time_threshold': 1.0e-3,
                'dvs_enabled': True,
                # safe to enable imu
                'imu_accel_enabled': True,
                'imu_gyro_enabled': True,
                # aps affects event quality, disable when not needed
                'aps_enabled': False,
                # other example settings
                # "aps_exposure": 4000,
                # "aps_frame_interval": 40000,
                # "auto_exposure_enabled": False,
                # "auto_exposure_illumination": 127,
                #
                # "subsample_enabled": False,
                # "subsample_horizontal": 3,
                # "bias_sensitivity": 2,  # for dvxplorer
                # "OFFBn_coarse": 4,  # for DAVIS
                # "OFFBn_fine": 0,  # for DAVIS
            },
        ],
        remappings=[('~/reset_timestamps', LaunchConfig('reset_topic'))],
    )
    return [node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg('camera_name', default_value=['event_camera'], description='camera name'),
            LaunchArg(
                'device_type',
                default_value=['davis'],
                description='device type (davis, dvxplorer...)',
            ),
            LaunchArg(
                'master',
                default_value=['True'],
                description='set to true for this camera to be master',
            ),
            LaunchArg(
                'reset_topic',
                default_value=['~/reset_timestamps'],
                description='on the slave, set this to the masters reset topic',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
