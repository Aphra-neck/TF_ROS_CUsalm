# Copyright 2026 u5-4
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

"""Launch the fixed mocap selector and pose-only PX4 output gateway."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Create the immutable mocap-primary output path."""
    gateway_share = get_package_share_directory('localization_output_gateway')
    selector_share = get_package_share_directory('localization_source_selector')
    gateway_contract = os.path.join(
        gateway_share, 'config', 'mocap_primary.contract.yaml')
    selector_launch = os.path.join(
        selector_share, 'launch', 'mocap_primary_selector.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(selector_launch)),
        Node(
            package='localization_output_gateway',
            executable='localization_output_gateway_node',
            name='localization_output_gateway',
            namespace='/',
            output='screen',
            parameters=[{'contract_file': gateway_contract}],
            respawn=False,
        ),
    ])
