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

"""Launch the immutable cuVSLAM-primary localization selector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the immutable cuVSLAM-primary selector."""
    share = get_package_share_directory('localization_source_selector')
    contract = os.path.join(share, 'config', 'cuvslam_primary.contract.yaml')
    return LaunchDescription([
        Node(
            package='localization_source_selector',
            executable='localization_source_selector_node',
            name='localization_source_selector',
            namespace='/',
            output='screen',
            parameters=[{
                'mode': 'cuvslam_primary',
                'contract_file': contract,
            }],
        ),
    ])
