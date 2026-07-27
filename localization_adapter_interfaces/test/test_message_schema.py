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

"""Regression tests for the pose-only localization candidate schemas."""

from pathlib import Path

import pytest


MESSAGE_DIRECTORY = Path(__file__).resolve().parents[1] / 'msg'

EXPECTED_SCHEMAS = {
    'LocalizationSourceCandidate.msg': [
        ('std_msgs/Header', 'header'),
        ('string', 'semantic_child_frame'),
        ('geometry_msgs/Pose', 'pose'),
        ('string', 'source_id'),
        ('string', 'source_contract_id'),
        ('string', 'authorization'),
    ],
    'SelectedPoseCandidate.msg': [
        ('std_msgs/Header', 'header'),
        ('string', 'semantic_child_frame'),
        ('geometry_msgs/Pose', 'pose'),
        ('string', 'selector_contract_id'),
        ('string', 'localization_epoch_id'),
        ('string', 'mode'),
        ('string', 'source_contract_id'),
        ('string', 'authorization'),
    ],
}


def read_fields(message_name):
    """Return ordered type/name declarations from a ROS message source."""
    declarations = []
    message_path = MESSAGE_DIRECTORY / message_name
    for raw_line in message_path.read_text(encoding='utf-8').splitlines():
        declaration = raw_line.split('#', maxsplit=1)[0].strip()
        if declaration:
            field_type, field_name = declaration.split()
            declarations.append((field_type, field_name))
    return declarations


@pytest.mark.parametrize('message_name', EXPECTED_SCHEMAS)
def test_candidate_schema_is_exactly_pose_only(message_name):
    """Candidate messages retain their ordered fields and pose-only scope."""
    actual_schema = read_fields(message_name)

    assert actual_schema == EXPECTED_SCHEMAS[message_name]

    field_names = {field_name.lower() for _, field_name in actual_schema}
    field_types = {field_type.lower() for field_type, _ in actual_schema}
    assert not any('twist' in name for name in field_names)
    assert not any('covariance' in name for name in field_names)
    assert not any('twist' in field_type for field_type in field_types)
    assert not any('covariance' in field_type for field_type in field_types)
