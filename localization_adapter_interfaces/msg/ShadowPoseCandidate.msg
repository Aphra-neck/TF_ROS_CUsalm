# Copyright (c) 2026 u5-4
# SPDX-License-Identifier: Apache-2.0

# Source-private validation message. It is intentionally incompatible with
# nav_msgs/Odometry and MAVROS pose inputs.
std_msgs/Header header
string semantic_child_frame
geometry_msgs/Pose pose

string contract_id
string authorization
string source_topic
string source_parent_frame
string source_child_frame
string source_world_axes
string source_body_axes
bool geographic_alignment_validated

string world_alignment_status
bool world_alignment_approved

string extrinsic_status
string extrinsic_assumption_id
bool extrinsic_approved
string expected_source_revision
string expected_timestamp_semantics
bool source_configuration_validated
bool capture_time_validated
