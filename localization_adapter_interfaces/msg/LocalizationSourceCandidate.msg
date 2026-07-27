# Copyright (c) 2026 u5-4
# SPDX-License-Identifier: Apache-2.0

# Source-private pose candidate. This type deliberately carries no velocity,
# covariance, PX4, TF, or control authority.
std_msgs/Header header
string semantic_child_frame
geometry_msgs/Pose pose
string source_id
string source_contract_id
string authorization
