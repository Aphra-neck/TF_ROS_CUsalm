# Copyright (c) 2026 u5-4
# SPDX-License-Identifier: Apache-2.0

# Launch-selected, locally aligned pose candidate. It is intentionally
# incompatible with nav_msgs/Odometry and MAVROS pose inputs.
std_msgs/Header header
string semantic_child_frame
geometry_msgs/Pose pose
string selector_contract_id
string localization_epoch_id
string mode
string source_contract_id
string authorization
