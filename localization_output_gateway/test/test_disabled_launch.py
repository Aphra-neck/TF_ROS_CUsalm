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

"""Verify the packaged default gateway graph through its public ROS seam."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing import actions
from launch_testing import asserts
import pytest
import rclpy
from rclpy.node import NodeNameNonExistentError
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


@pytest.mark.launch_test
def generate_test_description():
    """Include the installed default launch under test."""
    share = get_package_share_directory('localization_output_gateway')
    launch_file = os.path.join(
        share, 'launch', 'localization_output_gateway_disabled.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
        actions.ReadyToTest(),
    ])


class TestDisabledGatewayGraph(unittest.TestCase):
    """Observe only the node's public ROS graph interface."""

    @classmethod
    def setUpClass(cls):
        """Create an observer in the launch test's ROS context."""
        rclpy.init()
        cls.observer = rclpy.create_node('disabled_gateway_graph_observer')
        cls.diagnostics = []
        diagnostic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        cls.diagnostic_subscription = cls.observer.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            cls.diagnostics.append,
            diagnostic_qos,
        )

    @classmethod
    def tearDownClass(cls):
        """Release the observer ROS context."""
        cls.observer.destroy_subscription(cls.diagnostic_subscription)
        cls.observer.destroy_node()
        rclpy.shutdown()

    def wait_for_gateway_interface(self, timeout_sec):
        """Wait for the complete passive graph and one denied diagnostic."""
        expected_diagnostic_values = {
            'gateway_contract_id':
                'yopo_localization_output_gateway_disabled_20260724_v1',
            'profile': 'disabled',
            'state': 'disabled',
            'authorization': 'denied',
            'external_vision_output_authorization': 'denied',
            'reason_code': 'OUTPUT_AUTHORIZATION_DENIED',
        }
        node_identity = ('localization_output_gateway', '/')
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.observer, timeout_sec=0.05)
            if node_identity not in self.observer.get_node_names_and_namespaces():
                continue
            try:
                publishers = dict(
                    self.observer.get_publisher_names_and_types_by_node(
                        *node_identity))
                subscriptions = dict(
                    self.observer.get_subscriber_names_and_types_by_node(
                        *node_identity))
            except NodeNameNonExistentError:
                continue
            diagnostics_denied = any(
                all(
                    {
                        value.key: value.value
                        for value in status.values
                    }.get(key) == expected
                    for key, expected in expected_diagnostic_values.items()
                )
                for message in self.diagnostics
                for status in message.status
                if status.name ==
                '/localization_output_gateway: localization output gateway'
            )
            if (
                publishers.get('/diagnostics') ==
                ['diagnostic_msgs/msg/DiagnosticArray'] and
                subscriptions.get('/localization/selected/pose') ==
                ['localization_adapter_interfaces/msg/SelectedPoseCandidate'] and
                diagnostics_denied
            ):
                return publishers, subscriptions
        return None

    def assert_contract_qos(self, endpoint):
        """Require the reliable, volatile, keep-last-10 contract QoS."""
        self.assertEqual(
            endpoint.qos_profile.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(
            endpoint.qos_profile.durability, DurabilityPolicy.VOLATILE)
        self.assertIn(
            endpoint.qos_profile.history,
            (HistoryPolicy.KEEP_LAST, HistoryPolicy.UNKNOWN))
        if endpoint.qos_profile.history == HistoryPolicy.KEEP_LAST:
            self.assertEqual(endpoint.qos_profile.depth, 10)

    def test_default_launch_exposes_only_the_passive_gateway_graph(self):
        """Require selected input and diagnostics, with no privileged output."""
        interface = self.wait_for_gateway_interface(5.0)
        self.assertIsNotNone(
            interface, 'packaged disabled gateway interface did not converge')
        publishers, subscriptions = interface

        forbidden_publishers = {
            '/localization/odometry',
            '/mavros/vision_pose/pose_cov',
            '/mavros/vision_pose/pose',
            '/mavros/mocap/pose',
            '/mavros/odometry/out',
            '/state/odom',
            '/tf',
            '/tf_static',
            '/mavros/setpoint_raw/attitude',
        }
        allowed_gateway_publishers = {
            '/diagnostics',
            '/parameter_events',
            '/rosout',
        }
        allowed_gateway_subscriptions = {
            '/localization/selected/pose',
            '/parameter_events',
        }
        self.assertTrue(set(publishers).issubset(allowed_gateway_publishers))
        self.assertTrue(
            set(subscriptions).issubset(allowed_gateway_subscriptions))
        self.assertIn('/localization/selected/pose', subscriptions)
        for topic in forbidden_publishers:
            self.assertEqual(
                len(self.observer.get_publishers_info_by_topic(topic)), 0,
                f'forbidden publisher discovered on {topic}')

        mavros_publishers = {
            topic
            for topic, _types in self.observer.get_topic_names_and_types()
            if topic.startswith('/mavros/') and
            self.observer.get_publishers_info_by_topic(topic)
        }
        self.assertEqual(mavros_publishers, set())

        diagnostic_endpoints = [
            endpoint
            for endpoint in self.observer.get_publishers_info_by_topic('/diagnostics')
            if endpoint.node_name == 'localization_output_gateway' and
            endpoint.node_namespace == '/'
        ]
        self.assertEqual(len(diagnostic_endpoints), 1)
        self.assert_contract_qos(diagnostic_endpoints[0])

        input_endpoints = [
            endpoint
            for endpoint in self.observer.get_subscriptions_info_by_topic(
                '/localization/selected/pose')
            if endpoint.node_name == 'localization_output_gateway' and
            endpoint.node_namespace == '/'
        ]
        self.assertEqual(len(input_endpoints), 1)
        self.assert_contract_qos(input_endpoints[0])


@launch_testing.post_shutdown_test()
class TestGatewayShutdown(unittest.TestCase):
    """Require every launched process to exit cleanly."""

    def test_exit_codes(self, proc_info):
        """Reject a node that satisfies the graph and then crashes."""
        asserts.assertExitCodes(proc_info)
