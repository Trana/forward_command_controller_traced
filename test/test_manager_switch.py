"""Real Jazzy manager with GenericSystem only. No physical hardware plugins.

Run with ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 and an isolated built overlay.
Two sources use the forward plugin; a third uses the experimental hold-only plugin.
"""
import os
import json
from pathlib import Path
import subprocess
import time

import pytest

pytest.importorskip('rclpy')
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from marvin_trace_msgs.msg import TracedFloat64MultiArray
from controller_manager_msgs.srv import (
    LoadController, ConfigureController, SwitchController, ListControllers,
    ListHardwareInterfaces)


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '231'
                   or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
                   reason='requires isolated localhost domain 231')
@pytest.mark.parametrize('following_offset', [0., 0.01])
def test_strict_switch_transfers_exclusive_interface(following_offset):
    namespace = f'/joint_switch_probe_{os.getpid()}'
    config = Path(__file__).parent / 'fixtures/switch_probe.yaml'
    process = subprocess.Popen([
        '/opt/ros/jazzy/lib/controller_manager/ros2_control_node', '--ros-args',
        '-r', f'__ns:={namespace}', '-r', f'robot_description:={namespace}/robot_description',
        '--params-file', str(config)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True)
    rclpy.init()
    node = rclpy.create_node('switch_test', namespace=namespace)
    try:
        samples = []
        statuses = {}
        for name in ('hold_source', 'rejecting_hold'):
            node.create_subscription(String, namespace + '/' + name + '/status',
                lambda message, source=name: statuses.setdefault(source, []).append(json.loads(message.data)), 10)

        def check_status(name, phase, tracking_fault):
            first = len(statuses.get(name, []))
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                fresh = statuses.get(name, [])[first:]
                matching = [s for s in fresh if s['phase'] == phase]
                if len(matching) >= 2:
                    assert all(s['tracking_fault_latched'] == tracking_fault for s in matching)
                    assert all(s['motion_available'] is False for s in matching)
                    assert all(s['measurement_freshness'] == 'unverified' for s in matching)
                    if phase == 'holding':
                        assert matching[-1]['revision'] > matching[0]['revision']
                    return
            pytest.fail(f'Missing fresh {phase} status for {name}: {statuses.get(name)}')
        def feedback(message):
            # Normalize the deliberately injected fake tracking offset only for
            # expected-position assertions. The controller sees the raw state.
            samples.append((time.monotonic(),
                message.position[message.name.index('probe_joint')] - following_offset))
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(JointState, namespace + '/joint_states', feedback, qos)
        outputs = {name: node.create_publisher(TracedFloat64MultiArray,
                   namespace + '/' + name + '/commands_traced', qos)
                   for name in ('normal', 'test_source')}
        def send(name, value):
            message = TracedFloat64MultiArray()
            message.stamp = node.get_clock().now().to_msg()
            message.trace_id = 1  # Deliberately reused; not an ownership token.
            message.data = [value]
            outputs[name].publish(message)

        def settle(name, value):
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                send(name, value)
                rclpy.spin_once(node, timeout_sec=0.01)
                if samples and abs(samples[-1][1] - value) < 1e-9:
                    return
            pytest.fail(f'{name} did not reach {value}: {samples[-5:]}')

        def observe_hold(expected, inactive_source=None, first=None):
            if first is None:
                first = len(samples)
            deadline = time.monotonic() + 0.35
            while time.monotonic() < deadline:
                if inactive_source:
                    send(inactive_source, 0.9)
                rclpy.spin_once(node, timeout_sec=0.01)
            window = samples[first:]
            assert len(window) >= 10, 'Insufficient joint feedback'
            assert all(abs(value - expected) < 1e-9 for _, value in window), window

        publisher = node.create_publisher(String, namespace + '/robot_description',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher.publish(String(data=f'''<robot name="offline_probe">
          <link name="base"/><link name="tip"/>
          <joint name="probe_joint" type="revolute"><parent link="base"/>
            <child link="tip"/><axis xyz="0 1 0"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
          <ros2_control name="OfflineFake" type="system">
            <hardware><plugin>mock_components/GenericSystem</plugin>
              <param name="position_state_following_offset">{following_offset}</param>
            </hardware>
            <joint name="probe_joint"><command_interface name="position"/>
              <state_interface name="position"><param name="initial_value">0.2</param>
              </state_interface></joint>
          </ros2_control></robot>'''))

        def call(kind, endpoint, **values):
            client = node.create_client(kind, namespace + '/controller_manager/' + endpoint)
            try:
                assert client.wait_for_service(timeout_sec=15), endpoint
                future = client.call_async(kind.Request(**values))
                rclpy.spin_until_future_complete(node, future, timeout_sec=10)
                assert future.done(), endpoint
                return future.result()
            finally:
                node.destroy_client(client)

        # Waiting on hardware interfaces also verifies description ingestion.
        deadline = time.monotonic() + 20
        while True:
            state = call(ListHardwareInterfaces, 'list_hardware_interfaces')
            if state.command_interfaces:
                break
            assert time.monotonic() < deadline
            rclpy.spin_once(node, timeout_sec=0.1)
        for name in ('normal', 'test_source', 'hold_source', 'rejecting_hold', 'joint_state_broadcaster'):
            assert call(LoadController, 'load_controller', name=name).ok
            assert call(ConfigureController, 'configure_controller', name=name).ok

        def switch(activate, deactivate):
            request = SwitchController.Request()
            request.timeout.sec = 2
            return call(SwitchController, 'switch_controller',
                activate_controllers=activate, deactivate_controllers=deactivate,
                strictness=SwitchController.Request.STRICT, timeout=request.timeout).ok

        def check(owner, inactive):
            controllers = {c.name: c for c in call(ListControllers, 'list_controllers').controller}
            assert controllers[owner].state == 'active'
            assert list(controllers[owner].claimed_interfaces) == ['probe_joint/position']
            assert controllers[inactive].state == 'inactive'
            assert not controllers[inactive].claimed_interfaces
            interfaces = call(ListHardwareInterfaces, 'list_hardware_interfaces').command_interfaces
            assert len(interfaces) == 1 and interfaces[0].is_claimed

        assert switch(['normal', 'joint_state_broadcaster'], [])
        check('normal', 'test_source')
        settle('normal', 0.25)
        observe_hold(0.25, inactive_source='test_source')
        assert not switch(['test_source'], [])
        check('normal', 'test_source')
        switch_samples = len(samples)
        assert switch(['test_source'], ['normal'])
        check('test_source', 'normal')
        # Buffered inactive-source commands must not replace the inherited target.
        observe_hold(0.25, inactive_source='normal', first=switch_samples)
        settle('test_source', 0.35)
        observe_hold(0.35, inactive_source='normal')
        assert not switch(['normal'], [])
        check('test_source', 'normal')
        # Stop publishing the inactive source and let already-sent packets arrive.
        # This finite wait is a test setup, NOT a production fencing guarantee.
        observe_hold(0.35)
        switch_samples = len(samples)
        assert switch(['normal'], ['test_source'])
        check('normal', 'test_source')
        observe_hold(0.35, inactive_source='test_source', first=switch_samples)
        settle('normal', 0.4)
        observe_hold(0.4)
        switch_samples = len(samples)
        assert switch(['hold_source'], ['normal'])
        check('hold_source', 'normal')
        observe_hold(0.4, inactive_source='normal', first=switch_samples)
        check_status('hold_source', 'holding', False)
        observe_hold(0.4)
        switch_samples = len(samples)
        assert switch(['normal'], ['hold_source'])
        check('normal', 'hold_source')
        observe_hold(0.4, first=switch_samples)
        check_status('hold_source', 'inactive', True)
        if following_offset:
            # Tracking bound failure must no longer abandon the inherited hold.
            # The lifecycle unit test separately verifies the tracking latch.
            switch_samples = len(samples)
            assert switch(['rejecting_hold'], ['normal'])
            check('rejecting_hold', 'normal')
            observe_hold(0.4, inactive_source='normal', first=switch_samples)
            check_status('rejecting_hold', 'holding', True)
            observe_hold(0.4)
            assert switch(['normal'], ['rejecting_hold'])
            check('normal', 'rejecting_hold')
        assert switch([], ['normal', 'joint_state_broadcaster'])
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            output, _ = process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=5)
        print(output)
