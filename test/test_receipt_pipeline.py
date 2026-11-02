"""Local synthetic feedback -> actual adapter -> manager -> hold-controller status.

No Isaac, CAN, physical hardware, policy, or production service is started.
Requires isolated builds of topic_based_ros2_control and this package.
"""
import json
import hashlib
import struct
import uuid
import os
from pathlib import Path
import subprocess
import time

import pytest
pytest.importorskip('rclpy')
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
from runtime_interfaces.srv import PrepareJointTest, ControlJointTest, GetJointTestState
from runtime_interfaces.msg import JointTestTargetSample
from controller_manager_msgs.srv import LoadController, ConfigureController, SwitchController, ListControllers
from ament_index_python.packages import get_package_prefix


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '231'
                   or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
                   reason='requires isolated localhost domain 231')
@pytest.mark.parametrize('scenario', ['hold_feedback_loss', 'playback_feedback_loss', 'completion', 'lease_expiry', 'cancel'])
@pytest.mark.parametrize('chirp', [False, True])
def test_manager_playback_and_terminal_hold(scenario, chirp):
    simultaneous=chirp and os.environ.get('JOINT_TEST_SIMULTANEOUS')=='1'
    report_only=simultaneous and os.environ.get('JOINT_TEST_SHORT_REPORT_ONLY')=='1'
    wide=report_only and os.environ.get('JOINT_TEST_WIDE_SHORT')=='1'
    ten=wide and os.environ.get('JOINT_TEST_TEN_SHORT')=='1'
    fast=ten and os.environ.get('JOINT_TEST_FAST_SHORT')=='1'
    five=fast and os.environ.get('JOINT_TEST_FIVE_HZ_SHORT')=='1'
    long_sweep=five and os.environ.get('JOINT_TEST_LONG_SWEEP')=='1'
    profile_id=('marvin-stand-simultaneous-10deg-5hz-report-only-60s-v1' if long_sweep else
                'marvin-stand-simultaneous-10deg-5hz-report-only-8s-v1' if five else
                'marvin-stand-simultaneous-10deg-4hz-report-only-8s-v1' if fast else
                'marvin-stand-simultaneous-10deg-3hz-report-only-8s-v1' if ten else
                'marvin-stand-simultaneous-7deg-3hz-report-only-8s-v1' if wide else
                'marvin-stand-simultaneous-report-only-8s-v1' if report_only else 'synthetic_manager')
    soak = os.environ.get('JOINT_TEST_RENEWAL_SOAK') == '1' and scenario == 'completion'
    playback = scenario != 'hold_feedback_loss'
    terminal_trial = scenario in ('completion', 'lease_expiry', 'cancel')
    namespace = f'/receipt_pipeline_{os.getpid()}'
    config = Path(__file__).parent / 'fixtures/receipt_pipeline.yaml'
    arguments = [
        str(Path(get_package_prefix('controller_manager')) / 'lib/controller_manager/ros2_control_node'), '--ros-args',
        '-r', f'__ns:={namespace}', '-r', f'robot_description:={namespace}/robot_description',
        '-r', f'~/robot_description:={namespace}/robot_description',
        '--params-file', str(config)]
    if playback:
        arguments += ['--params-file', str(config.with_name('receipt_playback.yaml'))]
        if chirp:
            arguments += ['--params-file', str(config.with_name('receipt_chirp.yaml'))]
            if simultaneous:
                arguments += ['--params-file', str(config.with_name('receipt_simultaneous.yaml'))]
            if report_only:
                arguments += ['--params-file', str(config.with_name('receipt_short_report_only.yaml'))]
            if wide:
                arguments += ['--params-file', str(config.with_name('receipt_wide_short.yaml'))]
            if ten:
                arguments += ['--params-file', str(config.with_name('receipt_ten_short.yaml'))]
            if fast:
                arguments += ['--params-file', str(config.with_name('receipt_fast_short.yaml'))]
            if five:
                arguments += ['--params-file', str(config.with_name('receipt_five_hz_short.yaml'))]
        if soak:
            arguments += ['--params-file', str(config.with_name('receipt_soak.yaml'))]
        if long_sweep:
            arguments += ['--params-file', str(config.with_name('receipt_long_sweep.yaml'))]
    process = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('receipt_pipeline_test', namespace=namespace)
    try:
        statuses, commands, targets = [], [], []
        samples = []
        node.create_subscription(JointTestTargetSample, namespace + '/receipt_hold/joint_test/target_samples',
            samples.append, QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE))
        node.create_subscription(JointState, namespace + '/receipt_hold/joint_test/applied_targets',
            targets.append, qos_profile_sensor_data)
        node.create_subscription(String, namespace + '/receipt_hold/status',
            lambda msg: statuses.append(json.loads(msg.data)), 10)
        node.create_subscription(JointState, namespace + '/commands',
            lambda msg: commands.append(list(msg.position)), 100)
        publisher = node.create_publisher(JointState, namespace + '/feedback', 1)
        competing = node.create_publisher(Float64MultiArray, namespace + '/normal/commands', 1)
        sending = [True]
        measured = [.2]
        def publish_feedback():
            if sending[0]:
                msg = JointState()
                msg.name, msg.position = ['probe_joint'], [measured[0]]
                msg.header.stamp.sec = 1  # Receipt time must not use this simulated stamp.
                publisher.publish(msg)
        node.create_timer(0.02, publish_feedback)
        description = node.create_publisher(String, namespace + '/robot_description',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        description.publish(String(data=f'''<robot name="offline_receipt_probe">
          <link name="base"/><link name="tip"/>
          <joint name="probe_joint" type="revolute"><parent link="base"/>
            <child link="tip"/><axis xyz="0 1 0"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
          <ros2_control name="ReceiptProbe" type="system">
            <hardware><plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
              <param name="export_feedback_receipt_timing">true</param>
              <param name="joint_states_topic">{namespace}/feedback</param>
              <param name="joint_commands_topic">{namespace}/commands</param>
              <param name="trigger_joint_command_threshold">-1</param>
            </hardware>
            <joint name="probe_joint"><command_interface name="position"/>
              <state_interface name="position"><param name="initial_value">0.2</param></state_interface>
              <state_interface name="feedback_receipt_monotonic_s"/>
            </joint>
          </ros2_control></robot>'''))

        clients = {}
        def call(kind, endpoint, group='controller_manager', **values):
            key = (kind, group, endpoint)
            if key not in clients:
                client = node.create_client(kind, namespace + '/' + group + '/' + endpoint)
                assert client.wait_for_service(timeout_sec=15), endpoint
                clients[key] = client
            future = clients[key].call_async(kind.Request(**values))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            assert future.done(), endpoint
            return future.result()

        assert call(LoadController, 'load_controller', name='receipt_hold').ok
        assert call(ConfigureController, 'configure_controller', name='receipt_hold').ok
        # Feed the hardware before activation so its initial freshness latch can pass.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        assert not targets, 'Inactive controller must not publish applied targets'
        assert not samples
        request = SwitchController.Request()
        request.timeout.sec = 2
        if playback:
            assert call(LoadController, 'load_controller', name='normal').ok
            assert call(ConfigureController, 'configure_controller', name='normal').ok
            assert call(SwitchController, 'switch_controller', activate_controllers=['normal'],
                strictness=SwitchController.Request.STRICT, timeout=request.timeout).ok
        assert call(SwitchController, 'switch_controller', activate_controllers=['receipt_hold'],
            deactivate_controllers=['normal'] if playback else [],
            strictness=SwitchController.Request.STRICT, timeout=request.timeout).ok

        def observe(freshness, fault):
            first = len(statuses)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                matching = [s for s in statuses[first:] if s['phase'] == 'holding'
                            and s['measurement_freshness'] == freshness]
                if len(matching) >= 2:
                    assert all(s['feedback_fault_latched'] is fault for s in matching)
                    assert matching[-1]['revision'] > matching[0]['revision']
                    assert all(s['motion_available'] is False for s in matching)
                    assert all(s['tracking_fault_latched'] is False for s in matching)
                    return
            pytest.fail(f'Missing {freshness}/{fault}: {statuses[-10:]}')

        observe('fresh_receipt', False)
        assert targets
        assert all(t.name == ['probe_joint'] and list(t.position) == [0.2] for t in targets)
        assert all(not t.velocity and not t.effort for t in targets)
        assert all(t.header.stamp.sec > 1 for t in targets), 'Use manager time, not feedback stamp'
        assert samples and all(s.joint_names == ['probe_joint'] for s in samples)
        assert all(list(s.positions) == [0.2] for s in samples)
        assert all(s.controller_instance_id and s.activation_generation > 0 for s in samples)
        assert all(a.update_sequence+1 == b.update_sequence and
                   a.sampled_at_monotonic_s < b.sampled_at_monotonic_s
                   for a, b in zip(samples, samples[1:]))
        if playback:
            conflict = call(SwitchController, 'switch_controller', activate_controllers=['normal'],
                strictness=SwitchController.Request.STRICT, timeout=request.timeout)
            # Humble can acknowledge a switch despite rejecting resource conflict.
            # Actual ownership, not the acknowledgement, is the required invariant.
            ownership = {c.name: c for c in call(ListControllers, 'list_controllers').controller}
            assert ownership['normal'].state == 'inactive', (conflict, ownership)
            assert not ownership['normal'].claimed_interfaces
            assert ownership['receipt_hold'].state == 'active'
            assert list(ownership['receipt_hold'].claimed_interfaces) == ['probe_joint/position']
            state = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
            def encoded(text):
                raw = text.encode()
                return struct.pack('>I', len(raw)) + raw
            # Independent canonical hash of this synthetic fixture, not a trusted client limit.
            canonical = (b'joint-test-profile.v1\0' + encoded('synthetic_manager')
                + struct.pack('>dII', 30. if soak else 2., 2, 1) + encoded('probe_joint')
                + struct.pack('>6d', -1., 1., 1., 10., .05, .01))
            if chirp:
                canonical = (b'joint-test-profile.v2-chirp\0' + encoded(profile_id)
                    + struct.pack('>dIddI', 61. if long_sweep else 30. if soak else 2., 1, .1 if report_only else 1., 5. if five else 4. if fast else 3. if wide else 2., 1) + encoded('probe_joint')
                    + struct.pack('>6d', -1., 1., 1., 10., .05, .01))
                if simultaneous:
                    canonical=canonical.replace(b'joint-test-profile.v2-chirp\0',b'joint-test-profile.v3-simultaneous\0',1)+struct.pack('>d',1.)
            prepared = call(PrepareJointTest, 'prepare', group='receipt_hold/joint_test',
                controller_instance_id=state.controller_instance_id,
                activation_generation=state.activation_generation, operation_id=str(uuid.uuid4()),
                profile_id=profile_id, profile_sha256=hashlib.sha256(canonical).hexdigest(),
                joint_name='__all__' if simultaneous else 'probe_joint', amplitude_rad=.005, transition_duration_s=.05,
                excitation_duration_s=60. if long_sweep and scenario=='completion' else 20. if soak else (1.6 if terminal_trial else .4), cycles=1)
            assert prepared.accepted, prepared.error_code
            started = call(ControlJointTest, 'control', group='receipt_hold/joint_test',
                action=ControlJointTest.Request.START, controller_instance_id=state.controller_instance_id,
                activation_generation=state.activation_generation, session_id=prepared.state.session_id,
                operation_id=str(uuid.uuid4()), sequence=1, permit_challenge=prepared.state.permit_challenge)
            assert started.accepted, started.error_code
            deadline = time.monotonic() + (.8 if terminal_trial else .3)
            first = len(commands)
            while time.monotonic() < deadline:
                competing.publish(Float64MultiArray(data=[.9]))
                rclpy.spin_once(node, timeout_sec=.005)
                if ((soak or (long_sweep and scenario=='completion')) and len(commands)>first) or any(abs(values[0]-.2) > .001 for values in commands[first:]):
                    break
            else:
                pytest.fail(f'Playback produced no measured command excursion: {commands[first:]}')
            state = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
            assert state.phase == 'executing', state
            if report_only:
                measured[0]=.3  # Exceeds .05 tracking bound after startup only.
            assert state.applied_sequence == 1
            assert state.permit_challenge
            if terminal_trial:
                if scenario == 'cancel':
                    cancelled = call(ControlJointTest, 'control', group='receipt_hold/joint_test',
                        action=ControlJointTest.Request.CANCEL, controller_instance_id=state.controller_instance_id,
                        activation_generation=state.activation_generation, session_id=state.session_id,
                        operation_id=str(uuid.uuid4()), sequence=2)
                    assert cancelled.accepted, cancelled.error_code
                deadline = time.monotonic() + (64. if long_sweep else 24. if soak else 3.)
                next_renewal = time.monotonic()
                sequence = 2 if scenario == 'cancel' else 1
                last_challenge = state.permit_challenge
                terminal_phase = 'cancelled' if scenario == 'cancel' else 'execution_hold'
                while state.phase != terminal_phase:
                    assert state.phase != 'sample_unavailable', 'Healthy active snapshot disappeared'
                    assert time.monotonic() < deadline, state
                    competing.publish(Float64MultiArray(data=[.9]))
                    if (scenario == 'completion' and state.permit_challenge
                            and state.applied_sequence == sequence and time.monotonic() >= next_renewal):
                        last_challenge = state.permit_challenge
                        sequence += 1
                        renewed = call(ControlJointTest, 'control', group='receipt_hold/joint_test',
                            action=ControlJointTest.Request.RENEW,
                            controller_instance_id=state.controller_instance_id,
                            activation_generation=state.activation_generation, session_id=state.session_id,
                            operation_id=str(uuid.uuid4()), sequence=sequence, permit_challenge=last_challenge)
                        if not renewed.accepted:
                            # Completion can occur between the state read and this request.
                            assert renewed.error_code == 'NOT_READY', renewed.error_code
                            state = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
                            assert state.phase == 'execution_hold' and state.reason_code == 'TEST_COMPLETED', state
                            break
                        next_renewal = time.monotonic() + .1
                    rclpy.spin_once(node, timeout_sec=.01)
                    state = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
                expected = {'completion': 'TEST_COMPLETED', 'lease_expiry': 'LEASE_EXPIRED',
                            'cancel': 'NOT_READY'}[scenario]
                assert state.reason_code == expected, state
                if scenario == 'cancel':
                    assert state.applied_sequence == 2, state
                assert state.feedback_receipt_verified and not state.feedback_fault_latched
                assert not state.permit_challenge and not state.motion_available
                if scenario == 'completion':
                    assert sequence > 2, 'Completion must require repeated renewal beyond the initial lease'
                    assert min(values[0] for values in commands[first:]) < .199
                    assert max(values[0] for values in commands[first:]) > .201
                    if soak:
                        assert sequence > 100, 'Soak must exercise sustained renewal'
                late = call(ControlJointTest, 'control', group='receipt_hold/joint_test',
                    action=ControlJointTest.Request.RENEW, controller_instance_id=state.controller_instance_id,
                    activation_generation=state.activation_generation, session_id=state.session_id,
                    operation_id=str(uuid.uuid4()), sequence=sequence+1, permit_challenge=last_challenge)
                assert not late.accepted
                observe('fresh_receipt', False)  # Drain pre-terminal command samples.
                held = commands[-1]
                first_held = len(commands)
                observe('fresh_receipt', False)
                assert len(commands[first_held:]) >= 10
                assert all(values == held for values in commands[first_held:])
                assert list(targets[-1].position) == held
                assert list(samples[-1].positions) == held
                assert any(abs(t.position[0]-.2) > .001 for t in targets)
                controllers = {c.name: c for c in call(ListControllers, 'list_controllers').controller}
                assert controllers['receipt_hold'].state == 'active'
                assert list(controllers['receipt_hold'].claimed_interfaces) == ['probe_joint/position']
                assert controllers['normal'].state == 'inactive'
                cancelled = call(ControlJointTest, 'control', group='receipt_hold/joint_test',
                    action=ControlJointTest.Request.CANCEL, controller_instance_id=state.controller_instance_id,
                    activation_generation=state.activation_generation, session_id=state.session_id,
                    operation_id=str(uuid.uuid4()), sequence=sequence+2)
                assert cancelled.accepted
                deadline = time.monotonic() + 1.
                while True:
                    state = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
                    if state.phase == 'cancelled':
                        break
                    assert time.monotonic() < deadline, state
                    rclpy.spin_once(node, timeout_sec=.01)
                if os.environ.get('JOINT_TEST_SESSION_SEQUENCE') == '1' and chirp and scenario == 'completion':
                    for trial in range(11):
                        previous_session = state.session_id
                        prep = call(PrepareJointTest, 'prepare', group='receipt_hold/joint_test',
                            controller_instance_id=state.controller_instance_id,
                            activation_generation=state.activation_generation, operation_id=str(uuid.uuid4()),
                            profile_id='synthetic_manager', profile_sha256=hashlib.sha256(canonical).hexdigest(),
                            joint_name='probe_joint', amplitude_rad=.005, transition_duration_s=.05,
                            excitation_duration_s=1.6, cycles=1)
                        assert prep.accepted, prep.error_code
                        assert prep.state.session_id > previous_session
                        seq = 1
                        def ctl(action, challenge=''):
                            return call(ControlJointTest, 'control', group='receipt_hold/joint_test',
                                action=action, controller_instance_id=prep.state.controller_instance_id,
                                activation_generation=prep.state.activation_generation,
                                session_id=prep.state.session_id, operation_id=str(uuid.uuid4()),
                                sequence=seq, permit_challenge=challenge)
                        assert ctl(ControlJointTest.Request.START, prep.state.permit_challenge).accepted
                        finish = time.monotonic()+3
                        renew_at = time.monotonic()+.1
                        while True:
                            state = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
                            assert time.monotonic()<finish, state
                            if state.phase == 'execution_hold':
                                assert state.reason_code == 'TEST_COMPLETED', state
                                break
                            assert state.phase in ('start_pending','executing','renewal_pending'), state
                            if state.permit_challenge and state.applied_sequence == seq and time.monotonic()>=renew_at:
                                seq+=1
                                renewed = ctl(ControlJointTest.Request.RENEW,state.permit_challenge)
                                if not renewed.accepted:
                                    assert renewed.error_code == 'NOT_READY', renewed.error_code
                                    terminal = call(GetJointTestState,'state',group='receipt_hold/joint_test').state
                                    assert terminal.session_id == prep.state.session_id
                                    assert terminal.phase == 'execution_hold', terminal
                                    assert terminal.reason_code == 'TEST_COMPLETED', terminal
                                    break
                                renew_at=time.monotonic()+.1
                            rclpy.spin_once(node,timeout_sec=.01)
                        seq+=1
                        assert ctl(ControlJointTest.Request.CANCEL).accepted
                        while state.phase != 'cancelled':
                            assert time.monotonic()<finish
                            state=call(GetJointTestState,'state',group='receipt_hold/joint_test').state
                            rclpy.spin_once(node,timeout_sec=.01)
                assert call(SwitchController, 'switch_controller', deactivate_controllers=['receipt_hold'],
                    strictness=SwitchController.Request.STRICT, timeout=request.timeout).ok
                # Drain any publication already queued before deactivation.
                deadline = time.monotonic() + .1
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.01)
                count = len(targets)
                sample_count = len(samples)
                deadline = time.monotonic() + .15
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.01)
                assert len(targets) == count, 'Inactive controller continued target telemetry'
                assert len(samples) == sample_count
                assert all(a.update_sequence+1 == b.update_sequence
                           for a,b in zip(samples,samples[1:])), 'Target telemetry lost a controller update'
                assert all(s.get('target_queue_overflow',0) == 0 and
                           s.get('target_publish_errors',0) == 0 for s in statuses)
                inactive = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
                assert inactive.controller_instance_id == state.controller_instance_id
                assert inactive.activation_generation == state.activation_generation
                assert inactive.phase == 'sample_unavailable'
                assert not inactive.motion_available and not inactive.hold_applied
                if chirp and scenario == 'completion':
                    assert call(SwitchController, 'switch_controller', activate_controllers=['receipt_hold'],
                        strictness=SwitchController.Request.STRICT, timeout=request.timeout).ok
                    deadline = time.monotonic()+2
                    while True:
                        repeated = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
                        if repeated.hold_applied:
                            break
                        assert time.monotonic() < deadline
                        rclpy.spin_once(node, timeout_sec=.01)
                    assert repeated.controller_instance_id == inactive.controller_instance_id
                    assert repeated.activation_generation == inactive.activation_generation+1
                    assert call(SwitchController, 'switch_controller', deactivate_controllers=['receipt_hold'],
                        strictness=SwitchController.Request.STRICT, timeout=request.timeout).ok
                    again = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
                    assert again.activation_generation == repeated.activation_generation
                    assert not again.motion_available and not again.hold_applied
                return
            renewed = call(ControlJointTest, 'control', group='receipt_hold/joint_test',
                action=ControlJointTest.Request.RENEW, controller_instance_id=state.controller_instance_id,
                activation_generation=state.activation_generation, session_id=state.session_id,
                operation_id=str(uuid.uuid4()), sequence=2, permit_challenge=state.permit_challenge)
            assert renewed.accepted, renewed.error_code
            deadline = time.monotonic() + .1
            while True:
                state = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
                if state.applied_sequence == 2:
                    break
                assert time.monotonic() < deadline, state
                rclpy.spin_once(node, timeout_sec=.005)
        sending[0] = False
        first_command = len(commands)
        observe('stale_or_invalid_receipt', True)
        controllers = {c.name: c for c in call(ListControllers, 'list_controllers').controller}
        controller = controllers['receipt_hold']
        assert controller.state == 'active'
        assert list(controller.claimed_interfaces) == ['probe_joint/position']
        assert len(commands[first_command:]) >= 10
        held = commands[-1]
        if playback:
            assert controllers['normal'].state == 'inactive'
            assert not controllers['normal'].claimed_interfaces
            assert all(abs(values[0]-.2) <= .0051 for values in commands[first_command:])
            state = call(GetJointTestState, 'state', group='receipt_hold/joint_test').state
            assert state.phase == 'execution_hold'
            assert state.reason_code == 'READINESS_LOST', state
        else:
            assert all(values == [0.2] for values in commands[first_command:])
        first_command = len(commands)
        sending[0] = True
        observe('fresh_receipt', True)  # Recovery is not permission to resume.
        assert len(commands[first_command:]) >= 10
        assert all(values == held for values in commands[first_command:])
        assert call(SwitchController, 'switch_controller', deactivate_controllers=['receipt_hold'],
            strictness=SwitchController.Request.STRICT, timeout=request.timeout).ok
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
