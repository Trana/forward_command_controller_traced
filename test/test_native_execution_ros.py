"""Real ROS/controller-manager integration with synthetic feedback. No CAN or Isaac."""
import os
import subprocess
import time
from pathlib import Path
import pytest
import yaml
pytest.importorskip('rclpy')
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import LoadController, ConfigureController, SwitchController
from runtime_interfaces.srv import NativeExecution
from runtime_interfaces.msg import NativeExecutionCommand, NativeExecutionState
from ament_index_python.packages import get_package_prefix


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '229' or os.environ.get('ROS_LOCALHOST_ONLY') != '1', reason='isolated localhost domain 229 required')
def test_native_controller_cadence_cancel_fencing_and_frozen_feedback(tmp_path):
    names = [f'joint_{i}' for i in range(12)]
    ns = f'/native_test_{os.getpid()}'
    config = tmp_path/'controllers.yaml'
    config.write_text(yaml.safe_dump({
        '/**/controller_manager': {'ros__parameters': {'update_rate':200, 'output': {'type':'forward_command_controller_traced/SupervisedCommandController'}}},
        '/**/output': {'ros__parameters': {'joints':names,'interface_name':'position',
            'diagnostic_lower':[-2.]*12,'diagnostic_upper':[2.]*12,'diagnostic_amplitude':[.4]*12,
            'feedback_receipt_interface':'feedback_receipt_monotonic_s'}}}))
    log = (tmp_path/'manager.log').open('w')
    process = subprocess.Popen([str(Path(get_package_prefix('controller_manager'))/'lib/controller_manager/ros2_control_node'),
        '--ros-args','-r',f'__ns:={ns}','-r',f'robot_description:={ns}/robot_description',
        '-r',f'~/robot_description:={ns}/robot_description','--params-file',str(config)],stdout=log,stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('probe',namespace=ns)
    try:
        statuses=[];outputs=[];measured=[0.]*12;sending=[True];permission=[False];gen=[1];seq=[0]
        tracking_offset=[0.]
        qos=QoSProfile(depth=128,reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(NativeExecutionState,ns+'/output/execution_state',statuses.append,qos)
        node.create_subscription(JointState,ns+'/commands',lambda m:outputs.append(m),qos)
        pub=node.create_publisher(JointState,ns+'/feedback',qos)
        command_pub=node.create_publisher(NativeExecutionCommand,ns+'/output/execution_commands',qos)
        def feed():
            if outputs:
                measured[:]=outputs[-1].position
                measured[3]+=tracking_offset[0]
            if sending[0]:pub.publish(JointState(name=names,position=measured))
            if permission[0] and statuses:
                seq[0]+=1
                command_pub.publish(NativeExecutionCommand(epoch=statuses[-1].epoch,generation=gen[0],sequence=seq[0],targets=[1.]*12))
        node.create_timer(.02,feed)
        desc=node.create_publisher(String,ns+'/robot_description',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        links='<link name="base"/>'+''.join(f'<link name="link_{i}"/><joint name="{name}" type="revolute"><parent link="base"/><child link="link_{i}"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="20"/></joint>' for i,name in enumerate(names))
        joints=''.join(f'<joint name="{n}"><command_interface name="position"/><state_interface name="position"><param name="initial_value">0</param></state_interface><state_interface name="feedback_receipt_monotonic_s"/></joint>' for n in names)
        desc.publish(String(data=f'<robot name="native_probe">{links}<ros2_control name="Probe" type="system"><hardware><plugin>topic_based_ros2_control/TopicBasedSystem</plugin><param name="export_feedback_receipt_timing">true</param><param name="joint_states_topic">{ns}/feedback</param><param name="joint_commands_topic">{ns}/commands</param><param name="trigger_joint_command_threshold">-1</param></hardware>{joints}</ros2_control></robot>'))
        clients={}
        def call(kind,endpoint,**values):
            if endpoint not in clients:clients[endpoint]=node.create_client(kind,ns+'/'+endpoint)
            client=clients[endpoint];assert client.wait_for_service(timeout_sec=10)
            future=client.call_async(kind.Request(**values))
            rclpy.spin_until_future_complete(node,future,timeout_sec=5)
            assert future.done(),endpoint
            return future.result()
        def wait(check,timeout=3):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.005)
                if check():return
            pytest.fail(f'Condition timeout; latest state={statuses[-1] if statuses else None}; log={config.parent}')
        assert call(LoadController,'controller_manager/load_controller',name='output').ok
        assert call(ConfigureController,'controller_manager/configure_controller',name='output').ok
        assert call(SwitchController,'controller_manager/switch_controller',activate_controllers=['output'],strictness=2).ok
        wait(lambda:bool(statuses))
        epoch=statuses[-1].epoch
        assert epoch
        def control(action,generation,**kwargs):
            return call(NativeExecution,'output/execution',action=action,epoch=epoch,generation=generation,operation=generation,**kwargs)
        assert control(3,1).accepted
        wait(lambda:statuses[-1].generation==1 and statuses[-1].applied)
        assert control(2,2,baseline=[0.]*12,amplitude=[.02]*12,duration_s=1.,start_hz=.1,end_hz=2.).accepted
        assert not control(2,2,baseline=[0.]*12,amplitude=[.03]*12,duration_s=1.,start_hz=.1,end_hz=2.).accepted
        gen[0]=2;permission[0]=True
        wait(lambda:statuses[-1].generation==2 and statuses[-1].phase==1)
        tracking_offset[0]=.35
        wait(lambda:statuses[-1].generation==2 and statuses[-1].phase==2,2.)
        assert statuses[-1].fault==0
        assert 'CHIRP_TRACKING_REPORT_ONLY' in (tmp_path/'manager.log').read_text()
        assert 'joint=joint_3' in (tmp_path/'manager.log').read_text()
        tracking_offset[0]=0.
        assert 45<=statuses[-1].target_sequence<=50
        assert max(abs(v) for v in statuses[-1].targets)<1e-12
        # Permission messages contain nonsense targets. They must not overwrite the chirp.
        assert max(abs(v) for s in statuses if s.generation==2 for v in s.targets)<.021
        assert control(3,3).accepted
        wait(lambda:statuses[-1].generation==3 and statuses[-1].applied)
        assert all(s.mode != 2 or s.phase != 0 for s in statuses), 'Released chirp advertised as a waveform sample'
        for generation in (4,6):
            gen[0]=generation;seq[0]=0
            assert control(2,generation,baseline=list(statuses[-1].targets),amplitude=[.01]*12,duration_s=2.,start_hz=.1,end_hz=2.).accepted
            wait(lambda:statuses[-1].generation==generation and statuses[-1].phase==1)
            sending[0]=False
            wait(lambda:statuses[-1].phase==4)
            assert statuses[-1].fault==3
            held=list(statuses[-1].targets)
            sending[0]=True
            before=statuses[-1].revision
            wait(lambda:statuses[-1].revision>before+5)
            assert statuses[-1].phase==4 and list(statuses[-1].targets)==held
            assert control(3,generation+1).accepted
            wait(lambda:statuses[-1].generation==generation+1 and statuses[-1].applied)
        # A delayed old start cannot retake output after hold generation 7.
        assert not control(2,6,baseline=[1.]*12,amplitude=[.01]*12,duration_s=2.,start_hz=.1,end_hz=2.).accepted
        before=statuses[-1].revision
        wait(lambda:statuses[-1].revision>before+3)
        assert statuses[-1].generation==7 and statuses[-1].mode==3
        permission[0]=False
        assert control(1,8).accepted
        wait(lambda:statuses[-1].generation==8)
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=8,sequence=100,targets=[.005]*12))
        wait(lambda:list(statuses[-1].targets)==[.005]*12)
        assert statuses[-1].fault==0
        # Old generations cannot replace a new streamed owner either.
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=7,sequence=1000,targets=[.02]*12))
        before=statuses[-1].revision
        wait(lambda:statuses[-1].revision>before+2)
        assert list(statuses[-1].targets)==[.005]*12
        # The widened cap accepts a 0.34 rad step after budget replenishment.
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=8,sequence=101,targets=[.345]*12))
        wait(lambda:list(statuses[-1].targets)==[.345]*12)
        assert statuses[-1].fault==0
        # STREAM/IK accepts large target reversals, retaining absolute bounds.
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=8,sequence=102,targets=[1.5]*12))
        wait(lambda:list(statuses[-1].targets)==[1.5]*12)
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=8,sequence=103,targets=[-1.5]*12))
        wait(lambda:list(statuses[-1].targets)==[-1.5]*12)
        assert statuses[-1].fault==0
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=8,sequence=104,targets=[2.1]*12))
        wait(lambda:statuses[-1].fault==7)
        assert 'reason=absolute joint limit' in statuses[-1].fault_detail
        assert control(3,9).accepted
        wait(lambda:statuses[-1].generation==9 and statuses[-1].applied)
        # Policy PD targets do not inherit the STREAM target-delta constraint.
        assert control(NativeExecution.Request.POLICY_TARGET,10).accepted
        wait(lambda:statuses[-1].generation==10)
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=10,sequence=1,targets=[1.5]*12))
        wait(lambda:list(statuses[-1].targets)==[1.5]*12)
        assert statuses[-1].fault==0
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=10,sequence=2,targets=[-1.5]*12))
        wait(lambda:list(statuses[-1].targets)==[-1.5]*12)
        assert statuses[-1].fault==0
        # Absolute bounds still reject, retaining the last applied position.
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=10,sequence=3,targets=[2.1]*12))
        wait(lambda:statuses[-1].fault!=0)
        assert list(statuses[-1].targets)==[-1.5]*12
        assert 'reason=absolute joint limit' in statuses[-1].fault_detail
        assert 'target_rad=2.100000' in statuses[-1].fault_detail
        assert 'lower_rad=-2.000000 upper_rad=2.000000' in statuses[-1].fault_detail
        assert control(3,11).accepted
        wait(lambda:statuses[-1].generation==11 and statuses[-1].applied)
        assert statuses[-1].fault_detail == ''
        assert control(NativeExecution.Request.POLICY_TARGET,12).accepted
        wait(lambda:statuses[-1].generation==12)
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=12,sequence=1,targets=[0.]*12))
        wait(lambda:list(statuses[-1].targets)==[0.]*12)
        wait(lambda:statuses[-1].fault==6)  # Missing fresh commands still expires.
        assert control(3,13).accepted
        wait(lambda:statuses[-1].generation==13 and statuses[-1].applied)
        command_pub.publish(NativeExecutionCommand(epoch=epoch,generation=12,sequence=2,targets=[1.5]*12))
        before=statuses[-1].revision
        wait(lambda:statuses[-1].revision>before+2)
        assert statuses[-1].mode==3 and list(statuses[-1].targets)==[0.]*12
    finally:
        node.destroy_node();rclpy.shutdown();process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        log.close()
