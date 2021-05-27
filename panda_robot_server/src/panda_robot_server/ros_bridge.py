#! /usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState, GetLinkState
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Header, Bool
from std_srvs.srv import Empty
from franka_interface import ArmInterface

from visualization_msgs.msg import Marker
import PyKDL
import copy
# See https://docs.python.org/3/library/threading.html#event-objects
from threading import Event
import time
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

class PandaRosBridge:

    def __init__(self, real_robot=False):
        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        self.real_robot = real_robot

        # Joint States
        self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                                  'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        self.finger_names = ['panda_finger_joint1', 'panda_finger_joint2']
        self.joint_position = dict.fromkeys(self.joint_names, 0.0)
        self.joint_velocity = dict.fromkeys(self.joint_names, 0.0)
        self.joint_effort = dict.fromkeys(self.joint_names, 0.0)
        rospy.Subscriber('/joint_states', JointState, self._on_joint_states)

        self.panda_joint_num = len(self.joint_names)

        # TODO publisher, subscriber, target and state
        self.arm = ArmInterface()

        self.target = [0.0] * 6  # TODO define number of target floats
        # TODO define number of panda states (At least the number of joints)
        
        # Robot control
        self.arm_cmd_pub = rospy.Publisher('env_arm_command', JointTrajectory, queue_size=1) # joint_trajectory_command_handler publisher
        self.sleep_time = (1.0 / rospy.get_param('~action_cycle_rate')) - 0.01
        self.control_period = rospy.Duration.from_sec(self.sleep_time)
        self.max_velocity_scale_factor = float(rospy.get_param("~max_velocity_scale_factor"))
        self.min_traj_duration = 0.5 # minimum trajectory duration (s)
        self.joint_velocity_limits = self._get_joint_velocity_limits()
        
        # Robot frames
        self.reference_frame = rospy.get_param('~reference_frame', 'base')
        self.ee_frame = 'panda_hand'  

        # TF2
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.static_tf2_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Collision detection 
        # if not self.real_robot:
        #     rospy.Subscriber("panda_link0_collision", ContactsState, self._on_link0_collision)
        #     rospy.Subscriber("panda_link1_collision", ContactsState, self._on_link1_collision)
        #     rospy.Subscriber("panda_link2_collision", ContactsState, self._on_link2_collision)
        #     rospy.Subscriber("panda_link3_collision", ContactsState, self._on_link3_collision)
        #     rospy.Subscriber("panda_link4_collision", ContactsState, self._on_link4_collision)
        #     rospy.Subscriber("panda_link5_collision", ContactsState, self._on_link5_collision)
        #     rospy.Subscriber("panda_link6_collision", ContactsState, self._on_link6_collision)
        #     rospy.Subscriber("panda_link7_collision", ContactsState, self._on_link7_collision)
        #     rospy.Subscriber("panda_leftfinger_collision", ContactsState, self._on_leftfinger_collision)
        #     rospy.Subscriber("panda_rightfinger_collision", ContactsState, self._on_rightfinger_collision)
        # Initialization of collision sensor flags
        self.collision_sensors = dict.fromkeys(['panda_link0', 'panda_link1', 'panda_link2', 'panda_link3', 'panda_link4', \
                                                'panda_link5','panda_link6','panda_link7','panda_leftfinger','panda_rightfinger',], False)

        # TODO currently not used
        self.safe_to_move = True

        # Robot Server mode
        self.rs_mode = rospy.get_param('~rs_mode')

        # # Objects  Controller 
        # self.objects_controller = rospy.get_param("objects_controller", False)
        # self.n_objects = int(rospy.get_param("n_objects"))
        # if self.objects_controller:
        #     self.move_objects_pub = rospy.Publisher('move_objects', Bool, queue_size=10)
        #     # Get objects model name
        #     self.objects_model_name = []
        #     for i in range(self.n_objects):
        #         self.objects_model_name.append(rospy.get_param("object_" + repr(i) + "_model_name"))
        #     # Get objects TF Frame
        #     self.objects_frame = []
        #     for i in range(self.n_objects):
        #         self.objects_frame.append(rospy.get_param("object_" + repr(i) + "_frame"))
                
    def _on_joint_states(self, msg):
        """Callback function which sets the panda state
            - `msg.name`     -> joint names
            - `msg.position` -> current joint positions
            - `msg.velocity` -> current velocity of joints
            - `msg.effort`   -> current torque (effort) of joints

        Args:
            `msg` (JointStates): msg containing the current joint states
                
        """
        # TODO gripper might also be included in this state
        if self.get_state_event.is_set():
            for idx, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.joint_position[name] = msg.position[idx]
                    self.joint_velocity[name] = msg.velocity[idx]
                    self.joint_effort[name] = msg.effort[idx]
            
    def get_state(self):
        self.get_state_event.clear()
        # Get environment state
        state =[]
        state_dict = {}

        if self.rs_mode == 'only_robot':
            # Joint Positions and Joint Velocities
            joint_position = copy.deepcopy(self.joint_position)
            joint_velocity = copy.deepcopy(self.joint_velocity)
            joint_effort = copy.deepcopy(self.joint_effort)
            state += self._get_joint_ordered_value_list(joint_position)
            state += self._get_joint_ordered_value_list(joint_velocity)
            state += self._get_joint_ordered_value_list(joint_effort)
            state_dict.update(self._get_joint_states_dict(joint_position, joint_velocity, joint_effort))

            # ee to ref transform
            ee_to_ref_trans = self.tf2_buffer.lookup_transform(self.reference_frame, self.ee_frame, rospy.Time(0))
            ee_to_ref_trans_list = self._transform_to_list(ee_to_ref_trans)
            state += ee_to_ref_trans_list
            state_dict.update(self._get_transform_dict(ee_to_ref_trans, 'ee_to_ref'))
        
            # Collision sensors
            ur_collision = any(self.collision_sensors.values())
            state += [ur_collision]
            state_dict['in_collision'] = float(ur_collision)

        else: 
            raise ValueError

        self.get_state_event.set()
        
        # Create and fill State message
        msg = robot_server_pb2.State(state=state, state_dict=state_dict, success= True)

        return msg

    def set_state(self, state_msg):
        # Set environment state
        state = state_msg.state
        self.reset.clear()

        # Set target internal value
        if self.target_mode == FIXED_TARGET_MODE:
            # TODO found out how many state values are needed for panda
            self.target = copy.deepcopy(state[0:6])
            # Publish Target Marker
            self.publish_target_marker(self.target)
            # TODO Broadcast target tf2
        
        
        # TODO setup objects movement
        # if self.objects_controller:

        transformed_j_pos = self._transform_panda_list_to_dict(state[6:13])
        reset_steps = int(15.0 / self.sleep_time)

        for _ in range(reset_steps):
            self.arm.set_joint_positions(transformed_j_pos)

        self.reset.set()
        rospy.sleep(self.control_period)

        return True

    def publish_env_arm_cmd(self, position_cmd):
        """Publish environment JointTrajectory msg.

        Publish JointTrajectory message to the env_command topic.

        Args:
            position_cmd (type): Description of parameter `positions`.

        Returns:
            type: Description of returned object.

        """
        # if self.safe_to_move:
        #     msg = JointTrajectory()
        #     msg.header = Header()
        #     msg.joint_names = copy.deepcopy(self.joint_names)
        #     msg.points = [JointTrajectoryPoint()]
        #     msg.points[0].positions = position_cmd
        #     duration = []
            # for i in range(len(msg.joint_names)):
                # TODO check if the index is in bounds
                # !!! Be careful with panda_state index here
                # pos = self.panda_state[i]
                # cmd = position_cmd[i]
                # max_vel = self.panda_joint_vel_limits[i]
                # temp_duration = max(abs(cmd - pos) / max_vel, self.min_traj_duration)
                # duration.append(temp_duration)

            # msg.points[0].time_from_start = rospy.Duration.from_sec(max(duration))
            # print(msg)
            
            # self.arm_cmd_pub.publish(msg)
        if self.safe_to_move:
            transformed_j_pos = self._transform_panda_list_to_dict(position_cmd[0:7])
            self.arm.set_joint_positions(transformed_j_pos)
            rospy.sleep(self.control_period)
            return position_cmd
        else:
            rospy.sleep(self.control_period)
            return [0.0] * self.panda_joint_num
        
    def get_model_state_pose(self, model_name, relative_entity_name=''):
        # method used to retrieve model pose from gazebo simulation

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            s = model_state(model_name, relative_entity_name)

            pose = [s.pose.position.x, s.pose.position.y, s.pose.position.z, \
                    s.pose.orientation.x, s.pose.orientation.y, s.pose.orientation.z, s.pose.orientation.w]

            return pose
        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def _transform_panda_list_to_dict(self, panda_list):
        transformed_dict = {}
        for idx, value in enumerate(panda_list):
            current_joint_name = self.joint_names[idx]
            transformed_dict[current_joint_name] = value
        return transformed_dict

    def _get_joint_states_dict(self, joint_position, joint_velocity, joint_effort):

        d = {}
        d['joint1_position'] = joint_position['panda_joint1']
        d['joint2_position'] = joint_position['panda_joint2']
        d['joint3_position'] = joint_position['panda_joint3']
        d['joint4_position'] = joint_position['panda_joint4']
        d['joint5_position'] = joint_position['panda_joint5']
        d['joint6_position'] = joint_position['panda_joint6']
        d['joint7_position'] = joint_position['panda_joint7']
        d['joint1_velocity'] = joint_velocity['panda_joint1']
        d['joint2_velocity'] = joint_velocity['panda_joint2']
        d['joint3_velocity'] = joint_velocity['panda_joint3']
        d['joint4_velocity'] = joint_velocity['panda_joint4']
        d['joint5_velocity'] = joint_velocity['panda_joint5']
        d['joint6_velocity'] = joint_velocity['panda_joint6']
        d['joint7_velocity'] = joint_velocity['panda_joint7']
        d['joint1_effort'] = joint_effort['panda_joint1']
        d['joint2_effort'] = joint_effort['panda_joint2']
        d['joint3_effort'] = joint_effort['panda_joint3']
        d['joint4_effort'] = joint_effort['panda_joint4']
        d['joint5_effort'] = joint_effort['panda_joint5']
        d['joint6_effort'] = joint_effort['panda_joint6']
        d['joint7_effort'] = joint_effort['panda_joint7']
        
        return d 

    def _get_transform_dict(self, transform, transform_name):

        d ={}
        d[transform_name + '_translation_x'] = transform.transform.translation.x
        d[transform_name + '_translation_y'] = transform.transform.translation.y
        d[transform_name + '_translation_z'] = transform.transform.translation.z
        d[transform_name + '_rotation_x'] = transform.transform.rotation.x
        d[transform_name + '_rotation_y'] = transform.transform.rotation.y
        d[transform_name + '_rotation_z'] = transform.transform.rotation.z
        d[transform_name + '_rotation_w'] = transform.transform.rotation.w

        return d

    def _transform_to_list(self, transform):

        return [transform.transform.translation.x, transform.transform.translation.y, \
                transform.transform.translation.z, transform.transform.rotation.x, \
                transform.transform.rotation.y, transform.transform.rotation.z, \
                transform.transform.rotation.w]


    def _get_joint_ordered_value_list(self, joint_values):
        
        return [joint_values[name] for name in self.joint_names]

    def _get_joint_velocity_limits(self):

        absolute_joint_velocity_limits = {'panda_joint1': 2.1750, 'panda_joint2': 2.1750, 'panda_joint3': 2.1750, 'panda_joint1': 2.1750, \
                                          'panda_joint5': 2.6100, 'panda_joint6': 2.6100, 'panda_joint7': 2.6100,}
    

        return {name: self.max_velocity_scale_factor * absolute_joint_velocity_limits[name] for name in self.joint_names}