#! /usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState, GetLinkState
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Header, Bool
from std_srvs.srv import Empty

from visualization_msgs.msg import Marker
import PyKDL
import copy
# See https://docs.python.org/3/library/threading.html#event-objects
from threading import Event
import time
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

FIXED_TARGET_MODE = 'fixed'


class PandaRosBridge:

    def __init__(self, real_robot=False):

        # Event is claer while initialization or set_state is going on
        self.reset = Event()
        self._unlock_reset_event()
        self.get_state_event = Event()
        self._lock_state_event()

        self.real_robot = real_robot

        # TODO publisher, subscriber, target and state
        self.arm_cmd_pub = rospy.Publisher(
            'env_arm_command', JointTrajectory, queue_size=1)

        self.target = [0.0] * 1  # TODO define number of target floats
        # TODO define number of panda states (At least the number of joints)
        self.panda_joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                                  'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        self.panda_finger_names = ['panda_finger_joint1', 'panda_finger_joint2']
        self.panda_joint_num = len(self.panda_joint_names)
        self.panda_state = [0.0] * self.panda_joint_num

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Robot control rate
        self.sleep_time = (1.0 / rospy.get_param('~action_cycle_rate')) - 0.01
        self.control_period = rospy.Duration.from_sec(self.sleep_time)

        self.reference_frame = rospy.get_param('~reference_frame', 'base')
        self.ee_frame = 'tool0'  # TODO is the value for self.ee_frame correct?
        self.target_frame = 'target'

        # Minimum Trajectory Point time from start
        # TODO check if this value is correct for the panda robot
        self.min_traj_duration = 0.5

        if not self.real_robot:
            # Subscribers to link collision sensors topics

            # TODO add rospy.Subsribers
            # TODO add keys to collision sensors
            self.collision_sensors = dict.fromkeys([], False)
            pass

        # TODO currently not used
        self.safe_to_move = True

        # Target mode
        self.target_mode = rospy.get_param('~target_mode', FIXED_TARGET_MODE)
        self.target_mode_name = rospy.get_param('~target_model_name', 'box100')

        # Object parameters
        self.objects_controller = rospy.get_param('objects_controller', False)
        self.n_objects = int(rospy.get_param('n_objects', 0))

        if self.objects_controller:
            self.objects_model_name = []
            for i in range(self.n_objects):
                obj_model_name = rospy.get_param(
                    'object_' + repr(i) + '_model_name')
                self.objects_model_name.append(obj_model_name)

    def get_state(self):
        self._unlock_state_event()

        # Get environment state
        state = []  # TODO currently not used in function

        # currently only working on a fixed target mode
        if self.target_mode == FIXED_TARGET_MODE:
            target = copy.deepcopy(self.target)
        else:
            raise ValueError
            # raise ValueError as err(
            #     'Target mode was ill defined. Got error type: ' +
            #     str(type(err)) + ' with message: ' + err.message)

        panda_state = copy.deepcopy(self.panda_state)

        # TODO is ee_to_base_transform value correctly loaded and set
        (position, quaternion) = self.tf_listener.lookupTransform(
            self.reference_frame)
        ee_to_base_transform = position + quaternion

        if self.real_robot:
            panda_collision = False
        else:
            panda_collision = any(self.collision_sensors.values())

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State()
        msg.state.extend(target)
        msg.state.extend(panda_state)
        msg.state.extend(ee_to_base_transform)
        msg.state.extend([panda_collision])
        msg.success = True

        return msg

    def set_state(self, state_msg):
        # Set environment state
        state = state_msg.state
        self._unlock_reset_event()
        # Set target internal value
        if self.target_mode == FIXED_TARGET_MODE:
            # TODO found out how many state values are needed for panda
            self.target = copy.deepcopy(state[0:6])
            # Publish Target Marker
            self.publish_target_marker(self.target)

        # TODO setup objects movement
        # if self.objects_controller:

        # TODO reset_steps and init of corresponding variables

        self._lock_reset_event()

        return True

    def publish_env_arm_cmd(self, position_cmd):
        """Publish environment JointTrajectory msg.

        Publish JointTrajectory message to the env_command topic.

        Args:
            position_cmd (type): Description of parameter `positions`.

        Returns:
            type: Description of returned object.

        """
        if self.safe_to_move:
            msg = JointTrajectory()
            msg.header = Header()
            msg.joint_names = copy.deepcopy(self.panda_joint_names)
            msg.points = [JointTrajectoryPoint()]
            msg.points[0].positions = position_cmd
            duration = []
            for i, joint_name in enumerate(msg.joint_names):
                # TODO check if the index is in bounds
                # !!! Be careful with panda_state index here
                pos = self.panda_state[i]
                cmd = position_cmd[i]
                max_vel = self.panda_joint_vel_limits[i]
                temp_duration = max(abs(cmd - pos) / max_vel,
                                    self.min_traj_duration)
                duration.append(temp_duration)

            msg.points[0].time_from_start = rospy.Duration.from_sec(
                max(duration))

            rospy.sleep(self.control_period)
            return position_cmd

        else:
            rospy.sleep(self.control_period)
            return [0.0] * self.panda_joint_num

    def get_link_state(self, link_name, reference_frame=''):
        """Method is used to retrieve link state from gazebo simulation

        Args:
            link_name (name of the link): [description]
            reference_frame (str, optional): [description]. Defaults to ''.

        Returns:
            state of the link corresponding to the given name ->
                [x_pos, y_pos, z_pos, roll, pitch, yaw, x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel] 
        """
        gazebo_get_link_state_service = '/gazebo/get_link_state'
        rospy.wait_for_service(gazebo_get_link_state_service)
        try:
            link_state_srv = rospy.ServiceProxy(
                gazebo_get_link_state_service, GetLinkState)
            link_coordinates = link_state_srv(
                link_name, reference_frame).link_state
            link_pose, link_twist = link_coordinates.pose, link_coordinates.twist
            x_pos = link_pose.position.x
            y_pos = link_pose.position.y
            z_pos = link_pose.position.z

            orientation = PyKDL.Rotation.Quaternion(link_pose.orientation.x,
                                                    link_pose.orientation.y,
                                                    link_pose.orientation.z,
                                                    link_pose.orientation.w)

            euler_orientation = orientation.GetRPY()
            roll = euler_orientation[0]
            pitch = euler_orientation[1]
            yaw = euler_orientation[2]

            x_vel = link_twist.linear.x
            y_vel = link_twist.linear.y
            z_vel = link_twist.linear.z
            roll_vel = link_twist.angular.x
            pitch_vel = link_twist.angular.y
            yaw_vel = link_twist.angular.z

            return x_pos, y_pos, z_pos, roll, pitch, yaw, x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel
        except rospy.ServiceException as err:
            error_message = 'Service call failed: ' + err
            rospy.logerr(error_message)
            print(error_message)

    def publish_target_marker(self, target_pose):
        t_marker = Marker()
        # TODO add values to marker
        self.target_pub.publish(t_marker)

    def callback_panda(self, data):
        # TODO split and endpoint yet to be defined
        if self._is_state_event_locked():
            split_point = 6
            end_point = 12
            self.panda_state[0: split_point] = data.position[0: split_point]
            self.panda_state[split_point: end_point] = data.velocity[0: split_point]
        pass

    def _is_state_event_locked(self):
        return self.get_state_event.is_set()

    def _lock_reset_event(self):
        self.reset.set()

    def _lock_state_event(self):
        self.get_state_event.set()

    def _unlock_reset_event(self):
        self.reset.clear()

    def _unlock_state_event(self):
        self.get_state_event.clear()
