#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState, GetLinkState
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Float64MultiArray, Header
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import PyKDL
import copy
from threading import Event
import time


class UrRosBridge:

    def __init__(self,  real_robot=False):

        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        self.real_robot = real_robot

        # joint_trajectory_command_handler publisher
        self.arm_cmd_pub = rospy.Publisher('env_arm_command', JointTrajectory, queue_size=1)

        # Target RViz Marker publisher
        self.target_pub = rospy.Publisher('target_marker', Marker, queue_size=10)

        self.target = [0.0] * 6
        self.ur_state = [0.0] *12

        if self.real_robot:
            rospy.Subscriber("joint_states", JointState, self.callbackUR, queue_size=1, buff_size=96, tcp_nodelay=True )
        else:
            rospy.Subscriber("estimated_joint_states", JointState, self.callbackUR, queue_size=1, buff_size=96, tcp_nodelay=True )

        # TF Listener
        self.tf_listener = tf.TransformListener()

        self.control_period = rospy.Duration.from_sec(0.038)

        self.reference_frame = 'base'

        if self.real_robot:
            self.ee_frame = 'ee_link'
        else: 
            self.ee_frame = 'flange'


        self.max_velocity_scale_factor = float(rospy.get_param("~max_velocity_scale_factor"))
        self.absolute_ur_joint_vel_limits = [3.15, 2.16, 2.16, 3.2, 3.2, 3.2]
        self.ur_joint_vel_limits = [self.max_velocity_scale_factor * i for i in self.absolute_ur_joint_vel_limits]
        # Minimum Trajectory Point time from start
        self.min_traj_duration = 0.5

        if not self.real_robot:
            # Subscribers to link collision sensors topics

            rospy.Subscriber("shoulder_collision", ContactsState, self.shoulder_collision_callback)
            rospy.Subscriber("upper_arm_collision", ContactsState, self.upper_arm_collision_callback)
            rospy.Subscriber("forearm_collision", ContactsState, self.forearm_collision_callback)
            rospy.Subscriber("wrist_1_collision", ContactsState, self.wrist_1_collision_callback)
            rospy.Subscriber("wrist_2_collision", ContactsState, self.wrist_2_collision_callback)
            rospy.Subscriber("wrist_3_collision", ContactsState, self.wrist_3_collision_callback)

            # Initialization of collision sensor flags
            self.collision_sensors = dict.fromkeys(["shoulder","upper_arm","forearm","wrist_1","wrist_2","wrist_3"], False)

        #TODO
        self.safe_to_move = True

    def get_state(self):
        self.get_state_event.clear()
        # Get environment state
        state =[]
        target = copy.deepcopy(self.target)
        ur_state = copy.deepcopy(self.ur_state)

        (position, quaternion) = self.tf_listener.lookupTransform(self.reference_frame,self.ee_frame,rospy.Time(0))

        euler = tf.transformations.euler_from_quaternion(quaternion)
        ee_pose = position + [euler[0],euler[1],euler[2]]

        if self.real_robot:
            ur_collision = False
        else:
            ur_collision = any(self.collision_sensors.values())

        self.get_state_event.set()

        state.extend(target)
        state.extend(ur_state)
        state.extend(ee_pose)
        state.extend([ur_collision])

        return state

    def set_state(self, state):
        # Set environment state
        # Clear reset Event
        self.reset.clear()
        # Set target internal value
        self.target = copy.deepcopy(state[0:6])
        # Publish Target Marker
        self.publish_target_marker(self.target)
        # UR Joints Positions
        for i in range(350):
            self.publish_env_arm_cmd(state[6:12])
        if not self.real_robot:
            # Reset collision sensors flags
            self.collision_sensors.update(dict.fromkeys(["shoulder","upper_arm","forearm","wrist_1","wrist_2","wrist_3"], False))

        self.reset.set()

        return 1

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
            msg.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", \
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            msg.points=[JointTrajectoryPoint()]
            msg.points[0].positions = position_cmd
            dur = []
            for i in range(len(msg.joint_names)):
                # !!! Be careful here with ur_state index
                pos = self.ur_state[i]
                cmd = position_cmd[i]
                max_vel = self.ur_joint_vel_limits[i]
                dur.append(max(abs(cmd-pos)/max_vel,self.min_traj_duration))

            msg.points[0].time_from_start = rospy.Duration.from_sec(max(dur))
            self.arm_cmd_pub.publish(msg)
            # Sleep time set manually to achieve approximately 25Hz rate
            rospy.sleep(self.control_period)
            return position_cmd
        else:
            rospy.sleep(self.control_period)
            return [0.0]*6

    def get_model_state(self, model_name, relative_entity_name=''):
        # method used to retrieve model state from gazebo simulation

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_coordinates = model_state(model_name, relative_entity_name)
            x = model_coordinates.pose.position.x
            y = model_coordinates.pose.position.y

            orientation = PyKDL.Rotation.Quaternion(model_coordinates.pose.orientation.x,
                                                 model_coordinates.pose.orientation.y,
                                                 model_coordinates.pose.orientation.z,
                                                 model_coordinates.pose.orientation.w)

            euler_orientation = orientation.GetRPY()
            yaw = euler_orientation[2]

            x_vel = model_coordinates.twist.linear.x
            y_vel = model_coordinates.twist.linear.y
            yaw_vel = model_coordinates.twist.angular.z

            return [x, y, yaw, x_vel, y_vel, yaw_vel]
        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def get_link_state(self, link_name, reference_frame=''):
        # method used to retrieve link state from gazebo simulation

        rospy.wait_for_service('/gazebo/get_link_state')
        try:
            link_state_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            link_coordinates = link_state_srv(link_name, reference_frame).link_state
            x = link_coordinates.pose.position.x
            y = link_coordinates.pose.position.y
            z = link_coordinates.pose.position.z

            orientation = PyKDL.Rotation.Quaternion(link_coordinates.pose.orientation.x,
                                                 link_coordinates.pose.orientation.y,
                                                 link_coordinates.pose.orientation.z,
                                                 link_coordinates.pose.orientation.w)

            euler_orientation = orientation.GetRPY()
            roll = euler_orientation[0]
            pitch = euler_orientation[1]
            yaw = euler_orientation[2]

            x_vel = link_coordinates.twist.linear.x
            y_vel = link_coordinates.twist.linear.y
            z_vel = link_coordinates.twist.linear.z
            roll_vel = link_coordinates.twist.angular.x
            pitch_vel = link_coordinates.twist.angular.y
            yaw_vel = link_coordinates.twist.angular.z

            return x, y, z, roll, pitch, yaw, x_vel, y_vel, z_vel, roll_vel, pitch_vel,  yaw_vel
        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def publish_target_marker(self, target_pose):
        t_marker = Marker()
        t_marker.type=1 #=>CUBE
        t_marker.action=0
        t_marker.frame_locked=1
        t_marker.pose.position.x=target_pose[0]
        t_marker.pose.position.y=target_pose[1]
        t_marker.pose.position.z=target_pose[2]
        rpy_orientation = PyKDL.Rotation.RPY(target_pose[3],target_pose[4],target_pose[5])
        q_orientation = rpy_orientation.GetQuaternion()
        t_marker.pose.orientation.x = q_orientation[0]
        t_marker.pose.orientation.y = q_orientation[1]
        t_marker.pose.orientation.z = q_orientation[2]
        t_marker.pose.orientation.w = q_orientation[3]
        t_marker.scale.x=0.1
        t_marker.scale.y=0.1
        t_marker.scale.z=0.1
        t_marker.id=0
        t_marker.header.stamp=rospy.Time.now()
        t_marker.header.frame_id= self.reference_frame
        t_marker.color.a=0.7
        t_marker.color.r=1.0  # red
        t_marker.color.g=0.0
        t_marker.color.b=0.0
        self.target_pub.publish(t_marker)

    def callbackUR(self,data):

        self.ur_state[0:6]  = copy.deepcopy(data.position[0:6])
        self.ur_state[6:12] = copy.deepcopy(data.velocity[0:6])

    def shoulder_collision_callback(self,data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["shoulder"]=True

    def upper_arm_collision_callback(self,data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["upper_arm"]=True

    def forearm_collision_callback(self,data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["forearm"]=True

    def wrist_1_collision_callback(self,data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["wrist_1"]=True

    def wrist_2_collision_callback(self,data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["wrist_2"]=True

    def wrist_3_collision_callback(self,data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["wrist_3"]=True
