#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import PyKDL
import copy
from threading import Event
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

class RosBridge:

    def __init__(self, real_robot=False):

        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        self.real_robot = real_robot

        # Publisher to Command Handler
        self.env_cmd_vel_pub = rospy.Publisher('env_cmd_vel', Twist, queue_size=1)
        
        # Subscriber to robot Odometry
        if self.real_robot:
            rospy.Subscriber('odom', Odometry, self.callbackOdometry, queue_size=1)
        else:
            rospy.Subscriber('odom_comb', Odometry, self.callbackOdometry, queue_size=1)

        # Robot State
        self.mir_position = [0.0] * 2
        self.mir_twist = [0.0] *2

        rospy.Subscriber('robot_pose', Pose, self.callbackState, queue_size=1)

        self.rate = rospy.Rate(10)  # 30Hz
        self.reset.set()

    def get_state(self):
        self.get_state_event.clear()
        # Get environment state

        state = []
        state_dict = {}

    
        mir_position = copy.deepcopy(self.mir_position)
        mir_twist = copy.deepcopy(self.mir_twist)
        state += mir_position
        state += mir_twist
        state_dict['pos_x'] = mir_position[0]
        state_dict['pos_y'] = mir_position[1]
        state_dict['lin_vel'] = mir_twist[0]
        state_dict['ang_vel'] = mir_twist[1]

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State(state=state, state_dict=state_dict, success= True)
        
        return msg

    def set_state(self, state_msg):

        # Clear reset Event
        self.reset.clear()
        pos_x = state_msg.state_dict['pos_x']
        pos_y = state_msg.state_dict['pos_y']

        if not self.real_robot :
            # Set Gazebo Robot Model state
            self.set_model_state('mir', [pos_x, pos_y, 0])

        # Set reset Event
        self.reset.set()

        # Sleep time set manually to allow gazebo to reposition model
        rospy.sleep(0.2)

        return 1

    def publish_env_cmd_vel(self, lin_vel, ang_vel):

        msg = Twist()
        msg.linear.x = lin_vel
        msg.angular.z = ang_vel
        self.env_cmd_vel_pub.publish(msg)
        # Sleep time set manually to achieve approximately 10Hz rate
        rospy.sleep(0.07)
        return lin_vel, ang_vel

    def get_robot_state(self):
        # method to get robot position from real mir
        return self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta, self.robot_twist.linear.x, self.robot_twist.linear.y, self.robot_twist.angular.z

    def set_model_state(self, model_name, state):
        # Set Gazebo Model State
        rospy.wait_for_service('/gazebo/set_model_state')

        start_state = ModelState()
        start_state.model_name = model_name
        start_state.pose.position.x = state[0]
        start_state.pose.position.y = state[1]
        orientation = PyKDL.Rotation.RPY(0,0, state[2])
        start_state.pose.orientation.x, start_state.pose.orientation.y, start_state.pose.orientation.z, start_state.pose.orientation.w = orientation.GetQuaternion()

        start_state.twist.linear.x = 0.0
        start_state.twist.linear.y = 0.0
        start_state.twist.linear.z = 0.0
        start_state.twist.angular.x = 0.0
        start_state.twist.angular.y = 0.0
        start_state.twist.angular.z = 0.0

        try:
            set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state/', SetModelState)
            set_model_state_client(start_state)
        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def callbackState(self,data):
        # If state is not being reset proceed otherwise skip callback
        if self.reset.isSet():
            x = data.position.x
            y = data.position.y

            # Update internal Position variable
            self.mir_position = copy.deepcopy([x, y])
        else:
            pass

    def callbackOdometry(self, data):
        lin_vel = data.twist.twist.linear.x
        ang_vel = data.twist.twist.angular.z

        # Update internal Twist variable
        self.mir_twist = copy.deepcopy([lin_vel, ang_vel])
