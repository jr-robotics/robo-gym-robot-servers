#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, Pose, Pose2D
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState, GetLinkState
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Float64MultiArray, Header, Bool
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import PyKDL
import copy
from threading import Event
import time
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2


class UrRosBridge:

    def __init__(self,  real_robot=False, ur_model = 'ur10'):

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

        # Obstacle controller publisher
        self.obstacle_controller_pub = rospy.Publisher('move_obstacle', Bool, queue_size=10)

        self.target = [0.0] * 6
        self.ur_state = [0.0] *12

        rospy.Subscriber("joint_states", JointState, self.callbackUR)

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Robot control rate
        self.sleep_time = (1.0/rospy.get_param("~action_cycle_rate")) - 0.01
        self.control_period = rospy.Duration.from_sec(self.sleep_time)

        self.reference_frame = rospy.get_param("~reference_frame", "base")
        self.ee_frame = 'tool0'
        self.target_frame = 'target'


        self.max_velocity_scale_factor = float(rospy.get_param("~max_velocity_scale_factor"))
        if ur_model == 'ur3'or ur_model == 'ur3e':
            self.absolute_ur_joint_vel_limits = [3.14, 3.14, 3.14, 6.28, 6.28, 6.28]
        elif ur_model == 'ur5'or ur_model == 'ur5e':
            self.absolute_ur_joint_vel_limits = [3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
        elif ur_model == 'ur10'or ur_model == 'ur10e' or ur_model == 'ur16e':
            self.absolute_ur_joint_vel_limits = [3.14, 2.09, 2.09, 3.14, 3.14, 3.14]
        else:
            raise ValueError('ur_model not recognized')
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

        self.obstacle_controller = rospy.get_param("~obstacle_controller", False)

        # Target mode
        self.target_mode = rospy.get_param("~target_mode", 'fixed')
        self.target_model_name = rospy.get_param("~target_model_name", 'box100')

        # Second Object
        self.object_02_model_name = 'box200'
        self.object_02_frame = 'object_02'

    def get_state(self):
        self.get_state_event.clear()
        # Get environment state
        state =[]
        if self.target_mode == 'fixed':
            target = copy.deepcopy(self.target)
        elif self.target_mode == 'moving':
            if self.real_robot:
                (t_position, t_quaternion) = self.tf_listener.lookupTransform(self.reference_frame,self.target_frame,rospy.Time(0))
                target = t_position + [0,0,0]
            else:
                pose = self.get_model_state_pose(self.target_model_name)
                # Convert orientation target from Quaternion to RPY
                quaternion = PyKDL.Rotation.Quaternion(pose[3],pose[4],pose[5],pose[6])
                r,p,y = quaternion.GetRPY()
                target = pose[0:3] + [r,p,y]
        elif self.target_mode == '2moving':
            if self.real_robot:
                (t_position, t_quaternion) = self.tf_listener.lookupTransform(self.reference_frame,self.target_frame,rospy.Time(0))
                target = t_position + [0,0,0]
                (o2_position, o2_quaternion) = self.tf_listener.lookupTransform(self.reference_frame,self.object_02_frame,rospy.Time(0))
                object2 = o2_position + [0,0,0]
            else:
                # Target
                t_pose = self.get_model_state_pose(self.target_model_name)
                # Convert orientation target from Quaternion to RPY
                t_quaternion = PyKDL.Rotation.Quaternion(t_pose[3],t_pose[4],t_pose[5],t_pose[6])
                t_r,t_p,t_y = t_quaternion.GetRPY()
                target = t_pose[0:3] + [t_r,t_p,t_y]
                # Object 02
                o2_pose = self.get_model_state_pose(self.object_02_model_name)
                # Convert orientation target from Quaternion to RPY
                o2_quaternion = PyKDL.Rotation.Quaternion(o2_pose[3],o2_pose[4],o2_pose[5],o2_pose[6])
                o2_r,o2_p,o2_y = o2_quaternion.GetRPY()
                object2 = o2_pose[0:3] + [o2_r,o2_p,o2_y]
        else: 
            raise ValueError
            
        ur_state = copy.deepcopy(self.ur_state)

        (position, quaternion) = self.tf_listener.lookupTransform(self.reference_frame,self.ee_frame,rospy.Time(0))

        ee_to_base_transform = position + quaternion

        if self.real_robot:
            ur_collision = False
        else:
            ur_collision = any(self.collision_sensors.values())

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State()
        msg.state.extend(target)
        msg.state.extend(ur_state)
        msg.state.extend(ee_to_base_transform)
        msg.state.extend([ur_collision])
        if self.target_mode == '2moving':
            msg.state.extend(object2)
        msg.success = 1
        
        return msg

    def set_state(self, state_msg):
        # Set environment state
        state = state_msg.state 
        # Clear reset Event
        self.reset.clear()
        # Set target internal value
        if self.target_mode == 'fixed':
            self.target = copy.deepcopy(state[0:6])
            # Publish Target Marker
            self.publish_target_marker(self.target)
        # Stop movement of obstacles
        if self.obstacle_controller:
            msg = Bool()
            msg.data = False
            self.obstacle_controller_pub.publish(msg)
            if state_msg.string_params["function"] == "triangle_wave":
                rospy.set_param("target_function", "triangle_wave")
                rospy.set_param("x", state_msg.float_params["x"])
                rospy.set_param("y", state_msg.float_params["y"])
                rospy.set_param("z_amplitude", state_msg.float_params["z_amplitude"])
                rospy.set_param("z_frequency", state_msg.float_params["z_frequency"])
                rospy.set_param("z_offset",    state_msg.float_params["z_offset"])
                rospy.set_param("n_objects",    state_msg.float_params["n_objects"])
            elif state_msg.string_params["function"] == "3d_spline":
                rospy.set_param("target_function", "3d_spline")
                rospy.set_param("x_min", state_msg.float_params["x_min"])
                rospy.set_param("x_max", state_msg.float_params["x_max"])
                rospy.set_param("y_min", state_msg.float_params["y_min"])
                rospy.set_param("y_max", state_msg.float_params["y_max"])
                rospy.set_param("z_min", state_msg.float_params["z_min"])
                rospy.set_param("z_max", state_msg.float_params["z_max"])
                rospy.set_param("n_points", state_msg.float_params["n_points"])
                rospy.set_param("n_sampling_points", state_msg.float_params["n_sampling_points"])
                rospy.set_param("n_objects",    state_msg.float_params["n_objects"])

        # UR Joints Positions
        reset_steps = int(15.0/self.sleep_time)
        for i in range(reset_steps):
            self.publish_env_arm_cmd(state[6:12])
        if not self.real_robot:
            # Reset collision sensors flags
            self.collision_sensors.update(dict.fromkeys(["shoulder","upper_arm","forearm","wrist_1","wrist_2","wrist_3"], False))
        # Start movement of obstacles
        if self.obstacle_controller:
            msg = Bool()
            msg.data = True
            self.obstacle_controller_pub.publish(msg)

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
        if self.get_state_event.is_set():
            self.ur_state[0:6]  = data.position[0:6]
            self.ur_state[6:12] = data.velocity[0:6]

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
