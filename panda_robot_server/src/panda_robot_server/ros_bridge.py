#! /usr/bin/env python

import rospy
import tf2_ros
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Bool, Header
from franka_interface import ArmInterface
import copy
# See https://docs.python.org/3/library/threading.html#event-objects
from threading import Event
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2
from collections import deque
import numpy as np

class PandaRosBridge:

    def __init__(self, real_robot=False):
        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        self.real_robot = real_robot
        self.arm = ArmInterface()

        self.custom_joint_states_handler = bool(rospy.get_param('~custom_joint_states_handler', False))

        self.history_limit = 0

        if self.custom_joint_states_handler:
            rospy.loginfo("Initializing custom joint states handler.")
            self.joint_position = dict.fromkeys(self.arm._joint_names, 0.0)
            self.joint_velocity = dict.fromkeys(self.arm._joint_names, 0.0)
            self.joint_effort = dict.fromkeys(self.arm._joint_names, 0.0)

            rospy.Subscriber('/joint_states', JointState, self._on_joint_states)

            self.new_vel_weight = float(rospy.get_param("~new_vel_weight", 1))
            self.new_eff_weight = float(rospy.get_param("~new_eff_weight", 1))
            self.history_limit = int(rospy.get_param("~history_limit", 1))
              
            self.joint_vel_history = deque([])
            self.joint_eff_history = deque([])
            self.keep_history = False

        self.panda_joint_num = len(self.arm._joint_names)
        
        # Robot control
        self.arm_cmd_pub = rospy.Publisher('env_arm_command', JointTrajectory, queue_size=1) # joint_trajectory_command_handler publisher
    
        self.action_cycle_rate = rospy.get_param("~action_cycle_rate", 1000.0) # default value corresponds to control rate of a Panda
        # control_rate_float = rospy.get_param("~action_cycle_rate", 1000.0) # default value corresponds to control rate of a Panda
        self.control_rate = rospy.Rate(self.action_cycle_rate)
      
        # int, not less than 1
        self.subsampling_factor = max(int(rospy.get_param("~subsampling_factor", 1)), 1)

        # int, not less than 0
        self.static_subsamples = max(int(rospy.get_param("~static_subsamples", 1)), 0)
        self.subsample_control_rate = rospy.Rate(self.action_cycle_rate * self.subsampling_factor)

        self.max_velocity_scale_factor = float(rospy.get_param("~max_velocity_scale_factor"))
        self.velocity_limits = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61] 
        self.zero_vel= [0., 0., 0., 0., 0., 0., 0.]  
        self.zero_vel_cmd = self._transform_panda_list_to_dict(self.zero_vel[0:7])  
        self.velocity_limits = [x * self.max_velocity_scale_factor for x in self.velocity_limits] 
        self.robot_moving = False 
        # Robot frames
        self.reference_frame = rospy.get_param('~reference_frame', 'base')
        self.ee_frame = rospy.get_param("~ee_frame", "panda_hand")

        # TF2
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.static_tf2_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Collision detection 
        if not self.real_robot:
            rospy.Subscriber("panda_link1_collision", ContactsState, self._on_link1_collision)
            rospy.Subscriber("panda_link2_collision", ContactsState, self._on_link2_collision)
            rospy.Subscriber("panda_link3_collision", ContactsState, self._on_link3_collision)
            rospy.Subscriber("panda_link4_collision", ContactsState, self._on_link4_collision)
            rospy.Subscriber("panda_link5_collision", ContactsState, self._on_link5_collision)
            rospy.Subscriber("panda_link6_collision", ContactsState, self._on_link6_collision)
            rospy.Subscriber("panda_link7_collision", ContactsState, self._on_link7_collision)
            rospy.Subscriber("panda_leftfinger_collision", ContactsState, self._on_leftfinger_collision)
            rospy.Subscriber("panda_rightfinger_collision", ContactsState, self._on_rightfinger_collision)
        # Initialization of collision sensor flags
        self.collision_sensors = dict.fromkeys(['panda_link1', 'panda_link2', 'panda_link3', 'panda_link4', \
                                                'panda_link5','panda_link6','panda_link7','panda_leftfinger','panda_rightfinger',], False)

        # Robot Server mode
        self.rs_mode = rospy.get_param('~rs_mode')

        # Action Mode
        self.action_mode = rospy.get_param('~action_mode')
        if self.action_mode.endswith("_pos"):
            self.pos_action_impl = rospy.get_param('~pos_action_impl', 'set_pos')

        # Objects  Controller 
        self.objects_controller = rospy.get_param("objects_controller", False)
        self.n_objects = int(rospy.get_param("n_objects", 0))
        if self.objects_controller:
            self.move_objects_pub = rospy.Publisher('move_objects', Bool, queue_size=10)
            # Get objects model name
            self.objects_model_name = []
            for i in range(self.n_objects):
                self.objects_model_name.append(rospy.get_param("object_" + repr(i) + "_model_name"))
            # Get objects TF Frame
            self.objects_frame = []
            for i in range(self.n_objects):
                self.objects_frame.append(rospy.get_param("object_" + repr(i) + "_frame"))
         
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
                if name in self.arm._joint_names:
                    self.joint_position[name] = msg.position[idx]
                    self.joint_velocity[name] = msg.velocity[idx] * self.new_vel_weight + self.joint_velocity[name] * (1 - self.new_vel_weight)
                    self.joint_effort[name] = msg.effort[idx] * self.new_eff_weight + self.joint_effort[name] * (1 - self.new_eff_weight)
            if self.history_limit and self.keep_history:
                self.joint_vel_history.append(self.joint_velocity)
                while len(self.joint_vel_history) > self.history_limit:
                    self.joint_vel_history.popleft()
                self.joint_eff_history.append(self.joint_effort)
                while len(self.joint_eff_history) > self.history_limit:
                    self.joint_eff_history.popleft()
        else:
            rospy.logwarn("self.get_state_event.is_set() was False")
    

    def get_arguments(self):
        # All this arguments are populated by default in the ros launch files
        action_cycle_rate = self.action_cycle_rate
        rs_mode = self.rs_mode
        action_mode = self.action_mode
        
        # Create and fill Arguments
        args = robot_server_pb2.Arguments(action_cycle_rate=action_cycle_rate, 
                                     rs_mode = rs_mode,
                                     action_mode = action_mode,
                                     success= True)
        
        return args

    def get_joint_position(self):
        if self.custom_joint_states_handler:
            return self.joint_position
        return self.arm.joint_angles()
    
    def get_joint_velocity(self):
        if self.custom_joint_states_handler:
            return self.joint_velocity
        return self.arm.joint_velocities()

    def get_joint_effort(self):
        if self.custom_joint_states_handler:
            return self.joint_effort
        return self.arm.joint_efforts()
    
    def summarize_joint_vel_history(self):
        result = copy.deepcopy(self.joint_velocity)

        if self.history_limit:
            num_entries = len(self.joint_vel_history)
            rospy.logwarn("summarizing velocity from {} entries".format(num_entries))
            if num_entries:
                for joint_name in self.joint_velocity:
                    result[joint_name] = sum([entry[joint_name] for entry in self.joint_vel_history]) / num_entries
            self.joint_vel_history.clear()
        return result

    def summarize_joint_eff_history(self):
        result = copy.deepcopy(self.joint_effort)

        if self.history_limit:
            num_entries = len(self.joint_eff_history)
            if num_entries:
                for joint_name in self.joint_effort:
                    result[joint_name] = sum([entry[joint_name] for entry in self.joint_eff_history]) / num_entries
            self.joint_eff_history.clear()
        return result

    def get_state(self):
        self.get_state_event.clear()

        # Get environment state
        state =[]
        state_dict = {}

        joint_position = copy.deepcopy(self.get_joint_position())
        joint_velocity = self.summarize_joint_vel_history()
        joint_effort = self.summarize_joint_eff_history()

        ee_to_ref_trans = self.tf2_buffer.lookup_transform(self.reference_frame, self.ee_frame, rospy.Time(0))
        ee_to_ref_trans_list = self._transform_to_list(ee_to_ref_trans)
        
        if self.rs_mode == '1object':
            # Object 0 Pose 
            object_0_trans = self.tf2_buffer.lookup_transform(self.reference_frame, self.objects_frame[0], rospy.Time(0))
            object_0_trans_list = self._transform_to_list(object_0_trans)
            state += object_0_trans_list
            state_dict.update(self._get_transform_dict(object_0_trans, 'object_0_to_ref'))
        
        if True: # self.rs_mode == 'only_robot':
            # Joint Positions and Joint Velocities

            state += self._get_joint_ordered_value_list(joint_position)
            state += self._get_joint_ordered_value_list(joint_velocity)
            state += self._get_joint_ordered_value_list(joint_effort)
            state_dict.update(self._get_joint_states_dict(joint_position, joint_velocity, joint_effort))

            # ee to ref transform

            state += ee_to_ref_trans_list
            state_dict.update(self._get_transform_dict(ee_to_ref_trans, 'ee_to_ref'))
        
            # Collision sensors
            collision = any(self.collision_sensors.values())
            state += [collision]
            state_dict['in_collision'] = float(collision)

        else: 
            raise ValueError

        self.get_state_event.set()
        
        # Create and fill State message
        msg = robot_server_pb2.State(state=state, state_dict=state_dict, success= True)

        return msg
    
        
    def set_state(self, state_msg):
        if self.custom_joint_states_handler:
            self.keep_history = False
            self.joint_vel_history = deque([])
            self.joint_eff_history = deque([])

        # Set environment state
        # TODO replace the hardcoded numbers here
        if self.real_robot:  # When using real robot. the velocity message should not suddenly stop publishing.
            for i in range(100):
                self.arm.set_joint_velocities(self.zero_vel_cmd)
                # print(self.zero_vel_cmd)      
                self.control_rate.sleep()

        rospy.sleep(3.0)  # Only use in velocity mode. when using high publish rate (400Hz)
        state = state_msg.state

        # Clear reset Event
        self.reset.clear()
        
        # Setup Objects movement
        if self.objects_controller:
            # Stop movement of objects
            msg = Bool()
            msg.data = False
            self.move_objects_pub.publish(msg)

            # Loop through all the string_params and float_params and set them as ROS parameters
            for param in state_msg.string_params:
                rospy.set_param(param, state_msg.string_params[param])

            for param in state_msg.float_params:
                rospy.set_param(param, state_msg.float_params[param])

        positions = self._get_joint_position_dict_from_rs_dict(state_msg.state_dict)

        # for _ in range(reset_steps):
        #     self.arm.set_joint_positions(positions)
        #self.arm.set_joint_positions_velocities(positions[0:7], [0.0]*7) 
        self.arm.move_to_joint_positions(positions, use_moveit=False)

        if not self.real_robot:
            # Reset collision sensors flags
            self.collision_sensors.update(dict.fromkeys(['panda_link1', 'panda_link2', 'panda_link3', 'panda_link4', \
                                                'panda_link5','panda_link6','panda_link7','panda_leftfinger','panda_rightfinger',], False))
        # Start movement of objects
        if self.objects_controller:
            msg = Bool()
            msg.data = True
            self.move_objects_pub.publish(msg)

        self.reset.set()
        self.control_rate.sleep()

        self.keep_history = True
        return True

    def send_action(self, action): # adding action modes

        if self.action_mode == 'abs_pos':
            executed_action = self.publish_env_arm_cmd_abs_pos(action)
        
        elif self.action_mode == 'delta_pos':
            executed_action = self.publish_env_arm_cmd_delta_pos(action)

        elif self.action_mode == 'abs_vel':
            executed_action = self.publish_env_arm_cmd_abs_vel(action)

        elif self.action_mode == 'delta_vel':
            executed_action = self.publish_env_arm_cmd_delta_vel(action)

        return executed_action


    def publish_env_arm_cmd_abs_pos(self, abs_pos_cmd):
        abs_pos_cmd_dict = self._transform_panda_list_to_dict(abs_pos_cmd[0:7])
        self.execute_abs_pos_cmd(abs_pos_cmd_dict)
        return abs_pos_cmd


    def publish_env_arm_cmd_delta_pos(self, delta_pos_cmd): # Joint Position Controller difference mode
        position_cmd = []
        joint_position = copy.deepcopy(self.get_joint_position())
        for idx, name in enumerate(self.arm._joint_names):
            pos = joint_position[name]
            cmd = delta_pos_cmd[idx]
            position_cmd.append(pos + cmd)
        abs_pos_cmd_dict = self._transform_panda_list_to_dict(position_cmd[0:7])
        self.execute_abs_pos_cmd(abs_pos_cmd_dict)
        return delta_pos_cmd
    

    def execute_abs_pos_cmd(self, abs_pos_cmd_dict):
        #value = self.get_joint_position()
        #rospy.logwarn("Current-Pre:\t" + ", ".join([(key + ": " + str(value[key])) for key in sorted(value)]))

        value = abs_pos_cmd_dict
        rospy.logwarn("Command:\t" + ", ".join([(key + ": " + str(value[key])) for key in sorted(value)]))

        if self.pos_action_impl == 'set_pos_vel':   
            
            # rospy.logwarn("pos action impl set_pos_vel")
        
            # this approach seems to be equivalent to the default set_position approach due to the lack of meaningful velocity values
            abs_pos_cmd_list = [abs_pos_cmd_dict[name] for name in self.arm._joint_names]
            self.arm.set_joint_positions_velocities(abs_pos_cmd_list, self.zero_vel)
            self.control_rate.sleep()

        elif self.pos_action_impl == 'set_pos_subsampling':

            # rospy.logwarn("pos action impl set_pos_subsampling")

            # set_position but in smaller steps; sleeping is included
            self.set_position_subsampled(abs_pos_cmd_dict)

        elif self.pos_action_impl == 'move_to_joint_pos':
            
            # rospy.logwarn("pos action impl move_to_joint_pos")

            # smooth movement, but has problems adhering to the timeout
            # (even after removing the hardcoded sleep in the method implementation - can only work for a typical action rate if that is commented out!)
            timeout = self.control_rate.remaining().to_sec()
            rospy.logwarn("move command timeout: " + str(timeout))
            if timeout > 0:
            # this approach cannot handle the short timeouts resulting from a usual control rate:
                self.arm.move_to_joint_positions(abs_pos_cmd_dict, timeout=timeout, use_moveit=False)            
        
            # maintain control rate (should sleep for 0 almost always)
            self.control_rate.sleep()

        else:
            # elif self.pos_action_impl == 'set_pos':  
             
            rospy.logwarn("pos action impl set_pos")      

            # default impl - too high accelerations!
            self.arm.set_joint_positions(abs_pos_cmd_dict)
            self.control_rate.sleep()

        #value = self.get_joint_position()
        #rospy.logwarn("Current-Post:\t" + ", ".join([(key + ": " + str(value[key])) for key in sorted(value)]))



    def set_position_subsampled(self, abs_pos_cmd: dict):
        cmd = copy.deepcopy(abs_pos_cmd)

        start_pos = copy.deepcopy(self.get_joint_position())
        dest_pos = abs_pos_cmd

        # we interpolate between start pos and commanded pos during the nonstatic subsamples, then follow with repetitions of the commanded pos in the static subsamples
        nonstatic_subsamples = max(self.subsampling_factor - self.static_subsamples, 1)

        for i in range(self.subsampling_factor):
            for joint_name in self.arm._joint_names:
                # dest_weight goes up with i, is always > 0 and remains at 1 when reaching nonstatic_subsamples
                dest_weight = min(float(i+1) / nonstatic_subsamples, 1)
                # start_weight goes down
                start_weight = 1.0 - dest_weight
                cmd[joint_name] = start_pos[joint_name] * start_weight + dest_pos[joint_name] * dest_weight
            value = cmd
            rospy.logwarn("subsampling cmd:\t" + ", ".join([(key + ": " + str(value[key])) for key in sorted(value)]))
            self.arm.set_joint_positions(cmd)
            self.subsample_control_rate.sleep()


    def publish_env_arm_cmd_abs_vel(self, abs_vel_cmd):
        velocity_cmd = []
        for idx, name in enumerate(self.arm._joint_names):
            vel_cmd_i = abs_vel_cmd[idx]
            if abs(vel_cmd_i) > self.velocity_limits[idx]:
                vel_cmd_sign = np.sign(vel_cmd_i)
                vel_cmd_i = vel_cmd_sign * self.velocity_limits[idx]
            velocity_cmd.append(vel_cmd_i)      
        abs_vel_cmd_dict = self._transform_panda_list_to_dict(velocity_cmd[0:7])
        self.arm.set_joint_velocities(abs_vel_cmd_dict)
        self.control_rate.sleep()     
        return abs_vel_cmd

    def publish_env_arm_cmd_delta_vel(self, delta_vel_cmd):  # Velocity controller in difference mode.
        velocity_cmd = []
        joint_velocity = copy.deepcopy(self.get_joint_velocity())
        for idx, name in enumerate(self.arm._joint_names):
            vel = joint_velocity[name] # if idx == 0 else 0
            cmd = delta_vel_cmd[idx]
            vel_cmd_i = vel + cmd
            if abs(vel_cmd_i) > self.velocity_limits[idx]:
                vel_cmd_sign = np.sign(vel_cmd_i)
                vel_cmd_i = vel_cmd_sign * self.velocity_limits[idx]
            velocity_cmd.append(vel_cmd_i)      
        abs_vel_cmd_dict = self._transform_panda_list_to_dict(velocity_cmd[0:7])
        self.arm.set_joint_velocities(abs_vel_cmd_dict)
        self.control_rate.sleep()
        return delta_vel_cmd
       
        
    def _transform_panda_list_to_dict(self, panda_list):
        transformed_dict = {}
        for idx, value in enumerate(panda_list):
            current_joint_name = self.arm._joint_names[idx]
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
        
        return [joint_values[name] for name in self.arm._joint_names]

    def _get_joint_position_dict_from_rs_dict(self, rs_dict):

        d = {}
        d['panda_joint1'] = rs_dict['joint1_position']
        d['panda_joint2'] = rs_dict['joint2_position']
        d['panda_joint3'] = rs_dict['joint3_position']
        d['panda_joint4'] = rs_dict['joint4_position']
        d['panda_joint5'] = rs_dict['joint5_position']
        d['panda_joint6'] = rs_dict['joint6_position']
        d['panda_joint7'] = rs_dict['joint7_position']
    
        return d 

    def _on_link1_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors['panda_link1'] = True
    
    def _on_link2_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors['panda_link2'] = True

    def _on_link3_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors['panda_link3'] = True

    def _on_link4_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors['panda_link4'] = True

    def _on_link5_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors['panda_link5'] = True

    def _on_link6_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors['panda_link6'] = True

    def _on_link7_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors['panda_link7'] = True
    
    def _on_leftfinger_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors['panda_leftfinger'] = True

    def _on_rightfinger_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors['panda_rightfinger'] = True
            
