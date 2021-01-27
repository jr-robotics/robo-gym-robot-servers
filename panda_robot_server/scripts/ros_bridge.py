#! /usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker
import copy
# See https://docs.python.org/3/library/threading.html#event-objects
from threading import Event
import time
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2


class PandaRosBridge:

    def __init__(self, real_robot=False):

        # Event is claer while initialization or set_state is going on
        self.reset = Event()
        self._unlock_reset_event()
        self.get_state_event = Event()
        self._lock_state_event()

        self.real_robot = real_robot

        # TODO publisher, subscriber, target and state
        self.target = [0.0] * 1  # TODO define number of target floats
        self.panda_state = [0.0] * 1  # TODO define number of panda states

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Robot control rate
        self.sleep_time = (1.0 / rospy.get_param("~action_cycle_rate")) - 0.01
        self.control_period = rospy.Duration.from_sec(self.sleep_time)

        self.reference_frame = rospy.get_param("~reference_frame", "base")
        self.ee_frame = "tool0"
        self.target_frame = "target"

        if not self.real_robot:
            # Subscribers to link collision sensors topics

            # TODO add rospy.Subsribers
            self.collision_sensors = dict.fromkeys([], False)  # TODO add keys
            pass

        # TODO currently not used
        self.safe_to_move = True

        self.obstacle_controller = rospy.get_param(
            "~obstacle_controller", False)

        # Target mode
        self.target_mode = rospy.get_param("~target_mode", "fixed")
        self.target_mode_name = rospy.get_param("~target_model_name", "box100")

    def get_state(self):
        self._unlock_state_event()

        # Get environment state
        state = []  # TODO currently not used in function

        if self.target_mode == "fixed":
            target = copy.deepcopy(self.target)
        else:
            raise ValueError
            # raise ValueError as err(
            #     "Target mode was ill defined. Got error type: " +
            #     str(type(err)) + " with message: " + err.message)

        panda_state = copy.deepcopy(self.panda_state)

        # TODO
        # (position, quaternion) = self.tf_listener.lookupTransform(self.reference_frame)
        # ee_to_base_transform = position + quaternion

        if self.real_robot:
            panda_collision = False
        else:
            panda_collision = any(self.collision_sensors.values())

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State()
        msg.state.extend(target)
        msg.state.extend(panda_state)
        # msg.state.extend(ee_to_base_transform)
        msg.state.extend([panda_collision])
        msg.success = 1

        return msg

    def set_state(self, state_msg):
        # Set environment state
        state = state_msg.state
        self._unlock_reset_event()
        # Set target internal value
        if self.target_mode == "fixed":
            # TODO found out how many state values are needed for panda
            self.target = copy.deepcopy(state[0:6])
            # Publish Target Marker
            self.publish_target_marker(self.target)

        # TODO setup objects movement
        # if self.objects_controller:

        # TODO reset_steps and init of corresponding variables

        self._lock_reset_event()

        return 1

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
