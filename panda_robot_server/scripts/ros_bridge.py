#! /usr/bin/env python

import rospy
import tf
import copy
from threading import Event
import time
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2


class PandaRosBridge:

    def __init__(self, real_robot=False):

        # Event is claer while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

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
        self.get_state_event.clear()
        # Get environment state
        state = []

        if self.target_mode == "fixed":
            target = copy.deepcopy(self.target)
        else:
            raise ValueError as err
            print "Target mode was ill defined. Got error type: " + \
                str(type(err)) + " with message: " + err.message

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
