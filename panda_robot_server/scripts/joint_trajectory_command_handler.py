#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from queue import Queue


class JointTrajectoryCH:

    def __init__(self):
        rospy.init_node('joint_trajectory_command_handler')
        self.real_robot = rospy.get_param('~real_robot')
        ac_rate = rospy.get_param('~action_cycle_rate')
        self.rate = rospy.Rate(ac_rate)

        self._init_publisher()

        self.max_queue_size = 1
        self.queue = Queue(maxsize=self.max_queue_size)

        # Subscriber to JointTrajectory command coming from environment
        rospy.Subscriber('env_arm_command', JointTrajectory,
                         self.callback_env_joint_trajetory, queue_size=self.max_queue_size)

        # Flag used to publish empty JointTrajectory only once when interrupting execution
        self.stop_flag = False

    def _init_publisher(self):
        """Initializes the publisher to JointTrajectory robot controller
        """
        pub_queue_size = 10
        if self.real_robot:
            self.jt_pub = rospy.Publisher(
                '/pos_traj_controller/command', JointTrajectory, queue_size=pub_queue_size)
        else:
            self.jt_pub = rospy.Publisher(
                '/eff_joint_traj_controller/command', JointTrajectory, queue_size=pub_queue_size)

    def callback_env_joint_trajetory(self, data):
        """Callback function for the joint trajectory of the environment
           - In case of a callback, the provided data object is put into the command queue to be executed next

        Args:
            data ([type]): command to be executed
        """
        try:
            # Add to the Queue the next command to execute
            self.queue.put(data)
        except:
            pass

    def joint_trajectory_publisher(self):
        """Publishes the commands stored in self.queue
           - If self.queue is empty, an empty JointTrajectory message is published
        """
        while not rospy.is_shutdown():
            if self.queue.full():
                # If a command from the environment is waiting to be executed,
                # publish the command, otherwise preempt trajectory
                self.jt_pub.publish(self.queue.get())
                self.stop_flag = False
            else:
                # If the empty JointTrajectory message has not been published,
                # publish it and set the stop flag to True, else pass
                if not self.stop_flag:
                    self.jt_pub.publish(JointTrajectory())
                    self.stop_flag = True
                else:  # TODO - is this condition ever needed?
                    pass


if __name__ == '__main__':
    try:
        command_handler = JointTrajectoryCH()
        command_handler.joint_trajectory_publisher()
    except rospy.ROSInterruptException as err:
        rospy.loginfo(
            'Joint Trajectory Command Handler interrupted: {}'.format(err))
