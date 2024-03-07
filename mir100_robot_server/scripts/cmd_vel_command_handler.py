#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from queue import Queue

class CmdVelCH:
    def __init__(self):
        rospy.init_node('cmd_vel_command_handler')
        # Publisher to Differential Drive robot controller
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        ac_rate = rospy.get_param('~action_cycle_rate')
        self.rate = rospy.Rate(ac_rate)
        # Subscriber to Velocity Command coming from Environment
        rospy.Subscriber('env_cmd_vel', Twist, self.callback_env_cmd_vel, queue_size=1)
        self.msg = Twist()
        # Queue with maximum size 1
        self.queue = Queue(maxsize=1)

    def callback_env_cmd_vel(self, data):
        try:
            # Add to the Queue the next command to execute
            self.queue.put(data)
        except:
            pass

    def cmd_vel_publisher(self):
        while not rospy.is_shutdown():
            # If a command from the environment is waiting to be executed,
            # publish the command, otherwise publish zero velocity message
            if self.queue.full():
                self.cmd_vel_pub.publish(self.queue.get())
            else:
                self.cmd_vel_pub.publish(Twist())
            self.rate.sleep()


if __name__ == '__main__':
    try:
        ch = CmdVelCH()
        ch.cmd_vel_publisher()
    except rospy.ROSInterruptException:
        pass
