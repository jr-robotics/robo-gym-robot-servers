#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState
from scipy import signal 
import numpy as np 

move = False 
class ObstacleController:
    def __init__(self):
        
        # ac_rate = rospy.get_param("~action_cycle_rate")

        # Obstacle Model State publisher
        self.obstacle_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)

        self.obstacle = ModelState()
        self.obstacle.model_name = "box100"
        self.obstacle.pose = Pose()
        self.obstacle.twist = Twist()

        # Generate function sampling times
        self.sampling_rate = 100 
        self.total_sample_time = 16
        self.samples_len = self.sampling_rate * self.total_sample_time
        self.t = np.linspace(0, self.total_sample_time, self.samples_len)

        rospy.Subscriber("move_obstacle", Bool, self.callback_move_obstacle)
    
    def callback_move_obstacle(self,data):
        global move 
        if data.data == True:
            move = True
        else:
            move = False

    def get_triangle_wave(self, amplitude, frequency, offset):

        return offset + amplitude * signal.sawtooth(2 * np.pi * frequency * self.t, 0.5)

    def obstacle_velocity_publisher(self):
        
        while not rospy.is_shutdown():
            if move:
                i = 0 
                self.obstacle.pose.position.x = rospy.get_param("x", 0.0)
                self.obstacle.pose.position.y = rospy.get_param("y", 0.0)
                a = rospy.get_param("z_amplitude", 0.1)
                f = rospy.get_param("z_frequency", 1.0)
                o = rospy.get_param("z_offset", 1.0)
                z_function = self.get_triangle_wave(a, f, o)
                while move: 
                    i = i % self.samples_len
                    self.obstacle.pose.position.z = z_function[i]
                    self.obstacle_pub.publish(self.obstacle)
                    rospy.Rate(self.sampling_rate).sleep()
                    i = i + 1
                # Move obstacle up in the air 
                self.obstacle.pose.position.z = 3.0
                self.obstacle_pub.publish(self.obstacle)
                rospy.Rate(self.sampling_rate).sleep()
            else:
                pass 



if __name__ == '__main__':
    try:
        rospy.init_node('obstacle_controller')
        oc = ObstacleController()
        oc.obstacle_velocity_publisher()
    except rospy.ROSInterruptException:
        pass
