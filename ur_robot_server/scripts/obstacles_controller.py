#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState
from scipy import signal, interpolate 
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

        rospy.Subscriber("move_obstacle", Bool, self.callback_move_obstacle)
    
    def callback_move_obstacle(self,data):
        global move 
        if data.data == True:
            move = True
        else:
            move = False

    def get_triangle_wave(self, x, y, amplitude, frequency, offset):

        # Create array with time samples over 1 full function period
        self.samples_len = int(self.sampling_rate / frequency)
        t = np.linspace(0, (1/frequency), self.samples_len)

        x_function = np.full(self.samples_len, x)
        y_function = np.full(self.samples_len, y)
        z_function = offset + amplitude * signal.sawtooth(2 * np.pi * frequency * t, 0.5)

        return x_function, y_function, z_function
    
    def get_3d_spline(self, x_min, x_max, y_min, y_max, z_min, z_max, n_points = 10, n_sampling_points = 4000):

        # Number of random points used to interpolate a spline
        n_points = int(n_points)
        # By increasing the number of sampling points the speed of the object decreases
        n_sampling_points = int(n_sampling_points)
        # Create array with time samples over 1 full function period

        self.samples_len = n_sampling_points


        x = np.random.uniform(x_min,x_max,n_points)
        y = np.random.uniform(y_min,y_max,n_points)
        z = np.random.uniform(z_min,z_max,n_points)

        # set last point equal to first to have a closed trajectory
        x[n_points-1] = x[0]
        y[n_points-1] = y[0]
        z[n_points-1] = z[0]

        smoothness = 0
        tck, u = interpolate.splprep([x,y,z], s=smoothness)
        # x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
        u_fine = np.linspace(0,1,n_sampling_points)
        x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

        x_function = x_fine
        y_function = y_fine
        z_function = z_fine

        return x_function, y_function, z_function

    def obstacle_velocity_publisher(self):
        
        while not rospy.is_shutdown():
            if move:
                i = 0 
                if rospy.get_param("target_function") == "triangle_wave":
                    x = rospy.get_param("x", 0.0)
                    y = rospy.get_param("y", 0.0)
                    a = rospy.get_param("z_amplitude", 0.1)
                    f = rospy.get_param("z_frequency", 1.0)
                    o = rospy.get_param("z_offset", 1.0)
                    x_function, y_function, z_function = self.get_triangle_wave(x, y, a, f, o)
                elif rospy.get_param("target_function") == "3d_spline":
                    x_min = rospy.get_param("x_min")
                    x_max = rospy.get_param("x_max")
                    y_min = rospy.get_param("y_min")
                    y_max = rospy.get_param("y_max")
                    z_min = rospy.get_param("z_min")
                    z_max = rospy.get_param("z_max")
                    n_points = rospy.get_param("n_points", 10)
                    n_sampling_points = rospy.get_param("n_sampling_points", 4000)
                    x_function, y_function, z_function = self.get_3d_spline(x_min, x_max, y_min, y_max, z_min, z_max, n_points, n_sampling_points)
                while move: 
                    i = i % self.samples_len
                    self.obstacle.pose.position.x = x_function[i]
                    self.obstacle.pose.position.y = y_function[i]
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
