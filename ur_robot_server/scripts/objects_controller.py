#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState
from scipy import signal, interpolate 
import numpy as np 
import copy

move = False 
class ObstacleController:
    def __init__(self):

        # Obstacle Model State publisher
        self.obstacle_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)

        self.obstacle = ModelState()
        self.obstacle.model_name = "box100"
        self.obstacle.pose = Pose()
        self.obstacle.twist = Twist()

        self.obstacle2 = ModelState()
        self.obstacle2.model_name = "box100_2"
        self.obstacle2.pose = Pose()
        self.obstacle2.twist = Twist()

        # Obstacle position update frequency (Hz)
        self.update_rate = 100 

        # move_obstacle subscriber
        rospy.Subscriber("move_obstacle", Bool, self.callback_move_obstacle)
    
    def callback_move_obstacle(self,data):
        global move 
        if data.data == True:
            move = True
        else:
            move = False

    def get_triangle_wave(self, x, y, amplitude, frequency, offset):

        """Generate samples of triangle wave function with amplitude in the z axis direction.

        Args:
            x (float): x coordinate (m).
            y (float): y coordinate (m).
            amplitude (float): amplitude of the triangle wave (m).
            frequency (float): frequency of the triangle wave (Hz).
            offset (float): offset from the ground of the zero of the triangle wave (m).


        Returns:
            np.array: Samples of the x coordinate of the function over time.
            np.array: Samples of the y coordinate of the function over time.
            np.array: Samples of the z coordinate of the function over time.

        """

        # Create array with time samples over 1 full function period
        sampling_rate = copy.deepcopy(self.update_rate)
        self.samples_len = int(sampling_rate / frequency)
        t = np.linspace(0, (1/frequency), self.samples_len)

        x_function = np.full(self.samples_len, x)
        y_function = np.full(self.samples_len, y)
        z_function = offset + amplitude * signal.sawtooth(2 * np.pi * frequency * t, 0.5)

        return x_function, y_function, z_function
    
    def get_3d_spline(self, x_min, x_max, y_min, y_max, z_min, z_max, n_points = 10, n_sampling_points = 4000):
        
        """Generate samples of the cartesian coordinates of a 3d spline.

        Args:
            x_min (float): min x coordinate of random points used to interpolate spline (m).
            x_max (float): max x coordinate of random points used to interpolate spline (m).
            y_min (float): min y coordinate of random points used to interpolate spline (m).
            y_max (float): max y coordinate of random points used to interpolate spline (m).
            z_min (float): min z coordinate of random points used to interpolate spline (m).
            z_max (float): max z coordinate of random points used to interpolate spline (m).
            n_points (int): number of random points used to interpolate the 3d spline.
            n_sampling_points (int): number of the samples to take over the whole length of the spline.

        Returns:
            np.array: Samples of the x coordinate of the function over time.
            np.array: Samples of the y coordinate of the function over time.
            np.array: Samples of the z coordinate of the function over time.

        """

        # Convert number of points to int
        n_points = int(n_points)
        # Convert number of  sampling points to int
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
        u_fine = np.linspace(0,1,n_sampling_points)
        x_function, y_function, z_function = interpolate.splev(u_fine, tck)

        return x_function, y_function, z_function

    def obstacle_velocity_publisher(self):
        
        while not rospy.is_shutdown():
            if move:
                # Generate Movement Trajectories
                i = 0 
                n_objects = rospy.get_param("n_objects", 1)
                if  n_objects == 1 :
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
                elif n_objects == 2:
                    if rospy.get_param("target_function") == "triangle_wave":
                        x = rospy.get_param("x", 0.0)
                        y = rospy.get_param("y", 0.0)
                        a = rospy.get_param("z_amplitude", 0.1)
                        f = rospy.get_param("z_frequency", 1.0)
                        o = rospy.get_param("z_offset", 1.0)
                        x1_function, y1_function, z1_function = self.get_triangle_wave(x, y, a, f, o)
                        x2_function, y2_function, z2_function = self.get_triangle_wave(x, y, a, f, o)
                    elif rospy.get_param("target_function") == "3d_spline":
                        x_min = rospy.get_param("x_min")
                        x_max = rospy.get_param("x_max")
                        y_min = rospy.get_param("y_min")
                        y_max = rospy.get_param("y_max")
                        z_min = rospy.get_param("z_min")
                        z_max = rospy.get_param("z_max")
                        n_points = rospy.get_param("n_points", 10)
                        n_sampling_points = rospy.get_param("n_sampling_points", 4000)
                        x1_function, y1_function, z1_function = self.get_3d_spline(x_min, x_max, y_min, y_max, z_min, z_max, n_points, n_sampling_points)
                        x2_function, y2_function, z2_function = self.get_3d_spline(x_min, x_max, y_min, y_max, z_min, z_max, n_points, n_sampling_points)

                while move: 
                    i = i % self.samples_len
                    if n_objects == 1:
                        self.obstacle.pose.position.x = x_function[i]
                        self.obstacle.pose.position.y = y_function[i]
                        self.obstacle.pose.position.z = z_function[i]
                        self.obstacle_pub.publish(self.obstacle)
                    if n_objects == 2:
                        self.obstacle.pose.position.x = x1_function[i]
                        self.obstacle.pose.position.y = y1_function[i]
                        self.obstacle.pose.position.z = z1_function[i]
                        self.obstacle_pub.publish(self.obstacle)
                        self.obstacle2.pose.position.x = x2_function[i]
                        self.obstacle2.pose.position.y = y2_function[i]
                        self.obstacle2.pose.position.z = z2_function[i]
                        self.obstacle_pub.publish(self.obstacle2)
                    rospy.Rate(self.update_rate).sleep()
                    i = i + 1
                # Move obstacle up in the air 
                self.obstacle.pose.position.z = 3.0
                self.obstacle_pub.publish(self.obstacle)
                if n_objects == 2:
                    self.obstacle2.pose.position.z = 3.0 
                    self.obstacle_pub.publish(self.obstacle2)
                rospy.Rate(self.update_rate).sleep()
            else:
                pass 



if __name__ == '__main__':
    try:
        rospy.init_node('objects_controller')
        oc = ObstacleController()
        oc.obstacle_velocity_publisher()
    except rospy.ROSInterruptException:
        pass
