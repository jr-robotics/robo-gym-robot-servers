#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelState
from scipy import signal, interpolate 
import numpy as np 
import copy
import os
import random
import tf2_ros
import json

move = False 
class ObjectsController:
    def __init__(self):

        self.real_robot = rospy.get_param("real_robot")

        # Objects Model State publisher
        if not self.real_robot:
            self.set_model_state_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)

        # Objects position update frequency (Hz)
        self.update_rate = 100 

        # move_objects subscriber
        rospy.Subscriber("move_objects", Bool, self.callback_move_objects)
        
        self.reference_frame = rospy.get_param("reference_frame")
        
        # Static TF2 Broadcaster
        self.static_tf2_broadcaster = tf2_ros.StaticTransformBroadcaster()

        object_trajectory_file_name = rospy.get_param("object_trajectory_file_name")
        if object_trajectory_file_name != 'no_file':
            file_path = os.path.join(os.path.dirname(__file__),'../object_trajectories', object_trajectory_file_name + '.json')

            # Load object trajectory file 
            with open(file_path, 'r') as json_file:
                self.p = json.load(json_file)
    
    def callback_move_objects(self, data):
        global move
        if data.data == True:
            move = True
        else:
            move = False
    
    def get_fixed_position(self, x, y, z):
        """Generate trajectory for object in a fixed position

        Args:
            x (float): x coordinate (m).
            y (float): y coordinate (m).
            z (float): z coordinate (m).

        Returns:
            list: x coordinate function
            list: y coordinate function
            list: z coordinate function
        """        
        x_function = [x]
        y_function = [y]
        z_function = [z]
        self.samples_len = 1
        
        return x_function, y_function, z_function 

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
    
    def get_3d_spline(self, x_min, x_max, y_min, y_max, z_min, z_max, n_points=10, n_sampling_points=4000):
        
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

        x = np.random.uniform(x_min, x_max, n_points)
        y = np.random.uniform(y_min, y_max, n_points)
        z = np.random.uniform(z_min, z_max, n_points)

        # set last point equal to first to have a closed trajectory
        x[n_points-1] = x[0]
        y[n_points-1] = y[0]
        z[n_points-1] = z[0]

        smoothness = 0
        tck, u = interpolate.splprep([x, y, z], s=smoothness)
        u_fine = np.linspace(0, 1, n_sampling_points)
        x_function, y_function, z_function = interpolate.splev(u_fine, tck)

        return x_function, y_function, z_function

    def get_3d_spline_ur5_workspace(self, x_min, x_max, y_min, y_max, z_min, z_max, n_points = 10, n_sampling_points = 4000):
        
        """Generate samples of the cartesian coordinates of a 3d spline that do not cross a vertical 
            cylinder of radius r_min centered in 0,0.

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

        r_min_cylinder = 0.2 
        r_min_sphere_base = 0.35 

        # Convert number of points to int
        n_points = int(n_points)
        # Convert number of  sampling points to int
        # By increasing the number of sampling points the speed of the object decreases
        n_sampling_points = int(n_sampling_points)
        # Create array with time samples over 1 full function period

        self.samples_len = n_sampling_points
        
        search = True
        while search:
            x = np.random.uniform(x_min,x_max,n_points)
            y = np.random.uniform(y_min,y_max,n_points)
            z = np.random.uniform(z_min,z_max,n_points)

            # set first point oustide of square of size 0.5m centered in 0,0
            x[0] = random.choice([np.random.uniform(-1.0,-0.5),np.random.uniform(0.5,1.0)])
            y[0] = random.choice([np.random.uniform(-1.0,-0.5),np.random.uniform(0.5,1.0)])

            # set last point equal to first to have a closed trajectory
            x[n_points-1] = x[0]
            y[n_points-1] = y[0]
            z[n_points-1] = z[0]

            smoothness = 0
            tck, u = interpolate.splprep([x,y,z], s=smoothness)
            u_fine = np.linspace(0,1,n_sampling_points)
            x_function, y_function, z_function = interpolate.splev(u_fine, tck)
            
            search = False
            for i in range(len(x_function)):
                if (x_function[i]**2+y_function[i]**2)**(1/2) <= r_min_cylinder or \
                    (x_function[i]**2+y_function[i]**2+z_function[i]**2)**(1/2) <= r_min_sphere_base :
                    search = True

        return x_function, y_function, z_function

    def get_fixed_trajectory(self, trajectory_id):
        # file_name = "obstacle_trajectories.yaml"


        trajectory_name = "trajectory_" + str(int(trajectory_id))
        x_function = self.p[trajectory_name]["x"]
        y_function = self.p[trajectory_name]["y"]
        z_function = self.p[trajectory_name]["z"]

        # self.samples_len = self.p[trajectory_name]["n_sampling_points"]
        self.samples_len = 4000
        return x_function, y_function, z_function 

    def objects_initialization(self):
        self.n_objects = int(rospy.get_param("n_objects", 1))
        # Initialization of ModelState() messages
        if not self.real_robot:
            self.objects_model_state = [ModelState() for i in range(self.n_objects)]
            # Get objects model names
            for i in range(self.n_objects):
                self.objects_model_state[i].model_name = rospy.get_param("object_" + repr(i) +"_model_name")
                self.objects_model_state[i].reference_frame = self.reference_frame
        # Initialization of Objects tf frames names
        self.objects_tf_frame = [rospy.get_param("object_" + repr(i) +"_frame") for i in range(self.n_objects)]
        
        self.move_objects_up()

    def move_objects_up(self):
        # Move objects up in the air 
        for i in range(self.n_objects):
            if not self.real_robot:
                self.objects_model_state[i].pose.position.x = i
                self.objects_model_state[i].pose.position.y = 0.0
                self.objects_model_state[i].pose.position.z = 3.0
                self.set_model_state_pub.publish(self.objects_model_state[i]) 
            # Publish tf of objects
            t = TransformStamped()
            t.header.frame_id = self.reference_frame
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = self.objects_tf_frame[i]
            t.transform.translation.x = i
            t.transform.translation.y = 0.0
            t.transform.translation.z = 3.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.static_tf2_broadcaster.sendTransform(t)

    def objects_state_update_loop(self):
        while not rospy.is_shutdown():
            if move:
                # Generate Movement Trajectories
                objects_trajectories = []
                for i in range(self.n_objects):
                    function = rospy.get_param("object_" + repr(i) +"_function")
                    if function  == "fixed_position":
                        x = rospy.get_param("object_" + repr(i) + "_x")
                        y = rospy.get_param("object_" + repr(i) + "_y")
                        z = rospy.get_param("object_" + repr(i) + "_z")
                        x_trajectory, y_trajectory, z_trajectory = self.get_fixed_position(x,y,z)
                    elif function == "triangle_wave":
                        x = rospy.get_param("object_" + repr(i) + "_x")
                        y = rospy.get_param("object_" + repr(i) + "_y")
                        a = rospy.get_param("object_" + repr(i) + "_z_amplitude")
                        f = rospy.get_param("object_" + repr(i) + "_z_frequency")
                        o = rospy.get_param("object_" + repr(i) + "_z_offset")
                        x_trajectory, y_trajectory, z_trajectory = self.get_triangle_wave(x, y, a, f, o)
                    elif function == "3d_spline":
                        x_min = rospy.get_param("object_" + repr(i) + "_x_min")
                        x_max = rospy.get_param("object_" + repr(i) + "_x_max")
                        y_min = rospy.get_param("object_" + repr(i) + "_y_min")
                        y_max = rospy.get_param("object_" + repr(i) + "_y_max")
                        z_min = rospy.get_param("object_" + repr(i) + "_z_min")
                        z_max = rospy.get_param("object_" + repr(i) + "_z_max")
                        n_points = rospy.get_param("object_" + repr(i) + "_n_points")
                        n_sampling_points = rospy.get_param("n_sampling_points")
                        x_trajectory, y_trajectory, z_trajectory = self.get_3d_spline(x_min, x_max, y_min, y_max, z_min, z_max, n_points, n_sampling_points)
                    elif function == "3d_spline_ur5_workspace":
                        x_min = rospy.get_param("object_" + repr(i) + "_x_min")
                        x_max = rospy.get_param("object_" + repr(i) + "_x_max")
                        y_min = rospy.get_param("object_" + repr(i) + "_y_min")
                        y_max = rospy.get_param("object_" + repr(i) + "_y_max")
                        z_min = rospy.get_param("object_" + repr(i) + "_z_min")
                        z_max = rospy.get_param("object_" + repr(i) + "_z_max")
                        n_points = rospy.get_param("object_" + repr(i) + "_n_points")
                        n_sampling_points = rospy.get_param("n_sampling_points")
                        x_trajectory, y_trajectory, z_trajectory = self.get_3d_spline_ur5_workspace(x_min, x_max, y_min, y_max, z_min, z_max, n_points, n_sampling_points)
                    elif function == "fixed_trajectory":
                        trajectory_id = rospy.get_param("object_" + repr(i) + "_trajectory_id")
                        x_trajectory, y_trajectory, z_trajectory = self.get_fixed_trajectory(trajectory_id)
                    objects_trajectories.append([x_trajectory, y_trajectory, z_trajectory])

                # Move objects 
                s = 0 
                while move: 
                    s = s % self.samples_len
                    for i in range(self.n_objects):
                        if not self.real_robot:
                            self.objects_model_state[i].pose.position.x = objects_trajectories[i][0][s]
                            self.objects_model_state[i].pose.position.y = objects_trajectories[i][1][s]
                            self.objects_model_state[i].pose.position.z = objects_trajectories[i][2][s]
                            self.set_model_state_pub.publish(self.objects_model_state[i])
                        # Publish tf of objects
                        t = TransformStamped()
                        t.header.frame_id = self.reference_frame
                        t.header.stamp = rospy.Time.now()
                        t.child_frame_id = self.objects_tf_frame[i]
                        t.transform.translation.x = objects_trajectories[i][0][s]
                        t.transform.translation.y = objects_trajectories[i][1][s]
                        t.transform.translation.z = objects_trajectories[i][2][s]
                        t.transform.rotation.x = 0.0
                        t.transform.rotation.y = 0.0
                        t.transform.rotation.z = 0.0
                        t.transform.rotation.w = 1.0
                        self.static_tf2_broadcaster.sendTransform(t)                        
                    rospy.Rate(self.update_rate).sleep()
                    s = s + 1
                # Move objects up in the air 
                self.move_objects_up()
                rospy.Rate(self.update_rate).sleep()
            else:
                pass 

if __name__ == '__main__':
    try:
        rospy.init_node('objects_controller')
        oc = ObjectsController()
        oc.objects_initialization()
        oc.objects_state_update_loop()
    except rospy.ROSInterruptException:
        pass
