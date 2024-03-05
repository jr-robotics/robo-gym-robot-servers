#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState

def mir_pose_publisher():
    rospy.init_node('mir_pose_pub')
    pub = rospy.Publisher('robot_pose', Pose, queue_size=10)
    r = rospy.Rate(10.0)
    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    while not rospy.is_shutdown():
        try:
            model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_coordinates = model_state('mir', '')
            pose = Pose()
            pose.position.x = model_coordinates.pose.position.x
            pose.position.y = model_coordinates.pose.position.y
            pose.position.z = model_coordinates.pose.position.z
            pose.orientation.x = model_coordinates.pose.orientation.x
            pose.orientation.y = model_coordinates.pose.orientation.y
            pose.orientation.z = model_coordinates.pose.orientation.z
            pose.orientation.w = model_coordinates.pose.orientation.w

        except rospy.ServiceException as e:
            print('Service call failed:' + e)
        pub.publish(pose)
        r.sleep()


if __name__ == '__main__':
    try:
        mir_pose_publisher()
    except rospy.ROSInterruptException:
        pass
