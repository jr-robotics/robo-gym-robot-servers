#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import copy
import numpy as np

class JV_Estimator:
    def __init__(self):
        rospy.init_node('jv_estimator')
        # Subscriber to /joint_states
        rospy.Subscriber('joint_states', JointState, self.callbackJS, queue_size=1)
        # Publisher of /estimated_joint_states
        self.js_pub = rospy.Publisher('estimated_joint_states', JointState, queue_size=10)
        # Buffer storing previous joint positions
        self.previous_j_pos = None
        # Buffer storing previous joint velocities
        self.previous_j_vel = 0

        rospy.spin()

    def callbackJS(self,data):
        if self.previous_j_pos is None:
            self.previous_j_pos = copy.deepcopy(data.position)
            self.js_pub.publish(data)
        else:
            velocities = self.filtering_derivative(self.previous_j_pos,data.position,self.previous_j_vel,0.008,0.01)
            data.velocity = copy.deepcopy(velocities)
            self.js_pub.publish(data)
            self.previous_j_pos = copy.deepcopy(data.position)

    def filtering_derivative(self, q_t0, q_t1, previous_dq_dt,T_signal, T_filter):
        """Short summary.

        An higher time contant for the filter gives a smoothter derivative with higher delay.

        Args:
            q_t0 (type): Description of parameter `q_t0`.
            q_t1 (type): Description of parameter `q_t1`.
            T_signal (type): Description of parameter `T_signal`.
            T_filter (type): Time constant of filter.

        Returns:
            type: Description of returned object.

        """
        a = T_signal/(T_filter + T_signal)
        dq = np.subtract(np.array(q_t1),np.array(q_t0))
        dt = np.full(len(q_t0),T_signal)
        dq_dt = np.divide(dq,dt)

        filtered_dq_dt = (1-a)*np.array(previous_dq_dt) + a*dq_dt

        return filtered_dq_dt




if __name__ == '__main__':
    try:
        jv_estimator = JV_Estimator()
    except rospy.ROSInterruptException:
        pass
