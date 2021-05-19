#!/usr/bin/env python

import grpc
import rospy
from concurrent import futures
from mir100_robot_server.ros_bridge import RosBridge
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc


class RobotServerServicer(robot_server_pb2_grpc.RobotServerServicer):
    def __init__(self, real_robot):
        self.rosbridge = RosBridge(real_robot=real_robot)

    def GetState(self, request, context):
        try:
            return self.rosbridge.get_state()
        except:
            return robot_server_pb2.State(success=0)

    def SetState(self, request, context):
        try:
            s = self.rosbridge.set_state(state_msg=request)
            return robot_server_pb2.Success(success=1)
        except:
            return robot_server_pb2.Success(success=0)

    def SendAction(self, request, context):
        try:
            lin_vel, ang_vel = self.rosbridge.publish_env_cmd_vel(request.action[0], request.action[1])
            return robot_server_pb2.Success(success=1)
        except:
            return robot_server_pb2.Success(success=0)


def serve():
    server_port = rospy.get_param('~server_port')
    real_robot = rospy.get_param('~real_robot')
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_server_pb2_grpc.add_RobotServerServicer_to_server(RobotServerServicer(real_robot=real_robot), server)
    server.add_insecure_port('[::]:' + repr(server_port))
    server.start()
    if real_robot:
        rospy.loginfo('MiR 100 Real Robot Server started at ' + repr(server_port))
    else:
        rospy.loginfo('MiR 100 Sim Robot Server started at ' + repr(server_port))
    rospy.spin()


if __name__ == '__main__':

    try:
        rospy.init_node('robot_server')
        rospy.loginfo('Waiting 10s before starting initialization of robot_server')
        rospy.sleep(10)
        rospy.loginfo('Initializing robot_server node')
        serve()
    except (KeyboardInterrupt, SystemExit):
        pass
