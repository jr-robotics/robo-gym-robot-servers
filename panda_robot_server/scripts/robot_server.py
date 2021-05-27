#!/usr/bin/env python
import grpc
import rospy
from concurrent import futures
from panda_robot_server.ros_bridge import PandaRosBridge
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

class RobotServerServicer(robot_server_pb2_grpc.RobotServerServicer):

    def __init__(self, real_robot):
        self.rosbridge = PandaRosBridge(real_robot=real_robot)

    def GetState(self, request, context):
        try:
            return self.rosbridge.get_state()
        except:
            rospy.logerr('Failed to get state', exc_info=True)
            return robot_server_pb2.State(success=0)

    def SetState(self, request, context):
        try:
            self.rosbridge.set_state(state_msg=request)
            return robot_server_pb2.Success(success=1)
        except:
            rospy.logerr('Failed to set state', exc_info=True)
            return robot_server_pb2.Success(success=0)

    def SendAction(self, request, context):
        try:
            executed_action = self.rosbridge.publish_env_arm_cmd(request.action)
            return robot_server_pb2.Success(success=1)
        except:
            rospy.logerr('Failed to send action', exc_info=True)
            return robot_server_pb2.Success(success=0)
        
    def SendActionGetState(self, request, context):
        try:
            executed_action = self.rosbridge.publish_env_arm_cmd(request.action)
            return self.rosbridge.get_state()
        except:
            rospy.logerr('Failed to send action and get state', exc_info=True)
            return robot_server_pb2.State(success=0)


def serve():
    rospy.loginfo('Starting Panda Robot Server...')
    server_port = rospy.get_param('~server_port')
    real_robot = rospy.get_param('~real_robot', False)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_server_pb2_grpc.add_RobotServerServicer_to_server(RobotServerServicer(real_robot=real_robot), server)
    server.add_insecure_port('[::]:' + repr(server_port))
    server.start()
    if real_robot:
        rospy.loginfo('Panda - Real Robot Server started at ' +
                      repr(server_port))
    else:
        rospy.loginfo('Panda - Sim Robot Server started at ' +
                      repr(server_port))
    rospy.spin()

if __name__ == '__main__':
    try:
        wait_time = 5
        rospy.init_node('robot_server')
        rospy.loginfo('Waiting {}s before starting initialization of robot_server'.format(wait_time))
        rospy.sleep(wait_time)
        serve()
    except(KeyboardInterrupt, SystemExit):
        pass