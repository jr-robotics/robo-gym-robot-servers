#!/usr/bin/env python2
import grpc
import rospy
from concurrent import futures
from ros_bridge import PandaRosBridge
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc


class RobotServerServicer(robot_server_pb2_grpc.RobotServerServicer):

    def __init__(self, real_robot):
        # super(RobotServerServicer, self).__init__()
        self.rosbridge = PandaRosBridge(real_robot=real_robot)

    def GetState(self, request, context):
        # return super(RobotServerServicer, self).GetState(request, context)
        try:
            return self.rosbridge.get_state()
        except:
            return robot_server_pb2.State(success=0)
        pass

    def SetState(self, request, context):
        # return super(RobotServerServicer, self).SetState(request, context)
        try:
            self.rosbridge.set_state(state_msg=request)
            return _robot_server_get_success()
        except:
            return _robot_server_get_failure()

    def SendAction(self, request, context):
        # return super(RobotServerServicer, self).SendAction(request, context)
        try:
            # executed_action = self.rosbridge.publish_env_arm_cmd(request.action)
            return


def serve():
    server_port = rospy.get_param('~server_port')
    real_robot = rospy.get_param('~real_robot')

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_server_pb2_grpc.add_RobotServerServicer_to_server(
        RobotServerServicer(real_robot=real_robot), server)
    server.add_insecure_port('[::]' + repr(server_port))
    server.start()

    if real_robot:
        rospy.loginfo('Panda - Real Robot Server started at ' +
                      repr(server_port))
    else:
        rospy.loginfo('Panda - Sim Robot Server started at ' +
                      repr(server_port))
    rospy.spin()


def _robot_server_get_success():
    return robot_server_pb2.Success(success=1)


def _robot_server_get_failure():
    return robot_server_pb2.Success(success=0)


if __name__ == '__main__':

    try:
        rospy.init_node('robot_server')
        rospy.loginfo(
            'Waiting 10s before starting initialization of robot_server')
        rospy.sleep(10)
        rospy.loginfo('Initializing robot_server node')
        serve()
    except(KeyboardInterrupt, SystemExit):
        pass
