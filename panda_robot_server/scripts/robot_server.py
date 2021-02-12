#!/usr/bin/env python2
from sys import exc_info
import grpc
import rospy
from concurrent import futures
from ros_bridge import PandaRosBridge
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc
import logging
import logging.config
import yaml
import os


class RobotServerServicer(robot_server_pb2_grpc.RobotServerServicer):

    def __init__(self, real_robot):
        self.rosbridge = PandaRosBridge(real_robot=real_robot)

    def GetState(self, request, context):
        try:
            return self.rosbridge.get_state()
        except:
            # logger.error('Failed to get state', exc_info=True)
            return robot_server_pb2.State(success=0)

    def SetState(self, request, context):
        try:
            self.rosbridge.set_state(state_msg=request)
            return self._robot_server_get_success()
        except:
            # logger.error('Failed to set state', exc_info=True)
            return self._robot_server_get_failure()

    def SendAction(self, request, context):
        try:
            # executed_action = self.rosbridge.publish_env_arm_cmd(request.action)
            return
        except:
            # logger.error('Failed to send action', exc_info=True)
            return self._robot_server_get_failure()

    def _robot_server_get_success(self):
        return robot_server_pb2.Success(success=1)

    def _robot_server_get_failure(self):
        return robot_server_pb2.Success(success=0)


def serve():
    # _initialize_logger()
    # logger.info('Starting Panda Robot Server...')
    rospy.loginfo('Starting Panda Robot Server...')
    server_port = rospy.get_param('~server_port')
    real_robot = rospy.get_param('~real_robot', False)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_server_pb2_grpc.add_RobotServerServicer_to_server(
        RobotServerServicer(real_robot=real_robot), server)
    server.add_insecure_port('[::]:' + repr(server_port))
    server.start()

    if real_robot:
        rospy.loginfo('Panda - Real Robot Server started at ' +
                      repr(server_port))
    else:
        rospy.loginfo('Panda - Sim Robot Server started at ' +
                      repr(server_port))
    rospy.spin()


def _initialize_logger():
    global logger
    handlers_tag = 'handlers'
    file_tag = 'file'
    filename_tag = 'filename'

    package_path = os.path.join(os.path.dirname(__file__), '..', '..')
    with open(os.path.join(package_path, 'logging_config.yml'), 'r') as stream:
        config = yaml.safe_load(stream)
    config[handlers_tag][file_tag][filename_tag] = os.path.join(
        package_path, config[handlers_tag][file_tag][filename_tag])
    logging.config.dictConfig(config)
    logger = logging.getLogger('panda_robot_server_logger')


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
