#!/usr/bin/env python
from sys import exc_info
import grpc
import rospy
from concurrent import futures
from panda.ros_bridge import PandaRosBridge
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc
import logging
import logging.config
import yaml
import os

from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

class RobotServerServicer(robot_server_pb2_grpc.RobotServerServicer):

    def __init__(self, real_robot):
        self.rosbridge = PandaRosBridge(real_robot=real_robot)

    def GetState(self, request, context):
        try:
            msg = robot_server_pb2.State()
            return self.rosbridge.get_state()
        except:
            logger.error('Failed to get state', exc_info=True)
            return robot_server_pb2.State(success=0)

    def SetState(self, request, context):
        try:
            self.rosbridge.set_state(state_msg=request)
            return robot_server_pb2.Success(success=1)
        except:
            logger.error('Failed to set state', exc_info=True)
            return robot_server_pb2.Success(success=0)

    def SendAction(self, request, context):
        try:
            executed_action = self.rosbridge.publish_env_arm_cmd(request.action)
            return robot_server_pb2.Success(success=1)
        except:
            logger.error('Failed to send action', exc_info=True)
            return robot_server_pb2.Success(success=0)
        
        
    def SendActionGetState(self, request, context):
        try:
            executed_action = self.rosbridge.publish_env_arm_cmd(request.action)
            return self.rosbridge.get_state()
        except:
            logger.error('Failed to send action and get state', exc_info=True)
            return robot_server_pb2.State(success=0)


def serve():
    logger.info('Starting Panda Robot Server...')
    server_port = rospy.get_param('~server_port')
    real_robot = rospy.get_param('~real_robot', False)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_server_pb2_grpc.add_RobotServerServicer_to_server(RobotServerServicer(real_robot=real_robot), server)
    server.add_insecure_port('[::]:' + repr(server_port))
    server.start()

    if real_robot:
        logger.info('Panda - Real Robot Server started at ' +
                      repr(server_port))
    else:
        logger.info('Panda - Sim Robot Server started at ' +
                      repr(server_port))

    rospy.spin()


def _initialize_logger():
    global logger
    
    package_path = os.path.join(os.path.dirname(__file__), '..', '..')
    with open(os.path.join(package_path, 'panda_logging_config.yml'), 'r') as stream:
        config = yaml.safe_load(stream)
    config['handlers']['file']['filename'] = os.path.join(package_path, config['handlers']['file']['filename'] )
    logging.config.dictConfig(config)
    logger = logging.getLogger('panda_robot_server_logger')


if __name__ == '__main__':
    try:
        wait_time = 5
        _initialize_logger()
        rospy.init_node('robot_server')
        logger.info('Waiting {}s before starting initialization of robot_server'.format(wait_time))
        rospy.sleep(wait_time)
        logger.info('Initializing robot_server node')
        serve()
    except(KeyboardInterrupt, SystemExit):
        pass
