#!/usr/bin/env python2
import grpc
import rospy
from concurrent import futures
from ros_bridge import UrRosBridge
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc
import logging, logging.config
import yaml
import os
class RobotServerServicer(robot_server_pb2_grpc.RobotServerServicer):
    def __init__(self, real_robot, ur_model):
        self.rosbridge = UrRosBridge(real_robot=real_robot, ur_model=ur_model)

    def GetState(self, request, context):
        try:
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

def serve():
    initialize_logger()
    logger.info('UR Robot Server...')
    server_port = rospy.get_param("~server_port")
    real_robot = rospy.get_param("~real_robot")
    ur_model = rospy.get_param("~ur_model")
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_server_pb2_grpc.add_RobotServerServicer_to_server(
        RobotServerServicer(real_robot=real_robot, ur_model=ur_model), server)
    server.add_insecure_port('[::]:'+repr(server_port))
    server.start()
    if real_robot:
        rospy.loginfo(ur_model + " Real Robot Server started at " + repr(server_port))
    else:
        rospy.loginfo(ur_model + " Sim Robot Server started at " + repr(server_port))
    rospy.spin()

def initialize_logger():
    global logger 
    
    package_path = os.path.join(os.path.dirname(__file__), '..', '..')
    with open(os.path.join(package_path, 'logging_config.yml'), 'r') as stream:
        config = yaml.safe_load(stream)
    config['handlers']['file']['filename'] = os.path.join(package_path, config['handlers']['file']['filename'] )
    logging.config.dictConfig(config)
    logger = logging.getLogger('ur_robot_server_logger')

if __name__ == '__main__':

    try:
        rospy.init_node("robot_server")
        rospy.loginfo('Waiting 10s before starting initialization robot_server')
        rospy.sleep(10)
        rospy.loginfo('Initializing robot_server node')
        serve()
    except (KeyboardInterrupt, SystemExit):
        pass
