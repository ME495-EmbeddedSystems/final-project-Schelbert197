import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion, Pose
from sensor_msgs.msg import JointState

from path_planner.path_plan_execute import Path_Plan_Execute
from character_interfaces.alphabet import alphabet
from joint_interfaces.msg import JointTrajectories
from brain_interfaces.srv import BoardTiles, MovePose

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from enum import Enum, auto

from action_msgs.msg import GoalStatus

from brain_interfaces.srv import Cartesian
from std_msgs.msg import String, Float32

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf2_ros
from brain_interfaces.srv import MovePose, MoveJointState

class State(Enum):
    # will change states when drawing different components
    DASHES = auto()
    STAND = auto()

class Kickstart(Node):
    def __init__(self):
        super().__init__("kickstart")
        
        # create kickstart service
        self.kickstart_service = self.create_service(Empty, 'kickstart_service',self.kickstart_callback)
        
        # create mutually exclusive callback groups
        self.cal_callback_group = MutuallyExclusiveCallbackGroup()
        self.tile_callback_group = MutuallyExclusiveCallbackGroup()
        self.mp_callback_group = MutuallyExclusiveCallbackGroup()

        # create service clients
        self.cal_client = self.create_client(Empty,'calibrate_service',callback_group=self.cal_callback_group)
        self.tile_client = self.create_client(BoardTiles,'where_to_write',callback_group=self.tile_callback_group)
        self.movemp_client = self.create_client(MovePose,'/moveit_mp',callback_group=self.mp_callback_group)

        # wait for clients' services to be available
        while not self.cal_callback_group.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Calibrate service not available, waiting...')
        while not self.tile_callback_group.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Where to Write service not available, waiting...')
        while not self.mp_callback_group.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move It MP service not available, waiting...')

    async def kickstart_callback(self, request, response):
        # calibrate once -- call Ananya's stuff
        await self.cal_client.call_async()

        # QUEUE EACH SECTION OF GAME SETUP - INCORRECT LETTERS, CORRECT LETTERS, HANGMAN STAND

        # set up position for each component (list of Mode and positions)
        ############# list for BoardTiles of incorrect letter dashes ##############
        # DASH 1:
        request = BoardTiles()
        request.mode = 0
        request.position = 0
        request.x = [0.01,0.05,0.09,0.09]
        request.y = [0.0,0.0,0.0,0.0]
        request.onboard = [True,True,True,False]

        pose_list = await self.tile_client.call_async(request)
        # Use moveit_mp service to convert list of Poses to robot motions - should draw each dash!
        for pose in pose_list:
            await self.movemp_client.call_async(pose)

        # DASH 2:
        request = BoardTiles()
        request.mode = 0
        request.position = 1
        request.x = [0.01,0.05,0.09,0.09]
        request.y = [0.0,0.0,0.0,0.0]
        request.onboard = [True,True,True,False]

        pose_list = await self.tile_client.call_async(request)
        # Use moveit_mp service to convert list of Poses to robot motions - should draw each dash!
        for pose in pose_list:
            await self.movemp_client.call_async(pose)

        # DASH 3:
        # request = BoardTiles()
        # request.mode = 0
        # request.position = 2
        # request.x = [0.01,0.05,0.09,0.09]
        # request.y = [0.0,0.0,0.0,0.0]
        # request.onboard = [True,True,True,False]

        # pose_list = await self.tile_client.call_async(request)
        # # Use moveit_mp service to convert list of Poses to robot motions - should draw each dash!
        # for pose in pose_list:
        #     await self.movemp_client.call_async(pose)
        
        # convert list of Poses to Gripper pose --> use ananya's functions ## wait for ananya to do this

        # use graham's code to queue each section of components 
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Kickstart()
    rclpy.spin(node)
    rclpy.shutdown()
