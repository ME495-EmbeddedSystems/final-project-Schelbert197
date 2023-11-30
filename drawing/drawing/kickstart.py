import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion, Pose
from sensor_msgs.msg import JointState

from brain_interfaces.srv import BoardTiles, MovePose, Cartesian

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from enum import Enum, auto

from std_msgs.msg import String


class State(Enum):
    # will change states when drawing different components
    CALIBRATED = auto()
    WORD = auto()
    WRONG_GUESS = auto()
    STAND = auto()


class Kickstart(Node):
    def __init__(self):
        super().__init__("kickstart")

        # create kickstart service
        self.kickstart_service = self.create_service(
            Empty, 'kickstart_service', self.kickstart_callback)
        
        # timer variables
        # self.frequency = 100.0
        # self.timer_period = 1 / self.frequency
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # create mutually exclusive callback groups
        self.cal_callback_group = MutuallyExclusiveCallbackGroup()
        self.tile_callback_group = MutuallyExclusiveCallbackGroup()
        self.mp_callback_group = MutuallyExclusiveCallbackGroup()
        self.cartesian_callback_group = MutuallyExclusiveCallbackGroup()

        # create service clients
        self.cal_client = self.create_client(
            Empty, 'calibrate', callback_group=self.cal_callback_group)
        self.tile_client = self.create_client(
            BoardTiles, 'where_to_write', callback_group=self.tile_callback_group)
        self.movemp_client = self.create_client(
            MovePose, '/moveit_mp', callback_group=self.mp_callback_group)
        self.cartesian_client = self.create_client(
            Cartesian, '/cartesian_mp', callback_group=self.cartesian_callback_group)

        self.cal_state_subscriber = self.create_subscription(
            String, 'cal_state', self.cal_state_callback, 10)

        # wait for clients' services to be available
        while not self.cal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Calibrate service not available, waiting...')
        while not self.tile_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Where to Write service not available, waiting...')
        while not self.movemp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move It MP service not available, waiting...')

    def cal_state_callback(self, msg):
        self.cal_state = msg

    async def kickstart_callback(self, request, response):
        # CALIBRATE ONCE
        await self.cal_client.call_async(request=Empty.Request())

        # DASHES
        dash_x = [0.01, 0.05, 0.09, 0.09]
        dash_y = [0.0, 0.0, 0.0, 0.0]
        dash_on = [True, True, True, False]

        # WRONG DASHES
        # 1st wrong dash info
        request = BoardTiles.Request()
        request.mode = 0
        request.position = 0
        request.x = dash_x
        request.y = dash_y
        request.onboard = dash_on

        # denote pose_list and initial_pose from BoardTiles response
        resp = await self.tile_client.call_async(request)
        pose1 = resp.initial_pose
        pose_list = resp.pose_list

        self.get_logger().info(f"Pose List for Dash: {pose_list}")
    
        # FIRST TIME MOVING TO BOARD - MOVEIT_MP
        # move robot to first pose in front of dash origin - moveit_mp
        await self.movemp_client.call_async(pose1)
        # draw remaining pose dashes with Cartesian mp
        for pose in pose_list:
            self.get_logger().info(f'{pose}')
            request2 = Cartesian.Request()
            request2.target_pose = pose
            await self.cartesian_client.call_async(request2)

        return response

    # async def timer_callback(self):
    #     # calibrate once -- call Ananya's stuff
    #     await self.cal_client.call_async(request=Empty.Request())

    #     # QUEUE EACH SECTION OF GAME SETUP - INCORRECT LETTERS, CORRECT LETTERS, HANGMAN STAND

    #     # set up position for each component (list of Mode and positions)
    #     ############# list for BoardTiles of incorrect letter dashes ##############
    #     # DASH 1:
    #     if self.cal_state == "CALIBRATED":
    #         request = BoardTiles.Request()
    #         request.mode = 0
    #         request.position = 0
    #         request.x = [0.01, 0.05, 0.09, 0.09]
    #         request.y = [0.0, 0.0, 0.0, 0.0]
    #         request.onboard = [True, True, True, False]

    #         pose_list = await self.tile_client.call_async(request)

    

    #         request = Cartesian.Request()
    #         request.poses = pose_list.origin_pose

    #         self.get_logger().info(f"pose+list: {pose_list}")

    #         await self.cartesian_client.call_async(request)

        # Use moveit_mp service to convert list of Poses to robot motions - should draw each dash!
        # for pose in pose_list.origin_pose:
        #     self.get_logger().info(f'{pose}')
        #     request2 = MovePose.Request()
        #     request2.target_pose = pose
        #     await self.movemp_client.call_async(request2)

        # DASH 2:
        # request = BoardTiles()
        # request.mode = 0
        # request.position = 1
        # request.x = [0.01,0.05,0.09,0.09]
        # request.y = [0.0,0.0,0.0,0.0]
        # request.onboard = [True,True,True,False]

        # pose_list = await self.tile_client.call_async(request)
        # # Use moveit_mp service to convert list of Poses to robot motions - should draw each dash!
        # for pose in pose_list:
        #     await self.movemp_client.call_async(pose)

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


def main(args=None):
    rclpy.init(args=args)
    node = Kickstart()
    rclpy.spin(node)
    rclpy.shutdown()
