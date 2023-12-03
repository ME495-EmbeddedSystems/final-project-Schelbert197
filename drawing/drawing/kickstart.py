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
        self.get_logger().info("calibrated")
        # DASHES
        # [0.01, 0.1, 0.15, 0.19]
        dash_x = [0.01, 0.03, 0.08,0.08, 0.14, 0.19, 0.19]
        dash_y = [0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0]  # [0.0, 0.0, 0.0, 0.0]
        # [True, True, True, False]
        dash_on = [True, True, False, True, True, True, False]

        # WRONG DASHES
        # 1st wrong dash info
        request = BoardTiles.Request()
        request.mode = 0
        request.position = 2
        request.x = dash_x
        request.y = dash_x
        request.onboard = dash_on

        # denote pose_list and initial_pose from BoardTiles response
        resp = await self.tile_client.call_async(request)
        pose1 = resp.initial_pose
        pose_list = resp.pose_list
        use_force_control = resp.use_force_control

        self.get_logger().info(f"Pose List for Dash: {pose1}")
        self.get_logger().info(f"Pose List for Dash: {pose_list}")

        # FIRST TIME MOVING TO BOARD - MOVEIT_MP
        # move robot to first pose in front of dash origin - moveit_mp
        # request2 = MovePose.Request()

        # initial y position of the board
        default_y = -0.55
        default_z = 0.34

        # pose2 = Pose()
        # pose2.position = Point(x=0.0, y=-0.55, z=)
        # pose2.orientation = Quaternion(
        #     x=0.6915744353067583, y=-0.721024315749327, z=-0.010891706465215697, w=0.04159455804102933)

        # pose_list = []
        # for i, _ in enumerate(dash_x):
        #     pose = Pose()
        #     pose.position = Point(x=dash_x[i], y=default_y, z=default_z)
        #     pose.orientation = Quaternion(
        #         x=0.6915744353067583, y=-0.721024315749327, z=-0.010891706465215697, w=0.04159455804102933)
        #     pose_list.append(pose)

        request2 = Cartesian.Request()
        request2.poses = [pose1]
        request2.velocity = 0.1
        request2.replan = False
        request2.use_force_control = [False]
        
        await self.cartesian_client.call_async(request2)
        self.get_logger().info(f"one done")

        request2 = Cartesian.Request()
        request2.poses = [pose_list[0]]
        request2.velocity = 0.015
        request2.replan = False
        request2.use_force_control = [dash_on[0]]
        await self.cartesian_client.call_async(request2)
        self.get_logger().info(f"second done")
        # draw remaining pose dashes with Cartesian mp
        request3 = Cartesian.Request()
        request3.poses = pose_list[1:]
        request3.velocity = 0.015
        request3.replan = True
        request3.use_force_control = dash_on[1:]
        self.get_logger().info(f"pose_list: {pose_list[1:]}")
        await self.cartesian_client.call_async(request3)
        self.get_logger().info(f"all done")
       

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
