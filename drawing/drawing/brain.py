"""
The brain node.

Interfaces with all other nodes to evaulate data.

PUBLISHERS:
  + /ocr_run (Bool) - The message to initiate the OCR pipeline.

SUBSCRIBERS:
  + /writer (LetterMsg) - The data sent from hangman for a given play.

CLIENTS:
  + /where_to_write (BoardTiles) - The data sent to retrieve a tile pose.
  + /moveit_mp (MovePose) - The data to move to specific pose.
  + /cartesian_mp (Cartesian) - The data sent for a cartesian move.
  + /kickstart_service (Empty) - The data sent to initialize the board.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextToPath
from brain_interfaces.srv import BoardTiles, MovePose, Cartesian
from brain_interfaces.msg import LetterMsg
from geometry_msgs.msg import Pose, Point, Quaternion

from enum import Enum, auto
import numpy as np


class State(Enum):
    """
    The state class.

    Create the states of the node to determine what the timer
    fcn should be doing (PLAYING, WAITING, OR GAME_OVER).
    """

    INITIALIZE = auto(),
    WAITING = auto(),
    LETTER = auto()


class Brain(Node):
    """Controls the nodes required to play Hangman."""

    def __init__(self):
        super().__init__("brain")

        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.create_timer(0.01, self.timer_callback, self.timer_callback_group)

        # Create publlishers
        self.ocr_pub = self.create_publisher(
            Bool, '/ocr_run', 10)

        # Callback groups
        self.tile_cb_group = MutuallyExclusiveCallbackGroup()
        self.mp_callback_group = MutuallyExclusiveCallbackGroup()
        self.cartesian_cb_group = MutuallyExclusiveCallbackGroup()
        self.kick_cb_group = MutuallyExclusiveCallbackGroup()

        # Create clients
        self.board_service_client = self.create_client(
            BoardTiles, '/where_to_write', callback_group=self.tile_cb_group)
        self.movepose_client = self.create_client(
            MovePose, '/moveit_mp', callback_group=self.mp_callback_group)
        self.cartesian_mp_client = self.create_client(
            Cartesian, '/cartesian_mp', callback_group=self.cartesian_cb_group)
        self.kickstart_client = self.create_client(
            Empty, '/kickstart_service', callback_group=self.kick_cb_group)

        while not self.board_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Board service not available, waiting...')
        while not self.movepose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveIt MP service unavailable, waiting...')
        while not self.cartesian_mp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Carisian service unavailable, waiting...')
        while not self.kickstart_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kickstart service unavailable, waiting...')

        # Create subscription from hangman.py
        self.hangman = self.create_subscription(
            LetterMsg, '/writer',
            callback=self.hangman_callback, qos_profile=10)

        # define global variables
        self.home_position = Pose(
            position=Point(x=-0.5, y=0.0, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )
        self.alphabet = {}
        self.board_scale = 1.0
        self.scale_factor = 0.001 * self.board_scale
        self.shape_list = []
        self.current_mp_pose = Pose()
        self.current_traj_poses = []
        self.current_shape_poses = []
        self.kick_future = None
        self.calibrate_future = None
        self.board_future = None

        self.state = State.INITIALIZE
        self.create_letters()

    def create_letters(self):
        """Create the dictionary of bubble letters."""
        letters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0|-/_'
        for i in range(0, len(letters)):
            letter = letters[i]
            if letter == '0':  # Head of man
                xvec = []
                yvec = []
                q = 25
                for t in range(0, q+1):
                    x = 35*np.cos(2*np.pi*t/q)
                    y = 35+35*np.sin(2*np.pi*t/q)
                    xvec.append(x*self.scale_factor * self.board_scale)
                    yvec.append(y*self.scale_factor * self.board_scale)
                point_dict = {letter: {'xlist': xvec, 'ylist': yvec}}
                self.alphabet.update(point_dict)
            elif letter == '|':  # Body of man
                xlist = [0.0, 0.0, 0.0]
                ylist = [0.1 * self.board_scale, 0.05 *
                         self.board_scale, 0.002 * self.board_scale]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            elif letter == '-':  # Arms of man
                xlist = [0.05 * self.board_scale, 0.1 *
                         self.board_scale, 0.15 * self.board_scale]
                ylist = [0.05 * self.board_scale, 0.05 *
                         self.board_scale, 0.05 * self.board_scale]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            elif letter == '/':  # Leg of man 1
                xlist = [0.1 * self.board_scale, 0.075 *
                         self.board_scale, 0.05 * self.board_scale]
                ylist = [0.1 * self.board_scale, 0.06 *
                         self.board_scale, 0.02 * self.board_scale]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            elif letter == '_':  # Leg of man 2
                xlist = [0.0 * self.board_scale, 0.025 *
                         self.board_scale, 0.05 * self.board_scale]
                ylist = [0.1 * self.board_scale, 0.06 *
                         self.board_scale, 0.02 * self.board_scale]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            else:  # All letters of alphabet
                fp = FontProperties(
                    family="Liberation Sans Narrow", style="normal")
                verts, codes = TextToPath().get_text_path(fp, letters[i])
                xlist = []
                ylist = []
                for j in range(0, len(verts) - 1):
                    if verts[j][0] > 0:
                        xlist.append(
                            verts[j][0] * self.scale_factor * self.board_scale)
                        ylist.append(
                            verts[j][1] * self.scale_factor * self.board_scale)
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)

    def process_letter_points(self, letter):
        """
        Prepare the letter points.

        Function to make it easier to prepare letters for board tile type.
        letter (String) : The character to be written on the board

        Returns
        -------
        board_x (List) : The list of x points on the board plane
        board_y (List) : The list of y points on the board plane
        board_bool (List) : The list of boolean values on the board

        """
        xcoord = self.alphabet[letter]['xlist']
        ycoord = self.alphabet[letter]['ylist']
        board_x = []
        board_y = []
        board_bool = []
        for i in range(0, len(xcoord)):
            if not (0.0001 > xcoord[i] > -0.0001) \
                    or not (0.0001 > ycoord[i] > 0.0001):
                board_x.append(xcoord[i])
                board_y.append(ycoord[i])
                board_bool.append(True)
            elif i != len(xcoord):
                board_x.append(xcoord[i+1])
                board_y.append(ycoord[i+1])
                board_bool.append(False)
            else:
                board_x.append(xcoord[i])
                board_y.append(ycoord[i])
                board_bool.append(False)
        return board_x, board_y, board_bool

    def hangman_callback(self, msg: LetterMsg):
        """
        Call back when feedback is given from hangman.

        Args:
        ----
        msg (LetterMsg) : The character with postion to be written on the board

        """
        # establishes a global message variable for the duration of LETTER
        self.last_message = msg
        self.ocr_pub.publish(Bool(data=False))

        # Turns off the OCR pipeline
        self.ocr_pub.publish(Bool(data=False))

        self.shape_list = []
        for i in range(0, len(self.last_message.positions)):
            tile_origin = BoardTiles.Request()
            tile_origin.mode = self.last_message.mode[i]
            tile_origin.position = self.last_message.positions[i]

            # get x, y, onboard values
            tile_origin.x, tile_origin.y, tile_origin.onboard = \
                self.process_letter_points(self.last_message.letters[i])
            self.shape_list.append(tile_origin)

        # switches to calibrate state
        self.state = State.LETTER

    async def letter_writer(self, shape: BoardTiles.Request()):
        """
        Write the letters on the board.

        Function to process the shape into trajectory service calls.

        Args:
        ----
        shape (BoardTiles.Request()) : The poses in BoardTiles form

        """
        resp = await self.board_service_client.call_async(shape)
        pose1 = resp.initial_pose
        pose_list = resp.pose_list

        request2 = MovePose.Request()
        request2.target_pose = pose1
        request2.use_force_control = False
        await self.movepose_client.call_async(request2)

        request2 = Cartesian.Request()
        request2.poses = [pose_list[0]]
        request2.velocity = 0.015
        request2.replan = False
        request2.use_force_control = [shape.onboard[0]]
        await self.cartesian_mp_client.call_async(request2)

        request3 = Cartesian.Request()
        request3.poses = pose_list[1:]
        request3.velocity = 0.015
        request3.replan = True
        request3.use_force_control = shape.onboard[1:]
        await self.cartesian_mp_client.call_async(request3)

        self.shape_list.pop(0)

    async def timer_callback(self):
        """Timer running at a specified frequency."""
        if self.state == State.INITIALIZE:
            # Initializes the kickstart feature then waits for completion
            await self.kickstart_client.call_async(request=Empty.Request())
            goal_js = MovePose.Request()
            goal_js.target_pose.position = Point(
                x=0.545029890155533, y=0.05943234468738731, z=0.58935441642377)
            goal_js.target_pose.orientation = Quaternion(x=-0.4857648070976754,
                                                         y=-0.5175973920275,
                                                         z=-0.4696623291331898,
                                                         w=0.5249216975367619)
            goal_js.use_force_control = False
            self.get_logger().info('before moved')
            await self.movepose_client.call_async(goal_js)
            # Turns on OCR because the play has ended and returns to WAITING
            self.ocr_pub.publish(Bool(data=True))
            self.state = State.WAITING

        elif self.state == State.LETTER:
            if self.shape_list:
                # Creates the correct data type for the next shape
                await self.letter_writer(self.shape_list[0])

            else:
                request4 = Cartesian.Request()
                request4.poses = [Pose(
                    position=Point(
                        x=0.30744234834406486,
                        y=-0.17674628233240325,
                        z=0.5725350884705022),
                    orientation=Quaternion(
                        x=0.7117299678289105,
                        y=-0.5285053338340909,
                        z=0.268057323473255,
                        w=0.37718408812611504,))]
                request4.velocity = 0.1
                request4.replan = False
                request4.use_force_control = [False]
                await self.cartesian_mp_client.call_async(request4)

                goal_js = MovePose.Request()
                goal_js.target_pose.position = Point(
                    x=0.545029890155533, y=0.0594323446873873, z=0.58935441642)
                goal_js.target_pose.orientation = Quaternion(
                    x=-0.48576480709767544, y=-0.5175973920275,
                    z=-0.4696623291331898, w=0.5249216975367619)
                goal_js.use_force_control = False
                await self.movepose_client.call_async(goal_js)
                # Turns on the OCR when play has ended and returns to WAITING
                self.ocr_pub.publish(Bool(data=True))
                self.state = State.WAITING

        elif self.state == State.WAITING:
            pass


def main(args=None):
    """Node's entry point."""
    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
