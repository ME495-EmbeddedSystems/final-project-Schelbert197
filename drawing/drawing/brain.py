import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Empty
from std_msgs.msg import Bool
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextToPath
# from brain_interfaces.msg import Cartesian
from brain_interfaces.srv import BoardTiles, MovePose, Cartesian
from gameplay_interfaces.msg import LetterMsg
# from character_interfaces.alphabet import alphabet
from geometry_msgs.msg import Pose, Point, Quaternion

from enum import Enum, auto
import numpy as np


class State(Enum):
    INITIALIZE = auto(),
    CALIBRATE = auto(),
    APPROACHING = auto(),
    WAITING = auto(),
    WRITING = auto(),
    LETTER = auto()


class Brain(Node):

    def __init__(self):
        super().__init__("Brain")

        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.create_timer(0.01, self.timer_callback, self.timer_callback_group)

        # create publishers
        self.moveit_mp_pub = self.create_publisher(
            Pose, '/moveit_mp', 10)

        # self.cartesian_mp_pub = self.create_publisher(
        #     Cartesian, '/cartesian_mp', 10)

        self.ocr_pub = self.create_publisher(
            Bool, '/ocr_run', 10)

        # create service
        self.test_service = self.create_service(
            Empty, '/test_brain', self.test_service_callback)
        
        # Create clients
        self.board_service_client = self.create_client(
            BoardTiles, '/where_to_write')  # create custom service type
        self.calibrate_service_client = self.create_client(
            Empty, 'calibrate')  # create custom service type
        self.movepose_service_client = self.create_client(
            MovePose, '/moveit_mp')  # create custom service type
        self.cartesian_mp_service_client = self.create_client(
            Cartesian, '/cartesian_mp')  # create custom service type
        self.kickstart_service_client = self.create_client(
            Empty, '/kickstart')
        # self.ocr_service = self.create_service(
        #     Empty, '/ocr_service', self.test_service_callback)

        # Create subscription from hangman.py
        self.hangman = self.create_subscription(
            LetterMsg, '/writer', callback=self.hangman_callback, qos_profile=10)
        self.home = self.create_subscription(
            Bool, '/RTH', callback=self.home_callback, qos_profile=10)

        # define global variables

        self.home_position = Pose(
            position=Point(x=-0.5, y=0.0, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )
        self.alphabet = {}
        self.scale_factor = 0.001

        self.state = State.INITIALIZE

    def create_letters(self):
        """Create the dictionary of bubble letters"""

        letters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0|-/_'
        for i in range(0, len(letters)):
            letter = letters[i]
            if letter == '0': # Head of man
                xvec=[]
                yvec=[]
                q = 25
                for t in range(0,q+1):
                    x = 50*np.cos(2*np.pi*t/q)
                    y = 50+50*np.sin(2*np.pi*t/q)
                    xvec.append(x*self.scale_factor)
                    yvec.append(y*self.scale_factor)
                point_dict = {letter: {'xlist': xvec, 'ylist': yvec}}
                self.alphabet.update(point_dict)
            elif letter == '|': # Body of man
                xlist = [0.0,0.0,0.0]
                ylist = [0.1,0.05,0.002]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            elif letter == '-': # Arms of man
                xlist = [0.05,0.1,0.15]
                ylist = [0.05,0.05,0.05]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            elif letter == '/': # Leg of man 1
                xlist = [0.1,0.075,0.05]
                ylist = [0.1,0.06,0.02]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            elif letter == '_': # Leg of man 2
                xlist = [0.0,0.025,0.05]
                ylist = [0.1,0.06,0.02]
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
            else: # All letters of alphabet
                fp = FontProperties(family="MS Gothic", style="normal")
                verts, codes = TextToPath().get_text_path(fp, letters[i])
                xlist = []
                ylist = []
                for j in range(0, len(verts) - 1):
                    # if verts[j][0] > 0: Commented out because I want to keep the 0,0 for lifting off the board
                    xlist.append(verts[j][0]*self.scale_factor)
                    ylist.append(verts[j][1]*self.scale_factor)
                point_dict = {letter: {'xlist': xlist, 'ylist': ylist}}
                self.alphabet.update(point_dict)
        

    def coords_to_poses(self, letter, tilepose: Pose):
        # get the coordiantes for the letter from the dictionary
        xcoord = self.alphabet[letter]['xlist']
        ycoord = self.alphabet[letter]['ylist']
        poses = []
        # TODO: write a for loop to create a list of Pose() from the coords
        for i in range(0, len(xcoord)):
            if xcoord[i] > 0 and ycoord[i] > 0:
                p = Point(x=tilepose.position.x,
                          y=tilepose.position.y +
                          (xcoord[i]),
                          z=tilepose.position.z + (ycoord[i]))
                quat = tilepose.orientation
                point_pose = Pose(position=p, orientation=quat)
            else:
                p = Point(x=tilepose.position.x - 0.2,
                          y=tilepose.position.y +
                          (xcoord[i]),
                          z=tilepose.position.z + (ycoord[i]))
                quat = tilepose.orientation
                point_pose = Pose(position=p, orientation=quat)
            poses.append(point_pose)

    def process_letter_points(self, letter):
        """ Function to make it easier to prepare letters for board tile type"""
        xcoord = self.alphabet[letter]['xlist']
        ycoord = self.alphabet[letter]['ylist']
        board_x = []
        board_y = []
        board_bool = []
        for i in range(0, len(xcoord)):
            if not (0.0001 > xcoord[i] > -0.0001) or not (0.0001 > ycoord[i] > 0.0001):
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

    def test_service_callback(self, request, response):

        self.state = State.LETTER

        return response

    # def board_service_callback(self, request, response):
    #     """Callback for the service to get the board tile pose"""

    def hangman_callback(self, msg: LetterMsg):
        """Callback when feedback is given from hangman"""

        # establishes a global message variable for the duration of the letter state
        self.last_message = msg

        shape_list = []
        for i in range (0,len(self.last_message.positions)):
            tile_origin = BoardTiles()
            tile_origin.mode = self.last_message.mode[i]
            tile_origin.position = self.last_message.positions[i]
            
            # get x, y, onboard values
            tile_origin.x, tile_origin.y, tile_origin.onboard = self.process_letter_points(self.last_message.letters[i])
            shape_list.append(tile_origin)

        # switches to calibrate state
        self.state = State.CALIBRATE

    def home_callback(self, msg: Bool):
        """Callback for whether or not the robot has returned to home after writing"""
        if msg == True:
            self.ocr_pub.publish(True)
            self.state = State.WRITING
        else:
            self.state = State.WAITING

    def timer_callback(self):
        if self.state == State.INITIALIZE:

            self.kick_future = self.kickstart_service_client.call(Empty)

            if self.kick_future:
                # Moves to the waiting state once the robot has reached the home position
                self.ocr_pub.publish(True)
                self.state = State.WAITING

        elif self.state == State.CALIBRATE:

            # TODO: we will need to add this client that calls the calibrate action
            self.calibrate_service_client(Empty)
            # This should send the camera calibration service as well
            # TODO: Ananya can put this to have that service call how she prefers

            # Moves to waiting state and listens for a return value to switch to letter
            self.state = State.WAITING

        elif self.state == State.APPROACHING:

            # TODO: call the board service and switch to writing when where_to_write returns
            self.movepose_service_client.call()

            # Moves to the waiting state once we are setup, and waits for something to happen from hangman.py
            self.state = State.WAITING

        elif self.state == State.LETTER:

            # This for loop will run for each thing that we need to draw based on the message sent from hangman
            # TODO We will need to consider asynchronicity, but this is the flow of info
            for j in range(0, len(self.last_message.positions)):
                # Ananya's code gives origin for the tile that we are working in reference to (taking the mode group and position in group)
                self.tile_pose: Pose = self.board_service_client.call_async(
                    self.last_message.mode, self.last_message.positions[j])

                # Coords to poses takes in the letter and the pose from the board and returns a list of poses for the trajectory
                letter_poses = self.coords_to_poses(
                    self.last_message.letter, self.tile_pose)
                start_point = Pose(
                    position=Point(x=self.tile_pose.position.x - 0.1,
                                   y=self.tile_pose.position.y, z=self.tile_pose.position.z),
                    orientation=self.tile_pose.orientation
                )  # start point given by service call TODO: Ananya, please check if this is correct for the start pos
                # I have made this start point to be 10cm behind the board facing the origin of the tile in question

                # cartesian message type packages the start point and the list of letter poses
                cartesian_msg = Cartesian(
                    poses=letter_poses, start_point=start_point)

                # publish the message, draw.py is a subscriber
                self.cartesian_mp_pub.publish(cartesian_msg)
                ###########################

            if self.last_message.positions:
                # moves to the approaching state if there are still things to be written
                tile_origin = BoardTiles()
                self.state = State.APPROACHING
            else:
                self.state = State.WAITING

        elif self.state == State.WAITING:
            # waiting state for writing actions
            pass

        elif self.state == State.WRITING:
            # waiting state for the Franka to complete the cartesian trajectory before it moves back to letter
            pass
