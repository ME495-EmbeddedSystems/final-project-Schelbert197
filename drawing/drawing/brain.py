import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Empty
from std_msgs.msg import Bool, String
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

        self.state_pub = self.create_publisher(
            String, '/brain_states', 10) # maybe use this to publish the states

        self.ocr_pub = self.create_publisher(
            Bool, '/ocr_run', 10)
        
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

        # Create subscription from hangman.py
        self.hangman = self.create_subscription(
            LetterMsg, '/writer', callback=self.hangman_callback, qos_profile=10)
        self.home = self.create_subscription(
            Bool, '/RTH', callback=self.home_callback, qos_profile=10)
        self.trajectory_status = self.create_subscription(
            String, '/execute_trajectory_status',callback=self.trajectory_status_callback, qos_profile=10)

        # define global variables

        self.home_position = Pose(
            position=Point(x=-0.5, y=0.0, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )
        self.alphabet = {}
        self.scale_factor = 0.001
        self.shape_list = []
        self.current_mp_pose = Pose()
        self.current_traj_poses = []
        self.current_shape_poses = []
        self.kick_future = None
        self.calibrate_future = None
        self.board_future = None

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

    def trajectory_status_callback(self, msg: String):
        """Callback for the service to get execute the drawing on the board"""
        new_msg = msg
        if new_msg == 'done':
            # Remove the first instance in the shape list since it was just executed
            self.shape_list.pop[0]
            # Return to letter writing to see if more things need to be written
            self.state = State.LETTER
        else:
            self.get_logger().error("An error occured and the trajectory did not return done.")

    def hangman_callback(self, msg: LetterMsg):
        """Callback when feedback is given from hangman"""

        # establishes a global message variable for the duration of the letter state
        self.last_message = msg

        # Turns off the OCR pipeline
        self.ocr_pub.publish(False)

        self.shape_list = []
        for i in range (0,len(self.last_message.positions)):
            tile_origin = BoardTiles()
            tile_origin.mode = self.last_message.mode[i]
            tile_origin.position = self.last_message.positions[i]
            
            # get x, y, onboard values
            tile_origin.x, tile_origin.y, tile_origin.onboard = self.process_letter_points(self.last_message.letters[i])
            self.shape_list.append(tile_origin)

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
            # Initializes the kickstart feature then waits for completion
            self.kick_future = self.kickstart_service_client.call_async(Empty)
            self.state = State.WAITING

        elif self.state == State.CALIBRATE:
            # Starts calibration then moves to waiting
            self.calibrate_future = self.calibrate_service_client.call_async(Empty)
            self.state = State.WAITING

        elif self.state == State.APPROACHING:
            # Calls the service for the approach pose then moves to writing state
            self.movepose_future = self.movepose_service_client.call_async(self.current_mp_pose)
            self.state = State.WRITING

        elif self.state == State.LETTER:
            if self.shape_list:
                # moves to the approaching state if there are still things to be written
                self.board_future = self.board_service_client.call_async(self.shape_list[0])
                self.state = State.WAITING
            else:
                # Turns on the OCR because the play has ended and returns to WAITING
                self.ocr_pub.publish(True)
                self.state = State.WAITING

        elif self.state == State.WAITING:
            # waiting state for writing actions
            if self.kick_future:
                # Turns on OCR when kickstart finishes and waits for hangman callback
                self.ocr_pub.publish(True)
                self.kick_future = None
            elif self.calibrate_future:
                # Listens for a return value from calibration to switch to LETTER
                self.state = State.LETTER
                self.calibrate_future = None
            elif self.board_future:
                # Looks that board has returned values
                # Assigns poses for approach and cartesian then moves to APPROACHING
                self.current_mp_pose = self.board_future.initial_pose
                self.current_traj_poses = self.board_future.pose_list
                self.board_future = None
                self.state = State.APPROACHING
            else:
                # If nothing has returned from client call, WAITING passes
                pass

        elif self.state == State.WRITING:
            # waiting state for the Franka to complete the mp and cartesian trajectories
            # in 2 steps before it moves back to LETTER
            if self.movepose_future:
                self.cartesian_mp_service_client.call_async(self.current_traj_poses)
                self.movepose_future = None
            else:
                pass
            # Node will only leave this state once trajectory_status returns 'done'


def main(args=None):
    """ The node's entry point """
    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
