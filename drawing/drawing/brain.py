import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Empty

from brain_interfaces.msg import Cartesian
from character_interfaces.alphabet import alphabet
from geometry_msgs.msg import Pose, Point, Quaternion

from enum import Enum, auto


class State(Enum):
    INITIALIZE = auto()
    WAITING = auto()
    LETTER = auto()


class Brain(Node):

    def __init__(self):
        super().__init__("Brain")

        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.create_timer(0.01, self.timer_callback, self.timer_callback_group)

        # create publishers

        self.moveit_mp_pub = self.create_publisher(
            Pose, '/moveit_mp', 10)

        self.cartesian_mp_pub = self.create_publisher(
            Cartesian, '/cartesian_mp', 10)

        # create services

        self.test_service = self.create_service(
            Empty, '/test_brain', self.test_service_callback)

        # define global variables

        self.home_position = Pose(
            position=Point(x=-0.5, y=0.0, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

        self.state = State.INITIALIZE

    def coords_to_poses(self, char):
        # get the coordiantes for the letter from the dictionary
        coords = alphabet(char)
        # TODO: write a for loop to create a list of Pose() from the coords

    def test_service_callback(self, request, response):

        self.state = State.LETTER

        return response

    def timer_callback(self):
        if self.state == State.INITIALIZE:

            # publish the message, draw.py is a subscriber
            self.moveit_mp_pub.publish(self.home_position)

            # its possible this message is sent too fast, and that draw.py
            # doesn't receive it, just keep in mind.

        elif self.state == State.LETTER:
            # TODO: write a function coords_to_poses() to transform the list of
            # y and z coordinates we have for writing letters into a list of poses
            # that we can send on the /cartesian_mp topic

            letter_poses = []  # instead of empty list, = coords_to_poses()
            start_point = Pose(
                position=Point(x=0.0, y=0.0, z=0.4),
                orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
            )

            cartesian_msg = Cartesian(
                poses=letter_poses, start_point=start_point)

            # publish the message, draw.py is a subscriber
            self.cartesian_mp_pub.publish(cartesian_msg)

        elif self.state == State.WAITING:
            pass
