import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from brain_interfaces.msg import Cartesian
from geometry_msgs.msg import Pose, Point, Quaternion

from enum import Enum, auto


class State(Enum):
    INITIALIZE = auto()


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

        self.home_position = Pose(
            position=Point(x=-0.5, y=0.0, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )
