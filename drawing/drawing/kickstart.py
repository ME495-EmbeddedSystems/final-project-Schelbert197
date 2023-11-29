import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion, Pose
from sensor_msgs.msg import JointState

from path_planner.path_plan_execute import Path_Plan_Execute
from character_interfaces.alphabet import alphabet
from joint_interfaces.msg import JointTrajectories

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
        self.kickstart_service = self.create_service(Empty, '')
        
        


        ##### PUT THIS IN THE KICKSTART SERVICE CALLBACK
        # calibrate once -- call Ananya's stuff

        # set up position for each component (list of Mode and positions)

        # with the list of points, create trajectories --> list of Poses (graham's code)

        # convert list of Poses to Gripper pose --> use ananya's functions ## wait for ananya to do this

        # use graham's code to queue each component and draw (draw.py)
        

