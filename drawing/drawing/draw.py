import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion, Pose
from path_planner.path_plan_execute import Path_Plan_Execute

from character_interfaces.alphabet import alphabet

from rclpy.callback_groups import ReentrantCallbackGroup
from box_adder_interfaces.srv import Box
from enum import Enum, auto

from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from action_msgs.msg import GoalStatus

from franka_msgs.action import Homing, Grasp

import numpy as np
np.set_printoptions(suppress=True)


class State(Enum):
    START = auto()
    CLOSE = auto()
    FINISH = auto()
    STOP = auto()

    LOAD_MOVES = auto()
    PLANNING = auto()
    EXECUTING = auto()
    WAITING = auto()


class Drawing(Node):
    """
    Pick up trash with the Franka.

    Drive a robot around to pickup objects that are a user specified
    distance within its workspace.

    Args:
    ----
    None

    """

    def __init__(self):
        super().__init__("Drawing")

        # declare parameters
        self.declare_parameter('x_init', 0.5)
        self.declare_parameter('y_init', 0.0)
        self.declare_parameter('robot_name', 'panda')
        self.declare_parameter('group_name', 'panda_manipulator')
        self.declare_parameter('frame_id', 'panda_link0')

        # get parameters
        self.x_init = self.get_parameter(
            'x_init').get_parameter_value().double_value
        self.y_init = self.get_parameter(
            'y_init').get_parameter_value().double_value

        self.robot_name = self.get_parameter(
            'robot_name').get_parameter_value().string_value
        self.group_name = self.get_parameter(
            'group_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value

        # Initialize variables
        self.joint_names = []
        self.joint_pos = []
        self.client_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            0.5, self.timer_callback, callback_group=self.client_cb_group)
        self.path_planner = Path_Plan_Execute(self)

        # create services
        self.pick_service = self.create_service(
            Empty, 'draw', self.pick_callback)

        self.add_box_srv = self.create_service(
            Box, "add_box", self.add_box_callback)

        self.cancel_goal = self.create_service(
            Empty, 'cancel_goal', self.cancel_goal_callback)

        box_id = 'box'
        frame_id = 'panda_link0'
        dimensions = [3.0, 2.0, 3.0]  # size
        pose = [0.0, 0.0, -1.6]
        self.path_planner.add_box(box_id, frame_id, dimensions, pose)

        self.poses = []
        self.queue = []
        # [fx, fy, fz, tx, ty, tz] alternatively [N, N, N, Nm, Nm, Nm]
        self.prev_ee_wrench = None

        self.rng = np.random.default_rng()

        self.ans = 0
        self.state = State.STOP
        self.result = False
        # being used for looping through to Open/Close gripper
        self.counter = 0

    def pick_callback(self, request, response):
        """
        Listen for a callback from the pick service.

        Listens for a callback from the pick service. When a service call
        is received, set the state to START.

        Args:
        ----
            request: The service call's request containing data.
            response: This node does not deliver a response.

        """
        self.state = State.LOAD_MOVES

        return response

    def add_box_callback(self, request, response):
        """
        Listen for a callback from the box service.

        Listens for a callback from the box service. When a service call
        is received, spawn in a box using the path_planner class's
        add_box function.

        Args:
        ----
            request: The service call's request containing data.
            response: This node does not deliver a response.

        """
        # Define box parameters
        box_id = 'box'
        frame_id = 'panda_link0'
        dimensions = [request.size_x, request.size_y, request.size_z]  # size
        pose = [request.pos_x, request.pos_y, request.pos_z]  # position

        # Add the box to the planning scene using the add_box method
        self.path_planner.add_box(box_id, frame_id, dimensions, pose)
        return response

    def cancel_goal_callback(self):
        self.path_planner.cancel_execution()

    def load_move(self, letter):
        # pose1 = Pose()
        # pose1.position = Point(x=self.x_init, y=self.y_init, z=0.05)
        # pose1.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        # pose2 = Pose()
        # pose2.position = Point(x=0.2, y=0.3, z=0.5)
        # pose2.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        # self.queue.append(pose1)
        # self.queue.append(pose2)

        for point in alphabet[letter]:
            pose = Pose()
            pose.position = Point(
                x=self.x_init, y=self.y_init+point[0], z=0.4+point[1])
            pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
            self.queue.append(pose)

    def pick_callback(self, request, response):
        """
        Listen for a callback from the pick service.

        Listens for a callback from the pick service. When a service call
        is received, set the state to START.

        Args:
        ----
            request: The service call's request containing data.
            response: This node does not deliver a response.

        """
        self.state = State.LOAD_MOVES

        return response

    async def timer_callback(self):
        """
        Timer loop for the drawing node.

        The timer loop functions as the main loop of the node, and
        also contains a state machine. If the gripper server is
        available, the state machine will contain actions for the
        gripper. If it is not available, as is the case when using
        fake hardware, the gripper will not be used.

        Args:
        ----
        None

        """
        # check whether or not the marker is running into the board, maybe
        if self.path_planner.gripper_available:
            ee_wrench = self.path_planner.calc_EE_force()
            if np.abs(ee_wrench[5] - self.prev_ee_wrench[5]) > 15:
                self.path_planner.cancel_execution()

            self.prev_ee_wrench = ee_wrench

        if self.state == State.LOAD_MOVES:
            self.load_move('a')
            self.state = State.PLANNING

        elif self.state == State.PLANNING and len(self.queue) != 0:
            current_queue_item = self.queue[0]
            # if isinstance(current_queue_item, type(Grasp())) and self.path_planner.gripper_available:
            #     self.path_planner.MoveGripper()
            # else:
            await self.path_planner.get_goal_joint_states(current_queue_item)
            await self.path_planner.plan_path()

            self.queue.pop(0)
            self.state = State.WAITING

        elif self.state == State.WAITING:
            if self.path_planner.movegroup_status == GoalStatus.STATUS_SUCCEEDED:
                self.state = State.EXECUTING
                self.path_planner.movegroup_status = GoalStatus.STATUS_UNKNOWN
            elif self.path_planner.executetrajectory_status == GoalStatus.STATUS_SUCCEEDED:
                self.state = State.PLANNING
                self.path_planner.executetrajectory_status = GoalStatus.STATUS_UNKNOWN
            elif self.path_planner.gripper_status == GoalStatus.STATUS_SUCCEEDED:
                self.state = State.PLANNING
                self.path_planner.gripper_status = GoalStatus.STATUS_UNKNOWN

        elif self.state == State.EXECUTING:
            await self.path_planner.execute_path()
            self.state = State.WAITING

        elif len(self.queue) == 0:
            self.state = State.STOP


def main(args=None):
    rclpy.init(args=args)

    drawing = Drawing()

    rclpy.spin(drawing)


if __name__ == '__main__':
    main()
