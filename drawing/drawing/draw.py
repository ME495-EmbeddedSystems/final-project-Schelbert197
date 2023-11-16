import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from path_planner.path_plan_execute import Path_Plan_Execute

from character_interfaces.alphabet import alphabet

from rclpy.callback_groups import ReentrantCallbackGroup
from box_adder_interfaces.srv import Box
from enum import Enum, auto

from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory

from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes

from franka_msgs.action import Homing, Grasp

from geometry_msgs.msg import TransformStamped

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header

import numpy as np
np.set_printoptions(suppress=True)


class State(Enum):

    TOUCHING_BOARD = auto()
    WRITING_LETTERS = auto()

    CALIBRATE = auto()
    FK = auto()

    LOAD_MOVES = auto()
    STOP = auto()
    PLANNING = auto()
    EXECUTING = auto()
    CANCELING = auto()
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
            0.01, self.timer_callback, callback_group=self.client_cb_group)
        self.path_planner = Path_Plan_Execute(self)

        # create subscriber

        self.tf = self.create_subscription(
            TransformStamped, '/tf', self.tf_callback, 10)

        # create publisher
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)

        # create services
        self.pick_service = self.create_service(
            Empty, 'draw', self.pick_callback)

        self.add_box_srv = self.create_service(
            Box, "add_box", self.add_box_callback)

        self.cancel_goal = self.create_service(
            Empty, 'cancel_goal', self.cancel_goal_callback, callback_group=self.client_cb_group)

        box_id = 'box'
        frame_id = 'panda_link0'
        dimensions = [3.0, 2.0, 3.0]  # size
        pose = [0.0, 0.0, -1.6]
        self.path_planner.add_box(box_id, frame_id, dimensions, pose)

        self.font_size = 0.1
        self.touch_board_queue = []
        self.letter_queue = []

        self.stage = State.STOP
        self.state = State.STOP

        self.L1 = 0.1070  # length of panda_link7
        self.L2 = 0.1130  # distancefrom panda_joint7 to gripper tips

        self.force_offset = 0
        self.calibration_counter = 0

        self.current_pos = Point(x=0.0, y=0.0, z=0.0)

        self.prev_state = State.STOP

        self.tf_tree = None

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

    def tf_callback(self, msg):
        self.tf_tree = msg
        self.get_logger().info(f"tf: {self.tf_tree}")

    def cancel_goal_callback(self, request, response):
        self.path_planner.cancel_execution()

        self.state = State.CANCELING
        return response

    def queue_touch_board_moves(self):
        pose1 = Pose()
        pose1.position = Point(x=-self.x_init, y=self.y_init, z=0.4)
        pose1.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)

        pose2 = Pose()
        pose2.position = Point(x=-self.x_init - 0.3, y=self.y_init, z=0.4)
        pose2.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)

        self.touch_board_queue.append(pose1)
        self.touch_board_queue.append(pose2)

    def queue_letter(self, letter):
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
                x=self.current_pos.x, y=self.current_pos.y + point[0] * self.font_size, z=self.current_pos.z + point[1] * self.font_size)
            pose.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
            self.letter_queue.append(pose)

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
        self.stage = State.TOUCHING_BOARD
        self.state = State.CALIBRATE

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
        # self.get_logger().info(f"stage: {self.stage}")
        # self.get_logger().info(f"state: {self.state}\n")

        if self.stage == State.TOUCHING_BOARD:
            if self.state == State.CALIBRATE:
                calibration_cycles = 100
                while self.calibration_counter < calibration_cycles:
                    self.force_offset += self.path_planner.current_joint_state.effort[5] / (
                        self.L1 + self.L2)
                    self.calibration_counter += 1

                self.force_offset = self.force_offset/calibration_cycles
                self.state = State.LOAD_MOVES

            elif self.state == State.LOAD_MOVES:
                self.queue_touch_board_moves()
                self.state = State.PLANNING

            elif self.state == State.PLANNING and len(self.touch_board_queue) != 0:

                await self.path_planner.get_goal_joint_states(self.touch_board_queue[0])
                self.path_planner.plan_path()

                self.state = State.WAITING

                self.touch_board_queue.pop(0)

            elif self.state == State.EXECUTING and len(self.path_planner.robot_trajectories) != 0:
                # self.path_planner.execute_path(
                #     self.path_planner.robot_trajectories[0])

                # joint_trajectory = JointTrajectory()
                # joint_trajectory.header = Header(
                #     stamp=self.get_clock().now().to_msg())
                # joint_trajectory.joint_names = self.path_planner.current_joint_state.name
                # joint_trajectory.points = [
                #     self.path_planner.robot_trajectories[0]]

                self.trajectory_pub.publish(
                    self.path_planner.robot_trajectories[0].joint_trajectory)
                self.path_planner.robot_trajectories.pop(0)

                self.state = State.EXECUTING

            elif self.state == State.EXECUTING and len(self.path_planner.robot_trajectories) == 0:
                self.state = State.STOP

            elif self.state == State.CANCELING:
                # cancel_execution_future = await self.path_planner.executetrajectory_client._cancel_goal_async(
                #     self.path_planner.executetrajectory_goal_handle)
                # cancel_goal_handle = cancel_execution_future.result()
                # self.get_logger().info(
                #     f"cancel_goal_handle: {cancel_goal_handle}")
                # if cancel_goal_handle is not None:

                #     if not cancel_goal_handle.accepted:
                #         self.node.get_logger().info('Cancel Goal Rejected :P')
                #         return

                #     get_result_future = self.cancel_goal_handle.get_result_async()

                #     if get_result_future is not None:
                #         cancel_result = get_result_future.result().result
                #         cancel_status = get_result_future.result().status
                #         self.get_logger().info(
                #             f"cancel_status: {cancel_status}")

                if self.path_planner.cancel_status == GoalStatus.STATUS_SUCCEEDED:
                    self.stage = State.WRITING_LETTERS
                    self.state = State.FK

            elif self.state == State.WAITING:

                ee_force = self.path_planner.current_joint_state.effort[5] / (
                    self.L1 + self.L2) - self.force_offset

                # self.get_logger().info(f"ee_force: {ee_force}")

                if ee_force > 3:

                    await self.path_planner.cancel_execution()
                    self.state = State.CANCELING

                elif self.path_planner.movegroup_status == GoalStatus.STATUS_SUCCEEDED:
                    # separate the planned trajectory into individual trajectories
                    self.path_planner.execute_individual_trajectories()
                    self.state = State.EXECUTING
                    self.path_planner.movegroup_status = GoalStatus.STATUS_UNKNOWN

                elif self.path_planner.executetrajectory_status == GoalStatus.STATUS_SUCCEEDED:
                    self.path_planner.robot_trajectories.pop(0)
                    if len(self.path_planner.robot_trajectories) != 0:
                        self.state = State.EXECUTING
                    else:
                        self.state = State.PLANNING
                    self.path_planner.executetrajectory_status = GoalStatus.STATUS_UNKNOWN

            elif len(self.touch_board_queue) == 0:

                self.stage = State.WRITING_LETTERS
                self.state = State.FK

        elif self.stage == State.WRITING_LETTERS:

            if self.state == State.FK:

                await self.path_planner.fk_callback()
                self.current_pos = self.path_planner.fk_pose[6].pose.position
                self.state = State.LOAD_MOVES

            elif self.state == State.LOAD_MOVES:

                self.queue_letter('a')
                self.state = State.PLANNING

            elif self.state == State.PLANNING and len(self.letter_queue) != 0:

                await self.path_planner.plan_cartesian_path(self.letter_queue)
                self.letter_queue.clear()
                self.state = State.WAITING

            elif self.state == State.EXECUTING:

                await self.path_planner.execute_path(self.path_planner.planned_trajectory)
                self.state = State.WAITING

            elif self.state == State.WAITING:

                ee_force = self.path_planner.current_joint_state.effort[5] / (
                    self.L1 + self.L2) - self.force_offset

                if ee_force > 2.5:

                    self.path_planner.cancel_execution()
                    self.state = State.CANCELING

                elif self.path_planner.fk_error_code.val == 1:

                    self.state = State.LOAD_MOVES
                    self.path_planner.fk_error_code.val = 0

                elif self.path_planner.cartesian_trajectory_error_code.val == 1:

                    self.state = State.EXECUTING
                    self.path_planner.cartesian_trajectory_error_code.val = 0

                elif self.path_planner.executetrajectory_status == GoalStatus.STATUS_SUCCEEDED:

                    self.state = State.PLANNING
                    self.path_planner.executetrajectory_status = GoalStatus.STATUS_UNKNOWN

            elif len(self.letter_queue) == 0:

                self.stage = State.STOP
                self.state = State.STOP


def main(args=None):
    rclpy.init(args=args)

    drawing = Drawing()

    rclpy.spin(drawing)


if __name__ == '__main__':
    main()
