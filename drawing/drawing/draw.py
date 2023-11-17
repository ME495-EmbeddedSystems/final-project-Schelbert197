import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion, Pose
from path_planner.path_plan_execute import Path_Plan_Execute

from character_interfaces.alphabet import alphabet
from joint_interfaces.msg import JointTrajectories

from rclpy.callback_groups import ReentrantCallbackGroup
from box_adder_interfaces.srv import Box
from enum import Enum, auto

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import TransformStamped

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header, String

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
            0.5, self.timer_callback, callback_group=self.client_cb_group)
        self.path_planner = Path_Plan_Execute(self)

        # create subscriber

        self.tf = self.create_subscription(
            TransformStamped, '/tf', self.tf_callback, 10)

        self.chatting_sub = self.create_subscription(
            String, '/chatting', self.chatting_callback, 10)

        # create publisher
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)

        self.joint_traj_pub = self.create_publisher(
            JointTrajectories, '/joint_trajectories', 10)

        # create services
        self.pick_service = self.create_service(
            Empty, 'draw', self.pick_callback)

        self.font_size = 0.1
        self.touch_board_queue = []
        self.letter_queue = []

        self.stage = State.STOP
        self.state = State.STOP

        self.L1 = 0.1070  # length of panda_link7
        self.L2 = 0.1130  # distancefrom panda_joint7 to gripper tips

        self.force_offset = 0
        self.force_threshold = 3  # N
        self.calibration_counter = 0

        self.current_pos = Point(x=0.0, y=0.0, z=0.0)

        self.prev_state = State.STOP

        self.tf_tree = None

    def tf_callback(self, msg):
        self.tf_tree = msg.transforms
        # self.get_logger().info(f"tf: {self.tf_tree}")

    def chatting_callback(self, msg):
        if msg.data == "done":
            self.state = State.PLANNING

        self.get_logger().info("fhdsa;lfjdls;aksalk;fjsdal;jf")

    def queue_touch_board_moves(self):
        pose1 = Pose()
        pose1.position = Point(x=-self.x_init, y=self.y_init, z=0.4)
        pose1.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)

        pose2 = Pose()
        pose2.position = Point(x=-self.x_init - 0.2, y=self.y_init, z=0.4)
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
        self.get_logger().info(f"stage: {self.stage}")
        self.get_logger().info(f"state: {self.state}\n")

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

            elif self.state == State.EXECUTING:

                joint_trajectories = JointTrajectories()
                joint_trajectories.clear = False
                joint_trajectories.state = "publish"

                ee_force = self.path_planner.current_joint_state.effort[5] / (
                    self.L1 + self.L2) - self.force_offset

                if ee_force > self.force_threshold:
                    # self.path_planner.joint_trajectories.clear
                    joint_trajectories.clear = True
                    joint_trajectories.state = "stop"

                joint_trajectories.joint_trajectories = self.path_planner.joint_trajectories

                self.joint_traj_pub.publish(joint_trajectories)

                self.state = State.WAITING

                # for i, _ in enumerate(self.path_planner.current_joint_state.position[:7]):
                #     go_ahead = True
                #     if self.path_planner.current_joint_state.position[i] < self.path_planner.joint_trajectories[0].points[0].positions[i] - 0.01 \
                #             or self.path_planner.current_joint_state.position[i] > self.path_planner.joint_trajectories[0].points[0].positions[i] + 0.01:
                #         go_ahead = False

                # self.get_logger().info(f"go_ahead = {go_ahead}")

                # if go_ahead:
                # self.path_planner.joint_trajectories[0].header = Header(
                #     stamp=self.get_clock().now().to_msg())
                # self.trajectory_pub.publish(
                #     self.path_planner.joint_trajectories[0])
                # self.path_planner.joint_trajectories.pop(0)
                # else:
                # self.path_planner.joint_trajectories[0].header = Header(
                #     stamp=self.get_clock().now().to_msg())
                # self.trajectory_pub.publish(
                #     self.path_planner.joint_trajectories[0])

            elif self.state == State.WAITING:

                if self.path_planner.movegroup_status == GoalStatus.STATUS_SUCCEEDED:
                    # separate the planned trajectory into individual trajectories
                    self.path_planner.execute_individual_trajectories()
                    self.state = State.EXECUTING

                    self.path_planner.movegroup_status = GoalStatus.STATUS_UNKNOWN

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
                self.letter_queue.pop(0)
                self.state = State.WAITING

            elif self.state == State.EXECUTING:

                await self.path_planner.execute_path()
                self.state = State.WAITING

            elif self.state == State.WAITING:

                if self.path_planner.fk_error_code.val == 1:

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
