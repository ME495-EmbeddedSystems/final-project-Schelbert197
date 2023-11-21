import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion, Pose
from path_planner.path_plan_execute import Path_Plan_Execute

from character_interfaces.alphabet import alphabet
from joint_interfaces.msg import JointTrajectories

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from box_adder_interfaces.srv import Box
from enum import Enum, auto

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import TransformStamped

from trajectory_msgs.msg import JointTrajectory
from brain_interfaces.msg import Cartesian
from std_msgs.msg import Header, String, Float32

from moveit_msgs.srv import GetCartesianPath

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf2_ros

import numpy as np
np.set_printoptions(suppress=True)


class State(Enum):

    TOUCHING_BOARD = auto()
    WRITING_LETTERS = auto()
    CALIBRATE = auto()
    GET_TRANSFORM = auto()
    LOAD_MOVES = auto()
    STOP = auto()
    PLANNING = auto()
    EXECUTING = auto()
    WAITING = auto()

    PLAN_BIG_MOVE = auto()
    PLAN_CARTESIAN_MOVE = auto()


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
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.chatting_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            0.01, self.timer_callback, callback_group=self.timer_callback_group)

        self.path_planner = Path_Plan_Execute(self)

        # these are used for computing the current location of the end-effector
        # using the tf tree.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        ############# create subscribers ################

        # this subscriber is for the brain node to send singular poses for
        # this node to plan paths to using the moveit motion planner
        self.moveit_mp_sub = self.create_subscription(
            Cartesian, '/moveit_mp', self.moveit_mp_callback, 10)

        # this subscriber is for the brain node to send lists of poses
        # for this node to use to plan paths using the cartesian motion
        # planner. It also sends a Point() object, which contains the
        # start position of the letter to be planned
        self.cartesian_mp_sub = self.create_subscription(
            Pose, '/cartesian_mp', self.cartesian_mp_callback, 10)

        # this subscriber is used for communicating with the node we created
        # to execute our trajectories.
        self.chatting_sub = self.create_subscription(
            String, '/chatting', self.chatting_callback, 10, callback_group=self.chatting_callback_group)

        ############# create publishers ##############

        # this publisher is used to send the joint trajectories we plan to our
        # node that we created to execute them.
        self.joint_traj_pub = self.create_publisher(
            JointTrajectories, '/joint_trajectories', 10)

        # this publisher is used to send the current force at the end-effector
        # to the node we created to execute trajectories.
        self.force_pub = self.create_publisher(
            Float32, '/ee_force', 10)

        self.font_size = 0.1
        self.big_move_queue = []
        self.cartesian_move_queue = []

        self.moveit_mp_queue = []  # moveit motion planner queue
        self.cartesian_mp_queue = []  # cartesian motion planner queue
        self.letter_start_point = []

        self.state = State.CALIBRATE

        self.L1 = 0.1070  # length of panda_link7
        self.L2 = 0.1130  # distancefrom panda_joint7 to gripper tips

        self.force_offset = 0
        self.force_threshold = 3  # N
        self.calibration_counter = 0

        self.current_pos = Point(x=0.0, y=0.0, z=0.0)
        self.letter_start_pos = Point(x=0.0, y=0.0, z=0.0)

        self.home_position = Pose(
            position=Point(x=-0.5, y=0.0, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

        self.prev_state = State.STOP

    def chatting_callback(self, msg):

        # the "done" message signifies that the trajectory execution node has finished
        # executing the trajectory it was assigned. Once this happens, we should go back
        # to the planning state, and from there if there's nothing in the queue the state
        # will change to waiting.

        if msg.data == "done":
            trans, rotation = self.get_transform(
                'panda_link0', 'panda_hand_tcp')
            self.letter_start_pos = Point(x=trans[0], y=trans[1], z=trans[2])
            self.state = State.PLAN_BIG_MOVE

    def moveit_mp_callback(self, msg):
        self.moveit_mp_queue.append(msg)

    def cartesian_mp_callback(self, msg):
        '''
        Queue a letter to be drawn.

        This function will be called when the brain node sends
        this node a message with a cartesian path to plan. I had an
        idea that the brain can just send this node a list of poses
        that would need to traveled to for drawing a letter, and that
        this node could add in some in-between movements that help guide
        the robot to the correct position on the board. We can discuss this.

        Args:
        ----
        msg: the custom message (brain_interfaces/Cartesian.msg)
        '''

        self.letter_start_point.y = msg.start_point.y
        self.letter_start_point.z = msg.start_point.z

        # queue a move so that the end-effector moves laterally
        # in front of the position on the whiteboard we'd like to
        # start drawing the letter. This is very important for
        # maintaining accuracy.
        self.cartesian_mp_queue.append([Pose(
            position=Point(x=self.home_position.position.x,
                           y=self.letter_start_point.position.y,
                           z=self.letter_start_point.position.z)
        )])

        # queue a move to go and touch the board. I picked
        # 0.5m because it's just a big number. All we need
        # is to place this point somewhere behind the white-
        # board.
        self.cartesian_mp_queue.append([Pose(
            position=Point(x=self.home_position.position.x + 0.5,
                           y=self.letter_start_point.position.y,
                           z=self.letter_start_point.position.z)
        )])

        self.cartesian_mp_queue.append(msg.poses)

    def get_transform(self, parent_frame, child_frame):
        """
        Try catch block for listening to transforms between parent and child frame.

        Args:
        ----
            parent_frame (string): name of parent frame
            child_frame (string): name of child frame

        Returns
        -------
            brick_to_platform: the x,y,z of the translational transform

        """
        try:
            trans = self.buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            transl = trans.transform.translation
            rot = trans.transform.rotation
            brick_to_platform = [transl.x, transl.y, transl.z]
            rotation = [rot.x, rot.y, rot.z, rot.w]

            # print(brick_to_platform[2])
            return brick_to_platform, rotation

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f"Lookup exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]

    def queue_touch_board_moves(self):
        pose1 = Pose()
        pose1.position = Point(x=-self.x_init, y=self.y_init, z=0.4)
        pose1.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        pose2 = Pose()
        pose2.position = Point(x=-self.x_init - 0.3, y=self.y_init, z=0.4)
        pose2.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        self.big_move_queue.append(pose1)
        self.cartesian_move_queue.append(pose2)

    def queue_letter(self, letter):

        for point in alphabet[letter]:
            pose = Pose()
            pose.position = Point(
                x=self.current_pos[0], y=self.current_pos[1] + point[0] * self.font_size, z=self.current_pos[2] + point[1] * self.font_size)
            pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
            self.cartesian_move_queue.append(pose)

        move_back = Pose()
        move_back.position = Point(
            x=self.current_pos[0]+0.05, y=self.current_pos[1], z=self.current_pos[2])
        move_back.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        self.cartesian_move_queue.append(move_back)

        move_side = Pose()
        move_side.position = Point(
            x=self.current_pos[0]+0.05, y=self.current_pos[1] + 0.01, z=self.current_pos[2])
        move_side.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        self.cartesian_move_queue.append(move_side)

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
        if self.state == State.CALIBRATE:

            # here we figure out what the force offset should be by using an average.
            # we take 100 readings of the effort in panda_joint6, and take the average
            # to assign the force offset in the joint due to gravity.

            calibration_cycles = 100
            while self.calibration_counter < calibration_cycles:
                self.force_offset += self.path_planner.current_joint_state.effort[5] / (
                    self.L1 + self.L2)
                self.calibration_counter += 1

            self.force_offset = self.force_offset/calibration_cycles
            self.state = State.WAITING

        elif self.state == State.PLAN_BIG_MOVE:

            # here we check to see if the big_move queue is empty, and if not,
            # we use the moveit motion planner to create a trajectory.
            # then we go to the waiting loop, where we will wait for the future
            # to return true.

            if not self.big_move_queue:  # check if the queue is empty
                self.state == State.PLAN_CARTESIAN_MOVE
                return

            await self.path_planner.get_goal_joint_states(self.big_move_queue[0])
            self.path_planner.plan_path()

            self.state = State.WAITING

            self.big_move_queue.pop(0)

        elif self.state == State.PLAN_CARTESIAN_MOVE:

            # check to see if the cartesian move queue is empty, and if not
            # then plan a cartesian path using the poses in the queue. The
            # /compute_cartesian_path service takes in a list of poses, and
            # creates a trajectory to visit all of those poses.

            if not self.cartesian_move_queue:
                self.state == State.WAITING

            await self.path_planner.plan_cartesian_path(self.cartesian_move_queue)

            self.cartesian_move_queue.clear()
            self.state = State.EXECUTING

        elif self.state == State.EXECUTING:

            # send the trajectory previously planned, either by the moveit motion
            # planner or the cartesian path planner, to our node for executing trajectories.

            joint_trajectories = JointTrajectories()
            joint_trajectories.clear = False
            joint_trajectories.state = "publish"

            joint_trajectories.joint_trajectories = self.path_planner.execute_individual_trajectories()

            self.joint_traj_pub.publish(joint_trajectories)

            self.state = State.WAITING

        elif self.state == State.WAITING:

            # calculate the current force at the end-effector, and send it to the
            # node that is executing our trajectory. Also, check to see whether
            # the moveit motion planner has completed planning. This will only
            # happen if the state prior was State.PLAN_BIG_MOVE.

            ee_force = self.path_planner.current_joint_state.effort[5] / (
                self.L1 + self.L2) - self.force_offset

            self.force_pub.publish(Float32(data=ee_force))

            if self.path_planner.movegroup_status == GoalStatus.STATUS_SUCCEEDED:

                self.state = State.EXECUTING
                self.path_planner.movegroup_status = GoalStatus.STATUS_UNKNOWN


def main(args=None):
    rclpy.init(args=args)

    drawing = Drawing()

    rclpy.spin(drawing)


if __name__ == '__main__':
    main()
