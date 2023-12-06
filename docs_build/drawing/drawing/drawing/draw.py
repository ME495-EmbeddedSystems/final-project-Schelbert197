"""
Plan and execute robot paths using force control.

Accept requests to plan trajectories for the franka robot, and
subsequently send them to another node to be executed. Additionally,
calculate the estimated force at the end-effector, and publish it
on a topic.

Parameters
----------
  + use_fake_hardware (bool) - Determines whether or not trajectories will\
  be executed on the real robot or only in rviz.
  + robot_name (string) - the name of the robot.
  + group_name (string) - the planning group of the robot.
  + frame_id (string) - the id of the base frame of the robot.

SERVICES:
  + moveit_mp_service (MovePose) - Uses the request to send action requests\
  to MoveIT
  + cartesian_mp_service (Cartesian) - Use the request to send service calls\
  to the /compute_cartesian_path service.
  + replan_path (Replan) - Use the request to replan the trajectory to a Pose.

CLIENTS:
  + joint_trajectories_client (ExecuteJointTrajectories) - Send join\
  trajectories to be executed.

PUBLISHERS:
  + force_pub (EEForce) - Publish the force at the end-effector in the end-\
  effector frame's x axis.

"""

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from geometry_msgs.msg import Point, Quaternion, Pose

from drawing.path_plan_execute import Path_Plan_Execute

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from enum import Enum, auto

from action_msgs.msg import GoalStatus

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf2_ros
from brain_interfaces.srv import (MovePose, Cartesian,
                                  ExecuteJointTrajectories, Replan)
from brain_interfaces.msg import EEForce

import numpy as np
import transforms3d as tf
np.set_printoptions(suppress=True)


class State(Enum):

    EXECUTING = auto()
    WAITING = auto()

    PLAN_MOVEGROUP = auto()
    PLAN_CARTESIAN_MOVE = auto()
    MAKE_BOARD = auto()
    REMOVE_BOARD = auto()


class Drawing(Node):
    """
    Pick up trash with the Franka.

    Drive a robot around to pickup objects that are a user specified
    distance within its workspace.

    Args
    ----
    None

    Returns
    -------
    None

    """

    def __init__(self):

        super().__init__("Drawing")

        # declare parameters
        self.declare_parameter('use_fake_hardware', True)
        self.declare_parameter('robot_name', 'panda')
        self.declare_parameter('group_name', 'panda_manipulator')
        self.declare_parameter('frame_id', 'panda_link0')

        # get parameters
        self.use_fake_hardware = self.get_parameter(
            'use_fake_hardware').get_parameter_value().bool_value

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
        self.moveit_mp_callback_group = MutuallyExclusiveCallbackGroup()
        self.cartesian_mp_callback_group = MutuallyExclusiveCallbackGroup()
        self.replan_service_callback_group = MutuallyExclusiveCallbackGroup()
        self.execute_joint_trajectories_callback_group = \
            MutuallyExclusiveCallbackGroup()
        self.board_service_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            0.001, self.timer_callback,
            callback_group=self.timer_callback_group)

        self.path_planner = Path_Plan_Execute(self)

        # these are used for computing the current location of the end-effector
        # using the tf tree.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # create services

        # this service is for the brain node to send singular poses for
        # this node to plan paths using te moveite motion planner
        self.moveit_mp_service = self.create_service(
            MovePose, '/moveit_mp', self.moveit_mp_callback,
            callback_group=self.moveit_mp_callback_group)

        # this service is for the brain node to send lists of poses for
        # this node to use to plan paths using the cartesian motion
        # planner.
        self.cartesian_mp_service = self.create_service(
            Cartesian, '/cartesian_mp', self.cartesian_mp_callback,
            callback_group=self.cartesian_mp_callback_group)

        self.replan_service = self.create_service(
            Replan, '/replan_path', self.replan_callback,
            callback_group=self.replan_service_callback_group)

        self.joint_trajectories_client = self.create_client(
            ExecuteJointTrajectories, '/joint_trajectories',
            callback_group=self.execute_joint_trajectories_callback_group)

        # this publisher is used to send the current force at the end-effector
        # to the node we created to execute trajectories.
        self.force_pub = self.create_publisher(
            EEForce, '/ee_force', 10)

        self.plan_future = Future()
        self.execute_future = Future()

        self.moveit_mp_queue = []  # moveit motion planner queue
        self.cartesian_mp_queue = []  # cartesian motion planner queue

        self.state = State.WAITING

        self.gripper_mass = 1.795750991  # kg
        self.g = 9.81  # m/s**2

        # position of the center of mass of the end-effector
        # in the panda_hand frame
        self.pc = np.array([-0.01, 0, 0.03])

        # list bc moving average
        self.ee_force = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # N

        self.cartesian_velocity = []
        self.use_force_control = []
        self.replan = False

        self.i = 0

        self.joint_trajectories = ExecuteJointTrajectories.Request()

        self.home_position = Pose(
            position=Point(x=-0.5, y=0.0, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )
        table = Pose()
        table.position = Point(z=-1.6)
        self.draw_obs(name="table", pos=table, size=[1.5, 1.0, 3.0])
        self.board_future = rclpy.task.Future()

    def array_to_transform_matrix(self, translation, quaternion):
        """
        Calculate the transformation and rotation matrices.

        Calculate the transformation and rotation matrices using
        a 3x1 translation vector and a quaternion.

        Args
        ----
        translation (numpy array): A 3x1 translation vector.
        quaternion (numpy array): A 4x1 quaternion.

        Returns
        -------
        transform_matrix (numpy array): A transformation matrix.
        rotation_matrix (nuumpy array): A rotation matrix.

        """
        # Normalize the quaternion
        quaternion /= np.linalg.norm(quaternion)
        quaternion = [quaternion[3], quaternion[0],
                      quaternion[1], quaternion[2]]

        # Create rotation matrix from quaternion
        rotation_matrix = tf.quaternions.quat2mat(quaternion)

        # Create the transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation

        return transform_matrix, rotation_matrix

    def calc_joint_torque_offset(self):
        """
        Calculate the joint torque offset in panda_joint6.

        Using the location of the center of mass of the gripper and
        the mass of the gripper and its attachments, figure out the
        portion of the torque in panda_joint6 that is due to
        the gripper itself and the current time.

        Args
        ----
        None

        Returns
        -------
        joint_torque_offset (float): The amount of torque in panda_joint6
        that is due to the mass of the end-effector and its attachments.

        """
        # world to panda_joint6 from tf
        pw6, quaternion_w6 = self.get_transform(
            'panda_link0', 'panda_link6')

        # transformation and rotation matrices from translation vector
        # and quaternion.
        Tw6, Rw6 = self.array_to_transform_matrix(pw6, quaternion_w6)

        # panda_joint6 to the "flange" of the panda robot. This is
        # defined at panda0.robot in the settings.
        p6f, quaternion_6f = self.get_transform('panda_link6', 'panda_hand')

        # transformation matrices from translation vector and quaternion.
        T6f, R6f = self.array_to_transform_matrix(p6f, quaternion_6f)

        # calculate force due to gravity in the world frame
        Fw = np.array([0, 0, -self.gripper_mass * self.g])

        # calculate force in the frame of panda_joint6
        F6 = np.linalg.inv(Rw6) @ Fw

        # calculate the expected moment in panda_joint6
        M6 = F6 * (p6f + R6f @ self.pc)

        joint_torque_offset = M6[1]

        return joint_torque_offset

    def calc_ee_force(self, effort_joint6):
        """
        Calculate the force at the end-effector.

        Calculate the force at the end-effector in the xyz directions
        in the frame of the end-effector.

        Args
        ----
        effort_joint6 (float): The effort in panda_joint6 in Nm.

        Returns
        -------
        Fe (float): force at the end effector.

        """
        # ee to panda_joint6 from tf
        pe6, quaternion_e6 = self.get_transform(
            'panda_hand_tcp', 'panda_link6')

        # calc transformation and rotation matrices.
        Te6, Re6 = self.array_to_transform_matrix(pe6, quaternion_e6)

        # panda_joint6 to ee from tf
        p6e, quaternion_6e = self.get_transform(
            'panda_link6', 'panda_hand_tcp')

        # torque in panda_joint6, which is about its y axis.
        M6 = np.array([0, effort_joint6, 0])

        # calculate the force at the ee due to the moment.
        F6 = np.divide(M6, p6e,
                       out=np.zeros_like(p6e), where=p6e != 0)

        # transform force from the panda_joint6 frame to the ee frame.
        Fe = Re6 @ F6

        return Fe

    async def moveit_mp_callback(self, request, response):
        """
        Queue a pose to be planned with the MoveIT motion planner.

        Queue a pose to be planned using the MoveIT motion planner,
        and specify whether or not that pose should be executed with
        force control.

        Args
        ----
        request (MovePose.Request): Target pose for the MoveIT motion
        planner to plan to, and whether or not the trajectory should
        be executed with force control or not.

        Returns
        -------
        response: None

        """
        await self.board_future

        self.get_logger().info("MOVEIT MOTION PLAN REQUEST RECEIVED")

        self.plan_future = Future()
        self.execute_future = Future()

        self.moveit_mp_queue.append(request.target_pose)
        self.state = State.PLAN_MOVEGROUP
        self.use_force_control.append(request.use_force_control)
        self.replan = False

        await self.plan_future
        self.get_logger().info("MOVEIT MOTION PLAN REQUEST COMPLETE")

        self.plan_future = Future()
        self.execute_future = Future()
        self.state = State.REMOVE_BOARD
        await self.board_future

        return response

    async def cartesian_mp_callback(self, request, response):
        """
        Queue a letter to be drawn.

        This function will be called when the brain node sends
        this node a message with a cartesian path to plan. I had an
        idea that the brain can just send this node a list of poses
        that would need to traveled to for drawing a letter, and that
        this node could add in some in-between movements that help guide
        the robot to the correct position on the board. We can discuss this.

        Args
        ----
        request (Pose[]): List of poses to be computed by the
        /compute_cartesian_path service, the velocity of the cartesian
        trajectory, whether the cartesian request is a replan request or not,
        and whether or not the cartesian path should be executed with force
        control or not.

        Returns
        -------
        response: None

        """
        self.get_logger().info("CARTESIAN MOTION PLAN REQUEST RECEIVED")

        self.plan_future = Future()
        self.execute_future = Future()

        self.replan = request.replan
        for pose in request.poses:
            self.cartesian_mp_queue.append(pose)
            self.cartesian_velocity.append(request.velocity)

        self.state = State.PLAN_CARTESIAN_MOVE
        self.use_force_control = request.use_force_control

        await self.plan_future

        self.plan_future = Future()
        self.execute_future = Future()  # different from the moveit callback?

        return response

    async def replan_callback(self, request, response):
        """
        Replan a trajectory.

        Replans a trajectory that previously exceeded the force
        threshold when being executed.

        Args
        ----
        request (Pose): A pose to be replanned.

        Returns
        -------
        response (JointTrajectory[]): A list of joint trajectories
        to be executed.

        """
        self.get_logger().info("REPLAN REQUEST RECEIVED")

        self.get_logger().info(f"request.pose: {request.pose}")

        self.cartesian_mp_queue.insert(0, request.pose)
        self.cartesian_velocity.insert(0, 0.015)

        await self.path_planner.plan_cartesian_path(
            [self.cartesian_mp_queue[0]], self.cartesian_velocity[0])

        response.joint_trajectories = self.path_planner.\
            execute_individual_trajectories()

        self.cartesian_mp_queue.pop(0)
        self.cartesian_velocity.pop(0)

        return response

    def draw_obs(self, name, pos, size):
        """
        Draw an obstacle.

        Draw an obstacle so that motion planners do not plan
        paths that would intersect with the obstacle.

        Args
        ----
        name (string): The obstacle's id.
        pos (Pose): The pose at which the obstacle will be drawn.
        size (list): The size of the obstacle.

        Returns
        -------
        None

        """
        box_id = name
        frame_id = 'panda_link0'
        dimensions = size  # size
        pose = pos  # position

        # Add the box to the planning scene using the add_box method
        self.path_planner.add_box(box_id, frame_id, dimensions, pose)

    def get_transform(self, parent_frame, child_frame):
        """
        Listen to transforms between parent and child frame.

        Args
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
            brick_to_platform = np.array([transl.x, transl.y, transl.z])
            rotation = np.array([rot.x, rot.y, rot.z, rot.w])

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

    def execute_done_callback(self, future):
        """Perform actions when a move is done executing."""
        if not self.cartesian_mp_queue:
            self.get_logger().info("plan has been executed")
            self.plan_future.set_result("done")
        else:
            self.get_logger().info("cartesian move was executed")
            self.state = State.PLAN_CARTESIAN_MOVE
            self.execute_future = Future()

    async def timer_callback(self):
        """
        Timer loop for the drawing node.

        The timer loop functions as the main loop of the node, and
        also contains a state machine. It performs actions for force
        control, regulates when trajectories are planned, which motion
        planner is used, and send trajectories off to be executed.

        Args
        ----
        None

        Returns
        -------
        None

        """
        if self.state == State.PLAN_MOVEGROUP:

            # here we check to see if the big_move queue is empty, and if not,
            # we use the moveit motion planner to create a trajectory.
            # then we go to the waiting loop, where we will wait for the future
            # to return true.

            self.get_logger().info("here")

            if not self.moveit_mp_queue:  # check if the queue is empty
                self.state == State.PLAN_CARTESIAN_MOVE
                return

            self.get_logger().info("here")
            await self.path_planner.get_goal_joint_states(
                self.moveit_mp_queue[0])
            self.get_logger().info("here")
            self.joint_trajectories = ExecuteJointTrajectories.Request()
            self.joint_trajectories.current_pose = self.moveit_mp_queue[0]
            self.joint_trajectories.use_force_control = \
                self.use_force_control[0]

            self.path_planner.plan_path()

            self.state = State.WAITING

            self.moveit_mp_queue.pop(0)
            self.use_force_control.pop(0)

        elif self.state == State.PLAN_CARTESIAN_MOVE:

            # check to see if the cartesian move queue is empty, and if not
            # then plan a cartesian path using the poses in the queue. The
            # /compute_cartesian_path service takes in a list of poses, and
            # creates a trajectory to visit all of those posesp

            if not self.cartesian_mp_queue:
                self.state == State.WAITING

            self.get_logger().info(f"velocity: {self.cartesian_velocity[0]}")

            await self.path_planner.plan_cartesian_path(
                [self.cartesian_mp_queue[0]], self.cartesian_velocity[0])
            self.joint_trajectories = ExecuteJointTrajectories.Request()
            # queue the remaining poses, so that if force threshold is
            # exceeded, send_trajectories can initiate a replan request
            # self.plan_future.set_result("done") directly with the april tags
            # node.

            self.joint_trajectories.current_pose = self.cartesian_mp_queue[0]
            self.joint_trajectories.replan = self.replan
            self.joint_trajectories.use_force_control =\
                self.use_force_control[0]
            self.get_logger().info(
                f"cartesian queue: {self.cartesian_mp_queue}")

            if len(self.cartesian_mp_queue) == 1:
                self.replan = False

            self.cartesian_mp_queue.pop(0)
            self.cartesian_velocity.pop(0)
            self.use_force_control.pop(0)

            self.state = State.EXECUTING

        elif self.state == State.EXECUTING:

            # send the trajectory previously planned, either by the moveit
            # motion planner or the cartesian path planner, to our node for
            # executing trajectories.

            self.joint_trajectories.state = "publish"
            self.joint_trajectories.joint_trajectories = \
                self.path_planner.execute_individual_trajectories()

            self.execute_future = self.joint_trajectories_client.call_async(
                self.joint_trajectories)
            self.execute_future.add_done_callback(self.execute_done_callback)

            self.state = State.WAITING

        elif self.state == State.WAITING:

            # calculate the current force at the end-effector, and send it to
            # the node that is executing our trajectory. Also, check to see
            # whether the moveit motion planner has completed planning. This
            # will only happen if the state prior was State.PLAN_MOVEGROUP.

            while not self.path_planner.current_joint_state.effort:
                return

            joint_torque_offset = self.calc_joint_torque_offset()

            self.ee_force.append(self.calc_ee_force(
                self.path_planner.current_joint_state.effort[5] -
                joint_torque_offset)[2])
            self.ee_force.pop(0)

            if self.i % 10:  # publish the message at a frequency of 100hz
                ee_force_avg = np.average(self.ee_force)

                ee_force_msg = EEForce()
                ee_force_msg.ee_force = ee_force_avg
                self.force_pub.publish(ee_force_msg)

            if self.path_planner.movegroup_status ==\
                    GoalStatus.STATUS_SUCCEEDED:

                self.state = State.EXECUTING
                self.path_planner.movegroup_status = GoalStatus.STATUS_UNKNOWN

        elif self.state == State.MAKE_BOARD:
            ansT, ansR = self.get_transform("panda_link0", "board")
            board_pose = Pose()
            board_pose.position = Point(x=ansT[0], y=ansT[1], z=ansT[2])
            board_pose.orientation = Quaternion(
                x=ansR[0], y=ansR[1], z=ansR[2], w=ansR[3])
            self.draw_obs(pos=board_pose, name="board", size=[2.0, 2.0, 0.02])
            self.board_future.set_result("fdsa")

        elif self.state == State.REMOVE_BOARD:
            board_pose = Pose()
            board_pose.position.z = -0.3
            self.draw_obs(pos=board_pose, name="board", size=[0.0, 0.0, 0.0])
            self.board_future.set_result("remove")

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    drawing = Drawing()

    rclpy.spin(drawing)


if __name__ == '__main__':
    main()
