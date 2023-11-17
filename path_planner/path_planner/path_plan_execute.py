"""
Allow user to manipulate a robot.

Interfaces with the node and the robot to allow the user to manipulate
the robot using the move group and action clients.
PUBLISHERS:
  + /collision object (CollisionObject) - The collision box representing the
  table
SERVICES:
    none
PARAMETERS:
    none
"""

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from std_msgs.msg import Header

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (JointConstraint, TrajectoryConstraints,
                             Constraints, OrientationConstraint, PlanningScene, PlanningOptions,
                             RobotState, AllowedCollisionMatrix,
                             PlanningSceneWorld, MotionPlanRequest,
                             WorkspaceParameters, PositionIKRequest, RobotTrajectory)

from moveit_msgs.msg import MoveItErrorCodes

from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from sensor_msgs.msg import JointState, MultiDOFJointState

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.srv import GetCartesianPath

from octomap_msgs.msg import OctomapWithPose, Octomap
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionFK
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from franka_msgs.action import Homing, Grasp

import modern_robotics as mr
import numpy as np


class Path_Plan_Execute():
    """Class defining brick state."""

    def __init__(self, node):
        """
        Initialize an instance of a class.

        Initialize an instance of a facilitator class that provides the
        functions utilized by the node.

        Args:
        ----
        node: The ros2 node passed into the class that inherits its
        features.

        """
        self.node = node

        # getting current joint states
        self.joint_states_callback_group = MutuallyExclusiveCallbackGroup()
        self.joint_states_subs = self.node.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10, callback_group=self.joint_states_callback_group)
        self.current_joint_state = JointState()
        self.fk_callback_group = MutuallyExclusiveCallbackGroup()
        self.ik_callback_group = MutuallyExclusiveCallbackGroup()
        self.cartesian_callback_group = MutuallyExclusiveCallbackGroup()
        self.movegroup_callback_group = MutuallyExclusiveCallbackGroup()
        self.executetrajectory_callback_group = MutuallyExclusiveCallbackGroup()

        self.movegroup_client = ActionClient(self.node, MoveGroup,
                                             'move_action', callback_group=self.movegroup_callback_group)

        self.executetrajectory_client = ActionClient(
            self.node, ExecuteTrajectory, 'execute_trajectory', callback_group=self.executetrajectory_callback_group)
        self.node.gripper_homing_client = ActionClient(
            self.node, Homing, 'panda_gripper/homing')
        self.node.gripper_grasping_client = ActionClient(
            self.node, Grasp, 'panda_gripper/grasp')

        self.gripper_available = True
        if not self.node.gripper_grasping_client.wait_for_server(
                timeout_sec=2):
            self.gripper_available = False

        self.fk_client = self.node.create_client(
            GetPositionFK, 'compute_fk', callback_group=self.fk_callback_group)

        self.ik_client = self.node.create_client(
            GetPositionIK, 'compute_ik', callback_group=self.ik_callback_group)

        self.cartesian_path_client = self.node.create_client(
            GetCartesianPath, 'compute_cartesian_path', callback_group=self.cartesian_callback_group)

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'IK service not available, waiting again...')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'IK service not available, waiting again...')
        while not self.cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'IK service not available, waiting again...')

        # self.goal_pose = Point(x=0.0, y=0.0, z=0.0)
        self.goal_pose = Pose()
        self.goal_orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        self.robot_state = RobotState()

        self.movegroup_goal_msg = MoveGroup.Goal()
        self.movegroup_result = None
        self.movegroup_status = GoalStatus.STATUS_UNKNOWN
        self.executetrajectory_goal_msg = ExecuteTrajectory.Goal()
        self.executetrajectory_result = None
        self.executetrajectory_status = GoalStatus.STATUS_UNKNOWN

        self.gripper_status = GoalStatus.STATUS_UNKNOWN

        self.cancel_status = None

        self.goal_handle_status = None
        self.grasp_action_result = False

        self.goal_joint_state = None
        self.planned_trajectory = None

        self.cartesian_trajectory_error_code = MoveItErrorCodes()
        self.cartesian_trajectory_error_code.val = 0

        self.fk_error_code = MoveItErrorCodes()
        self.fk_error_code = 0

        self.L1 = 0.333
        self.L2 = 0.3160
        self.L3 = 0.3840
        self.L4 = 0.088
        self.L5 = 0.1070

        self.mass_matrix = np.array([[0.502922, -0.00659085, 0.476807, -0.00202931, 0.0526733, -0.000254634, -0.00282315],
                                     [-0.00659085, 0.476807, -0.00202931, 0.0526733, -
                                         0.000254634, -0.00282315, -0.00659085],
                                     [0.476807, -0.00202931, 0.0526733, -
                                         0.000254634, -0.00282315, -0.00659085, 1.55193],
                                     [-0.00202931, 0.0526733, -
                                    0.000254634, -0.00282315, -0.00659085, 1.55193, -0.0227345],
                                    [0.0526733, -0.000254634, -0.00282315, -
                                        0.00659085, 1.55193, -0.0227345, -0.687864],
                                    [-0.000254634, -0.00282315, -0.00659085,
                                        1.55193, -0.0227345, -0.687864, -0.00779532],
                                    [-0.00282315, -0.00659085, 1.55193, -0.0227345, -0.687864, -0.00779532, -0.0338294]])

        self.planning_scene_publisher = self.node.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )

    def set_initial_condition(self, movegroup_goal_msg):
        """
        Set the initial states of the joints for the JointState message.

        Args:
        ----
        None

        """
        self.node.get_logger().info("Initial Set")
        movegroup_goal_msg.request = MotionPlanRequest(
            reference_trajectories=[],
            pipeline_id='move_group',
            planner_id='',
            group_name=self.node.group_name,
            num_planning_attempts=10,
            allowed_planning_time=5.0,
            max_velocity_scaling_factor=0.1,
            max_acceleration_scaling_factor=0.1,
            cartesian_speed_end_effector_link='',
            max_cartesian_speed=0.0)
        movegroup_goal_msg.request.workspace_parameters = (
            WorkspaceParameters(
                header=Header(stamp=self.node.get_clock().now().to_msg(),
                              frame_id=self.node.frame_id),
                min_corner=Vector3(x=-1.0, y=-1.0, z=-1.0),
                max_corner=Vector3(x=1.0, y=1.0, z=1.0))
        )

        return movegroup_goal_msg

    def update_current_jointstate(self, movegroup_goal_msg):
        """
        Set the current robot joint state.

        Set the current states of the joints for the JointState message.

        Args:
        ----
        None

        """
        movegroup_goal_msg.request.start_state = RobotState(
            joint_state=JointState(
                header=Header(stamp=self.node.get_clock().now().to_msg(),
                              frame_id=self.node.frame_id),
                name=self.current_joint_state.name,
                position=self.current_joint_state.position,
                velocity=[],
                effort=[],
            ),
            multi_dof_joint_state=MultiDOFJointState(header=Header(
                stamp=self.node.get_clock().now().to_msg(),
                frame_id=self.node.frame_id),
                joint_names=[],
                transforms=[],
                twist=[],
                wrench=[]),
            attached_collision_objects=[],
            is_diff=False)

        return movegroup_goal_msg

    def set_goal_constraints(self, movegroup_goal_msg):
        """
        Set the goal position for a robot arm.

        Set the goal constraints iteratively for the robot arm links.

        Args:
        ----
        None

        """
        joint_constraints = []

        # dynamically create the goal constraints based on the size of the
        # data returned from the IK request.
        for i, joint_state_name in enumerate(self.goal_joint_state.name):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_state_name
            joint_constraint.position = self.goal_joint_state.position[i]
            joint_constraint.tolerance_above = 0.05
            joint_constraint.tolerance_below = 0.05
            joint_constraint.weight = 1.0

            joint_constraints.append(joint_constraint)

        movegroup_goal_msg.request.goal_constraints = [
            Constraints(name='goal_constraints',
                        joint_constraints=joint_constraints)
        ]

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = Header(
            stamp=self.node.get_clock().now().to_msg(), frame_id=self.node.frame_id)
        orientation_constraint.orientation = Quaternion(
            x=0.0, y=1.0, z=0.0, w=0.0)
        orientation_constraint.link_name = 'panda_hand'
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        movegroup_goal_msg.request.path_constraints = Constraints(
            name='',
            joint_constraints=[],
            position_constraints=[],
            orientation_constraints=[],
            visibility_constraints=[])
        movegroup_goal_msg.request.trajectory_constraints = (
            TrajectoryConstraints(constraints=[])
        )

        return movegroup_goal_msg

    def set_planning_options(self, movegroup_goal_msg):
        """
        Define planning parameters.

        Define the planning parameters required to use the movegroup node
        including the empty variables..

        Args:
        ----
        None

        """
        movegroup_goal_msg.planning_options = PlanningOptions(
            planning_scene_diff=PlanningScene(
                name='hello',
                robot_state=RobotState(
                    joint_state=JointState(header=Header(
                        stamp=self.node.get_clock().now().to_msg(),
                        frame_id=self.node.frame_id),
                        name=[],
                        position=[],
                        velocity=[],
                        effort=[]),
                    multi_dof_joint_state=MultiDOFJointState(header=Header(
                        stamp=self.node.get_clock().now().to_msg(),
                        frame_id=self.node.frame_id),
                        joint_names=[],
                        transforms=[],
                        twist=[],
                        wrench=[]),
                    attached_collision_objects=[],
                    is_diff=True),
                robot_model_name=self.node.robot_name,
                fixed_frame_transforms=[],
                allowed_collision_matrix=AllowedCollisionMatrix(
                    entry_names=[],
                    entry_values=[],
                    default_entry_names=[],
                    default_entry_values=[]),
                link_padding=[],
                link_scale=[],
                object_colors=[],
                world=PlanningSceneWorld(
                    collision_objects=[],
                    octomap=OctomapWithPose(
                        header=Header(
                            stamp=self.node.get_clock().now().to_msg(),
                            frame_id=self.node.frame_id),
                        origin=Pose(position=Point(x=0.0, y=0.0, z=0.0),
                                    orientation=Quaternion(
                                        x=0.0, y=0.0, z=0.0, w=1.0)),
                        octomap=Octomap(header=Header(
                            stamp=self.node.get_clock().now().to_msg(),
                            frame_id=self.node.frame_id),
                            binary=False,
                            id='',
                            resolution=0.0,
                            data=[])),
                    is_diff=True),
                plan_only=True,
                look_around=False,
                look_around_attempts=0,
                max_safe_execution_cost=0.0,
                replan=False,
                replan_attempts=0,
                replan_delay=0.0))

        return movegroup_goal_msg

    def joint_states_callback(self, msg):
        """Receive the message from the joint state subscriber."""
        self.current_joint_state = msg

        # self.node.get_logger().info(
        #     f"current_joint_state: {self.current_joint_state}")
        # self.node.get_logger().info(
        #     f"ee_force x_dir = {self.current_joint_state.effort[5] / (0.1070 + 0.1130)}")
        # self.node.get_logger().info(
        #     f"7: {self.current_joint_state.effort[5]}\n\n")

        # self.node.get_logger().info(
        #     f"1: {self.current_joint_state.effort[0]}\n")
        # self.node.get_logger().info(
        #     f"2: {self.current_joint_state.effort[1]}\n")
        # self.node.get_logger().info(
        #     f"3: {self.current_joint_state.effort[2]}\n")
        # self.node.get_logger().info(
        #     f"4: {self.current_joint_state.effort[3]}\n")
        # self.node.get_logger().info(
        #     f"5: {self.current_joint_state.effort[4]}\n")
        # self.node.get_logger().info(
        #     f"6: {self.current_joint_state.effort[5]}\n")
        # self.node.get_logger().info(
        #     f"7: {self.current_joint_state.effort[6]}\n\n")
        # self.node.get_logger().info(
        #     f"panda_joint8: {self.current_joint_state.position[7]}\n")
        # self.node.get_logger().info(
        #     f"panda_joint9: {self.current_joint_state.position[8]}\n\n")

    def calc_EE_force(self):
        thetalist = np.asarray(self.current_joint_state.position[:7])
        taulist = np.asarray(self.current_joint_state.effort[:7])
        # self.node.get_logger().info(f"{self.current_joint_state.effort}")
        # self.node.get_logger().info(f"{thetalist}")

        Slist = np.array([[0, 0, 0, 0, 0, 0, 0], [0, 1, 0, -1, 0, -1, 0], [1, 0, 1, 0, 1, 0, -1],
                         [0, -self.L1, 0, self.L1+self.L2, 0, self.L1+self.L2+self.L3, 0], [0, 0, 0, 0, 0, 0, self.L4], [0, 0, 0, -self.L4, 0, 0, 0]])

        Jb = mr.JacobianSpace(Slist, thetalist)

        Ftip = np.linalg.pinv(Jb.T) @ taulist

        return Ftip

    async def fk_callback(self):
        # self.node.get_logger().info(
        #     f"current_joint_state: {self.current_joint_state}")
        request = GetPositionFK.Request()
        request.header = Header(
            stamp=self.node.get_clock().now().to_msg()
        )
        request.fk_link_names = ['panda_link0', 'panda_link1', 'panda_link2', 'panda_link3',
                                 'panda_link4', 'panda_link5', 'panda_link6', 'panda_link7', 'panda_hand_tcp']
        request.robot_state = RobotState(
            joint_state=JointState(
                header=Header(stamp=self.node.get_clock().now().to_msg()),
                name=self.current_joint_state.name,
                position=self.current_joint_state.position,
                # velocity=self.current_joint_state.velocity,
                # effort=self.current_joint_state.effort
            ))

        fk_result = await self.fk_client.call_async(request)
        self.fk_pose = fk_result.pose_stamped
        # self.node.get_logger().info(f"fk_pose[]: {self.fk_pose}")
        self.fk_link_name = fk_result.fk_link_names
        self.fk_error_code = fk_result.error_code

    async def ik_callback(self, pose, joint_state):
        """
        Format the IK service message and send it.

        Format the IK service message to move the robot
        to a position defined by the user and tell the IK service
        to computer the joint states required to do so.

        Args:
        ----
        None

        """
        request = GetPositionIK.Request()
        position = PositionIKRequest()

        header = Header(
            stamp=self.node.get_clock().now().to_msg()
        )

        position.group_name = self.node.group_name
        position.avoid_collisions = True
        position.ik_link_name = 'panda_hand'
        position.robot_state.joint_state.header = header
        position.robot_state.joint_state = joint_state

        position.pose_stamped.header = header
        # position.pose_stamped.pose.position = self.goal_pose
        # position.pose_stamped.pose.orientation = self.goal_orientation
        position.pose_stamped.pose = pose
        position.timeout.sec = 5

        request.ik_request = position

        result = await self.ik_client.call_async(request)

        return result

    async def get_goal_joint_states(self, pose):
        """
        Set desired goal oreintation.

        Set the desired goal orientation for the robot arm using the
        IK service.

        Args:
        ----
        None

        """
        result = await self.ik_callback(pose, self.current_joint_state)
        self.goal_joint_state = result.solution.joint_state

    async def plan_cartesian_path(self, queue):
        self.cartesian_path_request = GetCartesianPath.Request()

        self.cartesian_path_request.header = Header(
            stamp=self.node.get_clock().now().to_msg())
        self.cartesian_path_request.start_state = RobotState(
            joint_state=JointState(
                header=Header(stamp=self.node.get_clock().now().to_msg()),
                name=self.current_joint_state.name,
                position=self.current_joint_state.position,
                velocity=self.current_joint_state.velocity,
                effort=self.current_joint_state.effort),
            is_diff=False
        )

        self.cartesian_path_request.group_name = self.node.group_name
        self.cartesian_path_request.waypoints = queue
        self.cartesian_path_request.link_name = 'panda_hand'
        # setting this to 0.1 for now, could cause problems later
        self.cartesian_path_request.max_step = 0.1
        # self.cartesian_path_request.jump_threshold = 0
        # self.cartesian_path_request.prismatic_jump_threshold = 0
        # self.cartesian_path_request.revolute_jump_threshold = 0
        self.cartesian_path_request.avoid_collisions = True
        self.cartesian_path_request.max_velocity_scaling_factor = 0.1
        self.cartesian_path_request.max_acceleration_scaling_factor = 0.1
        # self.node.get_logger().info(f"request: {self.cartesian_path_request}")

        cartesian_trajectory_result = await self.cartesian_path_client.call_async(self.cartesian_path_request)

        self.node.get_logger().info(
            f"result: {cartesian_trajectory_result}")
        self.cartesian_trajectory_start_state = cartesian_trajectory_result.start_state
        # this is the trajectory we will execute
        self.cartesian_trajectory_solution = cartesian_trajectory_result.solution
        # fraction of the path the was computed (number of waypoints traveled through)
        self.cartesian_trajectory_fraction = cartesian_trajectory_result.fraction
        self.cartesian_trajectory_error_code = cartesian_trajectory_result.error_code

        self.node.get_logger().info(
            f"{self.cartesian_trajectory_fraction * 100}% of the path was computed!")
        self.node.get_logger().info(
            f"cartesian_trajectory error code: {self.cartesian_trajectory_error_code}")

        self.planned_trajectory = self.cartesian_trajectory_solution

    def plan_path(self):
        """
        Plan a path using the robots joint states and other parameters.

        Plan a path by passing the current jointstates and planning
        parameters, and calls the movegroup_client asynchronously to calculate
        a valid path if possible.

        Args:
        ----
        request (int) : a dummy callback variable
        response (int) : a dummy response variable

        """
        self.movegroup_status = GoalStatus.STATUS_UNKNOWN
        self.movegroup_result = None

        if len(self.goal_joint_state.position) > 0:
            movegroup_goal_msg = MoveGroup.Goal()

            movegroup_goal_msg = self.set_initial_condition(movegroup_goal_msg)
            movegroup_goal_msg = self.update_current_jointstate(
                movegroup_goal_msg)
            movegroup_goal_msg = self.set_planning_options(movegroup_goal_msg)
            movegroup_goal_msg = self.set_goal_constraints(movegroup_goal_msg)

            movegroup_goal_msg.planning_options.plan_only = True

            # self.node.get_logger().info(
            #     f"movegroup_goal_msg: {movegroup_goal_msg}")

            self.send_goal_future = self.movegroup_client.send_goal_async(
                movegroup_goal_msg,
                feedback_callback=self.feedback_callback)
            self.send_goal_future.add_done_callback(
                self.movegroup_goal_response_callback)
        else:
            self.node.get_logger().error("Given pos is invalid")

    def movegroup_goal_response_callback(self, future):
        """
        Provide a result from a callback function.

        Provide a future result on the callback for async planning,
        executing, and planning and executing.

        Args:f
        ----
        future (result) : the future object from the send_goal_future
        function

        """
        self.goal_handle = future.result()
        self.movegroup_goal_handle_status = self.goal_handle.status
        if not self.goal_handle.accepted:
            self.node.get_logger().info('Planning Goal Rejected :P')
            return

        self.node.get_logger().info('Planning Goal Accepted :)')

        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(
            self.get_movegroup_result_callback)

    def get_movegroup_result_callback(self, future):
        """
        Provide a future result.

        Provide a future result on the movegroup goal response callback.

        Args:
        ----
        future (result) : the future object from the
        movegroup_goal_response_callback function

        """
        self.movegroup_result = future.result().result
        self.movegroup_status = future.result().status

        self.node.get_logger().info(
            f"movegroup_result: {self.movegroup_status}")

        # append the calculated trajectory to our global variable
        # for excuting later
        self.planned_trajectory = self.movegroup_result.planned_trajectory
        # self.node.get_logger().info(
        # f"planned_trajectory: {self.planned_trajectory}")

        self.node.get_logger().info("executetrajectory goal msg set!")

    def execute_individual_trajectories(self):

        joint_trajectories = []

        for point in self.planned_trajectory.joint_trajectory.points:
            temp = JointTrajectoryPoint()
            temp.positions = point.positions
            temp.velocities = point.velocities
            temp.accelerations = point.accelerations
            temp.effort = point.effort
            temp.time_from_start.nanosec = 100000000
            joint_trajectory = JointTrajectory()
            joint_trajectory.joint_names = self.planned_trajectory.joint_trajectory.joint_names
            joint_trajectory.points = [temp]

            joint_trajectories.append(joint_trajectory)

        return joint_trajectories

    def execute_path(self):
        """
        Execute a previously planned path.

        Execute a planned path by calling the execute_trajectory_client,
        and sends a future object to indicate completion of the async call.

        Args:
        ----
        None

        """
        # self.executetrajectory_status = GoalStatus.STATUS_UNKNOWN
        # self.executetrajectory_result = None

        executetrajectory_goal_msg = ExecuteTrajectory.Goal()
        executetrajectory_goal_msg.trajectory = self.planned_trajectory
        self.send_goal_future = (
            self.executetrajectory_client.send_goal_async(
                executetrajectory_goal_msg,
                feedback_callback=self.feedback_callback
            )
        )
        self.send_goal_future.add_done_callback(
            self.executetrajectory_goal_response_callback)

    def executetrajectory_goal_response_callback(self, future):
        """
        Provide a future result.

        Provide a future result on the execute path callback.

        Args:
        ----
        future (result) : the future object from the async
        execute_path function

        """
        self.executetrajectory_goal_handle = future.result()
        self.node.get_logger().info(
            f"self.executetrajectory_goal_handle: {self.executetrajectory_goal_handle}")
        if not self.executetrajectory_goal_handle.accepted:
            self.node.get_logger().info('Execute Goal Rejected :P')
            return

        self.node.get_logger().info('Execute Goal Accepted :)')

        self.get_result_future = self.executetrajectory_goal_handle.get_result_async()
        self.get_result_future.add_done_callback(
            self.get_executetrajectory_result_callback)

    def get_executetrajectory_result_callback(self, future):
        """
        Provide a future result.

        Provide a future result on the goal response callback.

        Args:
        ----
        future (result) : the future object from the async
        get_trajectory_result function

        """
        self.executetrajectory_result = future.result().result
        self.executetrajectory_status = future.result().status
        self.node.get_logger().info(
            f"EXECUTE TRAJECTORY STATUS: {self.executetrajectory_status}")
        self.node.get_logger().info(
            f'Result Error Code: {self.executetrajectory_result.error_code}')

    async def plan_and_execute_path(self):
        """
        Plan and execute a path to the goal.

        Plan and execute a path to a goal position and orientation
        asynchronously by calling the movegroup_client and seeking a response
        to indicate completion.

        Args:
        ----
        request (int) : a dummy callback variable
        response (int) : a dummy response variable

        """
        self.update_current_jointstate()
        self.set_planning_options()
        await self.get_goal_joint_states()
        if len(self.goal_joint_state.position) > 0:
            self.set_goal_constraints()

            self.movegroup_goal_msg.planning_options.plan_only = False
            self.send_goal_future = self.movegroup_client.send_goal_async(
                self.movegroup_goal_msg,
                feedback_callback=self.feedback_callback)
            self.send_goal_future.add_done_callback(
                self.movegroup_goal_response_callback)

        else:
            self.node.get_logger().error("Given pos is invalid")

    def cancel_execution(self):
        self.node.get_logger().info(f"help")
        self.cancel_execution_future = self.executetrajectory_goal_handle.cancel()
        self.cancel_execution_future.add_done_callback(
            self.cancel_execution_callback)

    def cancel_execution_callback(self, future):
        self.cancel_goal_handle = future.result()
        # self.cancel_goal_handle_status = self.cancel_goal_handle.status
        if not self.cancel_goal_handle.accepted:
            self.node.get_logger().info('Cancel Goal Rejected :P')
            return

        self.get_result_future = self.cancel_goal_handle.get_result_async()
        self.get_result_future.add_done_callback(
            self.get_cancel_result_callback)

    def get_cancel_result_callback(self, future):
        self.cancel_result = future.result().result
        self.cancel_status = future.result().status

        self.node.get_logger().info(
            f"cancel_status: {self.cancel_status}")

        self.node.get_logger().info("Current trajectory canceled")

    async def CloseGripper(self):
        """
        Create a gripper action call.

        Create the message type for the gripper action call and sends the
        goal async.

        Args:
        ----
        None

        """
        print("entering close gripper function")
        goal_msg = Grasp.Goal()
        goal_msg.width = 0.005
        goal_msg.speed = 0.05
        # goal_msg.force = 20.0
        self.send_goal_future = (
            self.node.gripper_grasping_client.send_goal_async(goal_msg)
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    async def OpenGripper(self):
        """
        Create a gripper action call.

        Create the message type for the gripper action call and sends the
        goal async.

        Args:
        ----
        None

        """
        print("entering close gripper function")
        goal_msg = Grasp.Goal()
        goal_msg.width = 0.035
        goal_msg.speed = 0.05
        # goal_msg.force = 20.0
        self.send_goal_future = (
            self.node.gripper_grasping_client.send_goal_async(goal_msg)
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    async def MoveGripper(self, goal_msg):
        # goal_msg = Grasp.Goal()
        # goal_msg.width = 0.0
        # goal_msg.speed = 0.05

        self.send_goal_future = (
            self.node.gripper_grasping_client.send_goal_async(goal_msg)
        )
        self.send_goal_future.add_done_callbak(
            self.gripper_goal_response_callback)

    def gripper_goal_response_callback(self, future):
        """
        Provide a response for the goal callback.

        Provide whether or not a goal was rejecetd to or not to the user,
        and forward the result along for further processing.

        Args:
        ----
        future : a future object.

        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return

        self.node.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_gripper_result_callback(self, future):
        """
        Provide a future result.

        Provide a future result on the action callback.

        Args:
        ----
        future (result) : the future object from the async action
        result call

        """
        self.gripper_result = future.result().result
        self.gripper_status = future.result().status

    async def feedback_callback(self, feedback_msg):
        """
        Provide a future result message.

        Provide a future result message on the feedback from the
        execute_path() and plan_and_execute() functions.

        Args:
        ----
        feedback_msg (result) : the response object from the async
        execution call

        """
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f"Received Feedback: {feedback}")

    def add_box(self, box_id, frame_id, dimensions, pose):
        """
        Add a collision box to the rviz scene.

        Add a box to the rviz scene representing the table that the robot
        is grabbing objects off of.

        Args:
        ----
        box_id (string) : the id of the box
        frame_id (string) : the id of the box's frame
        dimensions (list) : the lengths of the edges of the box
        pose (list) : the cartesian coordinates of the box origin

        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = box_id

        box_size = SolidPrimitive()
        box_size.type = SolidPrimitive.BOX
        box_size.dimensions = dimensions

        box_pose = Pose()
        box_pose.position.x = pose[0]  # x position
        box_pose.position.y = pose[1]  # y position
        box_pose.position.z = pose[2]  # z position

        collision_object.primitives.append(box_size)
        collision_object.primitive_poses.append(box_pose)

        self.planning_scene_publisher.publish(collision_object)
