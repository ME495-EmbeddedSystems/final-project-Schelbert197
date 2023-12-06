"""
Allow user to manipulate a robot.

Interface with the node and the robot to allow the user to manipulate
the robot using the move group and action clients.

Services:
    none

Clients:
  + compute_fk (GetPositionFK) - Calls the FK service for forward kinematics.
  + compute_ik (GetPositionIK) - Calls the IK service for inverse kinematics.
  + compute_cartesian_path (GetCartesianPath) - Calls a service to compute a\
  cartesian path.

Actions:
  + move_action (MoveGroup) - The action for asking MoveIT to plan non-\
  cartesian paths.
  + panda_gripper/homing (Homing) - The action for homing the panda gripper.
  + panda_gripper/grasp (Grasp) - The action for grasping with the panda\
  gripper.

Publishers:
  + /collision object (CollisionObject) - The collision box representing the\
  table

Subscribers:
  + /joint_states (JointState) - The current joint state of the robot.

"""

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Header

from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetCartesianPath
from moveit_msgs.msg import (JointConstraint, Constraints,
                             OrientationConstraint, PlanningScene,
                             PlanningOptions, RobotState,
                             MotionPlanRequest, WorkspaceParameters,
                             PositionIKRequest, CollisionObject)

from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from franka_msgs.action import Homing, Grasp

from shape_msgs.msg import SolidPrimitive


class Path_Plan_Execute():

    def __init__(self, node):
        """
        Initialize an instance of a class.

        Initialize an instance of a facilitator class that provides the
        functions utilized by the node.

        Args
        ----
        node (Node): The ros2 node passed into the class that inherits its
        features.

        Returns
        -------
        None

        """
        self.node = node

        # create mutually exclusive callback groups
        self.joint_states_callback_group = MutuallyExclusiveCallbackGroup()
        self.fk_callback_group = MutuallyExclusiveCallbackGroup()
        self.ik_callback_group = MutuallyExclusiveCallbackGroup()
        self.cartesian_callback_group = MutuallyExclusiveCallbackGroup()
        self.movegroup_callback_group = MutuallyExclusiveCallbackGroup()
        self.executetrajectory_callback_group = \
            MutuallyExclusiveCallbackGroup()

        # create subscribers
        self.joint_states_subs = self.node.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10,
            callback_group=self.joint_states_callback_group)
        self.current_joint_state = JointState()

        # create action clients

        self.movegroup_client = ActionClient(
            self.node, MoveGroup, 'move_action',
            callback_group=self.movegroup_callback_group)
        self.node.gripper_homing_client = ActionClient(
            self.node, Homing, 'panda_gripper/homing')
        self.node.gripper_grasping_client = ActionClient(
            self.node, Grasp, 'panda_gripper/grasp')

        # check to see if the gripper is available
        self.gripper_available = True
        if not self.node.gripper_grasping_client.wait_for_server(
                timeout_sec=2):
            self.gripper_available = False

        # create service clients
        self.fk_client = self.node.create_client(
            GetPositionFK, '/compute_fk',
            callback_group=self.fk_callback_group)

        self.ik_client = self.node.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.ik_callback_group)

        self.cartesian_path_client = self.node.create_client(
            GetCartesianPath, 'compute_cartesian_path', callback_group=self.
            cartesian_callback_group)

        # wait for the clients' services to be available
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'IK service not available, waiting again...')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'IK service not available, waiting again...')
        while not self.cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'IK service not available, waiting again...')

        self.movegroup_goal_msg = MoveGroup.Goal()
        self.movegroup_result = None
        self.movegroup_status = GoalStatus.STATUS_UNKNOWN

        self.goal_joint_state = None
        self.planned_trajectory = None

        self.planning_scene_publisher = self.node.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )

    def joint_states_callback(self, msg):
        """Receive the message from the joint state subscriber."""
        self.current_joint_state = msg

    def create_movegroup_msg(self, movegroup_goal_msg):
        """
        Create a movegroup message for trajectory planning.

        Create a movegroup message and populate it with the correct
        planning parameters for the moveit motion planner.

        Args
        ----
        movegroup_goal_msg (MoveGroup): An unpopulated MoveGroup action object.

        Returns
        -------
        movegroup_goal_msg (MoveGroup): A populated MoveGroup action object.

        """
        self.node.get_logger().info("Initial Set")
        # set up the message
        movegroup_goal_msg.request = MotionPlanRequest(
            reference_trajectories=[],
            pipeline_id='move_group',
            planner_id='',
            group_name='panda_manipulator',
            num_planning_attempts=10,
            allowed_planning_time=5.0,
            max_velocity_scaling_factor=0.1,
            max_acceleration_scaling_factor=0.1,
            max_cartesian_speed=0.0
        )

        # set the workspace parameters
        movegroup_goal_msg.request.workspace_parameters = (
            WorkspaceParameters(
                header=Header(stamp=self.node.get_clock().now().to_msg(),
                              frame_id=self.node.frame_id),
                min_corner=Vector3(x=-1.0, y=-1.0, z=-1.0),
                max_corner=Vector3(x=1.0, y=1.0, z=1.0))
        )

        # set the start state of the robot as a RobotState variable
        movegroup_goal_msg.request.start_state = RobotState(
            joint_state=JointState(
                header=Header(stamp=self.node.get_clock().now().to_msg(),
                              frame_id=self.node.frame_id),
                name=self.current_joint_state.name,
                position=self.current_joint_state.position,
                velocity=self.current_joint_state.velocity,
                effort=self.current_joint_state.effort
            )
        )

        # set the goal constraint
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
            stamp=self.node.get_clock().now().to_msg())
        orientation_constraint.orientation = Quaternion(
            x=1.0, y=0.0, z=0.0, w=0.0)
        orientation_constraint.link_name = 'panda_hand_tcp'
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.orientation_constraints = [orientation_constraint]

        movegroup_goal_msg.request.trajectory_constraints.constraints = [
            Constraints(
                name='',
                orientation_constraints=[orientation_constraint]
            )]

        # set the planning options
        movegroup_goal_msg.planning_options = PlanningOptions(
            planning_scene_diff=PlanningScene(
                robot_state=RobotState(
                    joint_state=JointState(header=Header(
                        stamp=self.node.get_clock().now().to_msg())),
                    is_diff=True),
                robot_model_name=self.node.robot_name),
            plan_only=True,
            look_around=False,
            look_around_attempts=0,
            max_safe_execution_cost=0.0,
            replan=False)

        return movegroup_goal_msg

    async def ik_callback(self, pose, joint_state):
        """
        Format the IK service message and send it.

        Format the IK service message to move the robot
        to a position defined by the user and tell the IK service
        to computer the joint states required to do so.

        Args
        ----
        pose (Pose): A Pose object representing the robot's desired end-
        effector position.
        joint_state: The current joint state of the robot.

        Returns
        -------
        result: The result of the ik service callback.

        """
        request = GetPositionIK.Request()
        position = PositionIKRequest()

        header = Header(
            stamp=self.node.get_clock().now().to_msg()
        )

        self.node.get_logger().info("what")

        position.group_name = self.node.group_name
        position.avoid_collisions = True
        position.ik_link_name = 'panda_hand_tcp'
        position.robot_state.joint_state.header = header
        position.robot_state.joint_state = joint_state

        position.pose_stamped.header = header
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

        Args
        ----
        pose (Pose): A Pose object representing the robot's desired
        end-effector
        position.

        Returns
        -------
        None

        """
        result = await self.ik_callback(pose, self.current_joint_state)
        self.node.get_logger().info(
            f"solution.jiont_state: {result.solution.joint_state}")
        self.goal_joint_state = result.solution.joint_state

    async def plan_cartesian_path(self, queue, velocity=0.025):
        """
        Plan a cartesian path.

        Create a GetCartesianPath.Request() object, and then populate it
        with the desired parameters for cartesian motion.

        Args
        ----
        queue (Pose[]): A list of Pose messages that you would like the robot
        end-effector to travel to.
        velocity (float): The velocity at which the end-effector should move
        during execution.

        Returns
        -------
        None

        """
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
        self.cartesian_path_request.link_name = 'panda_hand_tcp'
        self.cartesian_path_request.max_step = 0.01
        self.cartesian_path_request.avoid_collisions = True
        self.cartesian_path_request.max_velocity_scaling_factor = velocity
        self.cartesian_path_request.max_acceleration_scaling_factor = 0.05

        cartesian_trajectory_result = await self.cartesian_path_client.\
            call_async(self.cartesian_path_request)

        self.cartesian_trajectory_start_state = cartesian_trajectory_result.\
            start_state
        self.cartesian_trajectory_solution = cartesian_trajectory_result.\
            solution
        self.cartesian_trajectory_fraction = cartesian_trajectory_result.\
            fraction
        self.cartesian_trajectory_error_code = cartesian_trajectory_result.\
            error_code

        self.node.get_logger().info(
            f"{self.cartesian_trajectory_fraction * 100}% of the path was \
                computed!")
        self.node.get_logger().info(
            f"cartesian_trajectory error code: \
                {self.cartesian_trajectory_error_code}")

        self.planned_trajectory = self.cartesian_trajectory_solution

    def plan_path(self):
        """
        Plan a path using the robots joint states and other parameters.

        Plan a path by passing the current jointstates and planning
        parameters, and calls the movegroup_client asynchronously to calculate
        a valid path if possible.

        Args
        ----
        None

        Returns
        -------
        None

        """
        self.movegroup_status = GoalStatus.STATUS_UNKNOWN
        self.movegroup_result = None
        # await self.get_goal_joint_states()
        if len(self.goal_joint_state.position) > 0:
            movegroup_goal_msg = MoveGroup.Goal()

            movegroup_goal_msg = self.create_movegroup_msg(movegroup_goal_msg)

            self.node.get_logger().info("here2")

            self.send_goal_future = self.movegroup_client.send_goal_async(
                movegroup_goal_msg,
                self.movegroup_goal_response_callback)
        else:
            self.node.get_logger().error("Given pos is invalid")

    def movegroup_goal_response_callback(self, future):
        """
        Provide a result from a callback function.

        Provide a future result on the callback for async planning,
        executing, and planning and executing.

        Args
        ----
        future (Future): the future object from the send_goal_future function.

        Returns
        -------
        None

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

        Args
        ----
        future (result) : the future object from the
        movegroup_goal_response_callback function

        Returns
        -------
        None

        """
        self.movegroup_result = future.result().result
        self.movegroup_status = future.result().status

        self.node.get_logger().info(
            f"movegroup_result: {self.movegroup_status}")

        self.planned_trajectory = self.movegroup_result.planned_trajectory
        self.node.get_logger().info("Trajectory Planned!")

    def execute_individual_trajectories(self):
        """
        Reorganize a list of joint trajectories.

        Manipulate the RobotTrajectory message returned from either the
        MoveGroup motion planner or the Cartesian motion planner into a
        discrete list of joint trajectories to execute one by one.

        Args
        ----
        None

        Returns
        -------
        joint_trajectories (JointTrajectory[]): A list of JointTrajectory
        objects to be executed.

        """
        joint_trajectories = []

        for point in self.planned_trajectory.joint_trajectory.points:
            temp = JointTrajectoryPoint()
            temp.positions = point.positions
            temp.velocities = point.velocities
            temp.accelerations = point.accelerations
            temp.effort = point.effort
            temp.time_from_start.nanosec = 100000000  # 0.1 seconds
            joint_trajectory = JointTrajectory()
            joint_trajectory.joint_names = self.planned_trajectory.\
                joint_trajectory.joint_names
            joint_trajectory.points = [temp]

            joint_trajectories.append(joint_trajectory)

        return joint_trajectories

    async def feedback_callback(self, feedback_msg):
        """
        Provide a future result message.

        Provide a future result message on the feedback from the
        execute_path() and plan_and_execute() functions.

        Args
        ----
        feedback_msg (Feedback): the response object from the async
        execution call

        Returns
        -------
        None

        """
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f"Received Feedback: {feedback}")

    def set_goal_pose(self, pose):
        """
        Set desired goal position.

        Set the desired goal position for the robot arm.

        Args
        ----
        pose (Pose): The goal cartesian pose of the end effector

        Returns
        -------
        None

        """
        self.goal_pose = pose
        self.goal_orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)

        return self.goal_pose

    def add_box(self, box_id, frame_id, dimensions, pose):
        """
        Add a collision box to the rviz scene.

        Add a box to the rviz scene representing the table that the robot
        is grabbing objects off of.

        Args
        ----
        box_id (string) : the id of the box
        frame_id (string) : the id of the box's frame
        dimensions (list) : the lengths of the edges of the box
        pose (list) : the cartesian coordinates of the box origin

        Returns
        -------
        None

        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = box_id

        box_size = SolidPrimitive()
        box_size.type = SolidPrimitive.BOX
        box_size.dimensions = dimensions

        box_pose = pose

        collision_object.primitives.append(box_size)
        collision_object.primitive_poses.append(box_pose)

        self.planning_scene_publisher.publish(collision_object)
