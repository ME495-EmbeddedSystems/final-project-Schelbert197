"""
Execute trajectories planned for the franka robot.

Execute trajectories planned for the franka robot at 10hz. This is a stand in
for the MoveIT execute trajectory action, since MoveIT doesn't allow us to
cancel goals. When a trajectory is planned by either the MoveGroup motion
planner or the /compute_cartesian_path service, the result is returned in the
form of a RobotTrajectory message. This message includes a JointTrajectory
message, which contains a list of JointTrajectoryPoint messages. We must break
up this list into individual JointTrajectory messages, with just one element in
the JointTrajectoryPoint list. This way, we can execute the original
RobotTrajectory discretely by publishing JointTrajectories on the
/panda_arm_controller/joint_trajectory topic.

SERVICES:
  + joint_trajectory_service (ExecuteJointTrajectories): Execute joint\
    trajectories discretely.

CLIENTS:
  + replan_client (Replan): Replan trajectories that hit the force threshold.
  + update_trajectory_client (UpdateTrajectory): Modify the pose that was\
  planned for so that it is moved slightly in the positive z direction in\
  the whiteboard's frame, which is out of the board.

SUBSCRIBERS:
  + force_sub (EEForce): Receive the current force at the end-effector in the\
  end effector's frame.

"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String

from brain_interfaces.msg import EEForce

from brain_interfaces.srv import (ExecuteJointTrajectories, Replan,
                                  UpdateTrajectory)

from enum import Enum, auto

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros


class State(Enum):

    PUBLISH = auto()
    STOP = auto()


class Executor(Node):

    def __init__(self):

        super().__init__("Execute")

        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.joint_trajectories_callback_group = \
            MutuallyExclusiveCallbackGroup()
        self.replan_callback_group = MutuallyExclusiveCallbackGroup()
        self.update_trajectory_callback_group = \
            MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(
            0.01, self.timer_callback,
            callback_group=self.timer_callback_group)

        # create publishers
        self.pub = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)

        self.execute_trajectory_status_pub = self.create_publisher(
            String, '/execute_trajectory_status', 10)

        # create services
        self.joint_trajectories_service = self.create_service(
            ExecuteJointTrajectories, '/joint_trajectories',
            self.joint_trajectories_callback,
            callback_group=self.joint_trajectories_callback_group)

        # create clients
        self.replan_client = self.create_client(
            Replan, '/replan_path', callback_group=self.replan_callback_group)

        self.update_trajectory_client = \
            self.create_client(UpdateTrajectory,
                               'update_trajectory',
                               callback_group=self.
                               update_trajectory_callback_group)

        # create subscriptions
        self.force_sub = self.create_subscription(
            EEForce, '/ee_force', self.force_callback, 10)

        # these are used for computing the current location of the end-effector
        # using the tf tree.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.joint_trajectories = []
        self.pose = None

        self.ee_force = 0
        self.upper_threshold = 3.0  # N
        self.state = None

        self.use_force_control = False
        self.use_control_loop = False
        self.force_error = 0.0  # N
        self.previous_force_error = 0.0  # N
        self.initial_trajectory_angle = 0.0  # rad
        self.output_angle = 0.0  # rad
        self.integral_force_error = 0.0

        self.replan = False

        self.future = Future()

        self.i = 1

    def get_transform(self, parent_frame, child_frame):
        """
        Listen to transforms between parent and child frame.

        Args
        ----
        parent_frame (string): name of parent frame
        child_frame (string): name of child frame

        Returns
        -------
        brick_to_platform (Pose): the x,y,z of the translational transform

        """
        try:
            pose = Pose()
            trans = self.buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            transl = trans.transform.translation
            rot = trans.transform.rotation
            pose.position = Point(x=transl.x, y=transl.y, z=transl.z)
            pose.orientation = Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w)

            # print(brick_to_platform[2])
            return pose

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

    def force_callback(self, msg):
        """Receive the current force at the end-effector."""
        self.ee_force = msg.ee_force
        # self.use_force_control = msg.use_force_control

    async def joint_trajectories_callback(self, request, response):
        """
        Receive a list of joint trajectories to be executed.

        Receive a list of joint trajectories to be executed. This includes
        flags that specify whether or not to replan the trajectory if it fails,
        whether or not to use force control, and the pose that the trajectory
        was planned for.

        Args
        ----
        request (ExecuteJointTrajectories): List of joint trajectories to be
        executed.

        Returns
        -------
        response (None): None

        """
        self.get_logger().info("message received!")
        self.i = 0

        self.joint_trajectories = request.joint_trajectories
        self.output_angle = self.joint_trajectories[0].points[0].positions[5]
        self.pose = request.current_pose
        self.replan = request.replan
        self.use_force_control = request.use_force_control
        if not self.use_force_control:
            self.use_control_loop = False

        if request.state == "publish":
            self.state = State.PUBLISH
        else:
            self.state = State.STOP

        await self.future

        self.future = Future()

        return response

    async def replan_trajectory(self, into_the_board):
        """
        Replan a trajectory that failed.

        Replan a trajectory that failed. First, call a service that returns
        a new pose to plan for. This pose will be based on where the robot
        collided with the whiteboard. Next, plan a cartesian path to this pose
        and prepare it to be executed.

        Args
        ----
        into_the_board (bool): Whether or not the modified pose should be
        adjusted so that the robot travels towards the whiteboard, or away
        from the whiteboard.

        Returns
        -------
        None

        """
        self.get_logger().info("joint trajectories cleared")
        self.joint_trajectories.clear()

        # replan the trajectory!!
        self.get_logger().info(
            f"pose to be adjusted out of board: {self.pose}")
        update_trajectory_response = await self.update_trajectory_client.\
            call_async(UpdateTrajectory.Request(
                input_pose=self.pose, into_board=into_the_board))

        self.pose = update_trajectory_response.output_pose

        replan_response = await self.replan_client.call_async(Replan.Request(
            pose=self.pose))

        self.joint_trajectories = replan_response.joint_trajectories
        self.output_angle = self.joint_trajectories[0].points[0].positions[5]

    async def timer_callback(self):
        """
        Perform force control and execution of trajectories.

        This node checks whether or not the force at the end-effector is higher
        than a threshold, and if it is then the current trajectory is
        replanned. After the trajectory is replanned, then the trajectory is
        completed using a PID control loop with end-effector force as the
        input, and angle of panda_joint6 in radians as the output.

        Args
        ----
        None

        Returns
        -------
        None

        """
        if self.ee_force > self.upper_threshold and self.use_force_control and\
                self.joint_trajectories:
            self.get_logger().info(
                f"upper_threshold: {self.upper_threshold}")
            self.get_logger().info(
                f"UPPER FORCE THRESHOLD EXCEEDED, EE_FORCE: {self.ee_force}")

            if self.replan:
                # clear the current trajectories first, as we don't want to
                # execute them anymore
                await self.replan_trajectory(False)

                self.use_control_loop = True
                self.use_force_control = False
                self.initial_trajectory_angle = self.joint_trajectories[0].\
                    points[0].positions[5]

            else:

                self.get_logger().info("joint trajectories cleared")
                self.get_logger().info("poses all done")
                self.joint_trajectories.clear()

        elif self.joint_trajectories and self.state == State.PUBLISH and \
                self.i % 10 == 0:

            Kp = 0.0029
            Ki = 0.000005
            Kd = 0.0009

            self.get_logger().info(
                f"user control loop: {self.use_control_loop}")
            self.get_logger().info(
                f"initial angle: {self.initial_trajectory_angle}")

            if self.use_control_loop:
                self.get_logger().info(f"ee_force: {self.ee_force}")
                self.get_logger().info(
                    f"original joint pos: {self.output_angle}")
                force_error = 2.3 - self.ee_force
                self.integral_force_error += force_error * 0.1
                angle_adjustment = Kp * force_error + \
                    Ki * self.integral_force_error + \
                    Kd * (force_error - self.previous_force_error)
                self.output_angle += angle_adjustment
                self.joint_trajectories[0].points[0].positions[5] = \
                    self.output_angle
                self.previous_force_error = force_error
                # here i'm assuming joint angle 6 is basically the same
                # for all trjactories, which may or may not be true.

                difference_from_initial = self.output_angle - \
                    self.initial_trajectory_angle

                if difference_from_initial > 0.09:
                    self.get_logger().info("tilted too far forward, \
                                           replanning")
                    await self.replan_trajectory(False)
                    self.use_control_loop = False
                elif difference_from_initial < -0.09:
                    self.get_logger().info("tilted too far backward,\
                                           replannign")
                    await self.replan_trajectory(True)
                    self.use_control_loop = False

                self.pub.publish(self.joint_trajectories[0])
                self.joint_trajectories.pop(0)

            else:
                self.pub.publish(self.joint_trajectories[0])
                self.joint_trajectories.pop(0)

        # if we've reached the goal, send a message to draw.py that says we're
        # done.
        elif not self.joint_trajectories and self.state == State.PUBLISH:

            self.future.set_result("done")
            self.get_logger().info("done executing!!")

            self.state = State.STOP

        self.i += 1


def main(args=None):

    rclpy.init(args=args)

    executor = Executor()

    rclpy.spin(executor)


if __name__ == '__main__':
    main()
