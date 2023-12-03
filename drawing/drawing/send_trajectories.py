import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String

from brain_interfaces.msg import EEForce

from brain_interfaces.srv import ExecuteJointTrajectories, Cartesian, Replan, UpdateTrajectory

from enum import Enum, auto

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import transforms3d as tf


class State(Enum):

    PUBLISH = auto()
    STOP = auto()


class Executor(Node):

    def __init__(self):

        super().__init__("Execute")

        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.joint_trajectories_callback_group = MutuallyExclusiveCallbackGroup()
        self.replan_callback_group = MutuallyExclusiveCallbackGroup()
        self.update_trajectory_callback_group = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(
            0.01, self.timer_callback, callback_group=self.timer_callback_group)

        # create publishers
        self.pub = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)

        self.execute_trajectory_status_pub = self.create_publisher(
            String, '/execute_trajectory_status', 10)

        # create services
        self.joint_trajectories_service = self.create_service(
            ExecuteJointTrajectories, '/joint_trajectories', self.joint_trajectories_callback, callback_group=self.joint_trajectories_callback_group)

        # create clients
        self.replan_client = self.create_client(
            Replan, '/replan_path', callback_group=self.replan_callback_group)

        self.update_trajectory_client = self.create_client(UpdateTrajectory, 'update_trajectory',
                                                           callback_group=self.update_trajectory_callback_group)

        # create subscriptions
        self.force_sub = self.create_subscription(
            EEForce, '/ee_force', self.force_callback, 10)

        # these are used for computing the current location of the end-effector
        # using the tf tree.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.joint_trajectories = []
        self.poses = []
        self.ee_force = 0
        self.ee_force_threshold = 3.6  # N
        self.state = None

        self.distance = 0.01  # distance along quaternion to move
        self.replan = False

        self.future = Future()

        self.i = 0

    def force_callback(self, msg):
        self.ee_force = msg.ee_force
        self.use_force_control = msg.use_force_control

    async def joint_trajectories_callback(self, request, response):
        self.get_logger().info("message received!")
        self.i = 0

        self.joint_trajectories = request.joint_trajectories
        self.poses = request.poses
        self.get_logger().info(f"initial poses: {self.poses}")

        if len(self.poses) > 1:
            self.ee_force_threshold = 2.0

        if not self.use_force_control:
            self.replan = False

        if self.replan:
            update_trajectory_response = await self.update_trajectory_client.call_async(UpdateTrajectory.Request(input_poses=self.poses))
            self.poses = update_trajectory_response.output_poses

            replan_response = await self.replan_client.call_async(Replan.Request(poses=self.poses))

            self.joint_trajectories = replan_response.joint_trajectories

        if request.state == "publish":
            self.state = State.PUBLISH
        else:
            self.state = State.STOP

        await self.future

        self.future = Future()

        return response

    async def timer_callback(self):

        # if force is above threshold, stop executing.
        # self.get_logger().info(f"ee_force: {self.ee_force}")
        # if self.ee_force > self.ee_force_threshold and self.use_force_control and self.joint_trajectories:
        if self.ee_force > self.ee_force_threshold and self.use_force_control and self.joint_trajectories:
            self.get_logger().info(
                f"FORCE THRESHOLD EXCEEDED, EE_FORCE: {self.ee_force}")

            self.joint_trajectories.clear()
            self.replan = True
            self.ee_force_threshold = self.ee_force + 3
            self.get_logger().info("joint trajectories cleared")

        # if list of waypoints is not empty, publish to the topic that executes
        # trajectories oof the panda
        elif len(self.joint_trajectories) != 0 and self.state == State.PUBLISH and self.i % 10 == 0:
            self.get_logger().info(f"publishing!!!!!!!!!!!!!!!")

            # self.get_logger().info(
            #     f"joint_Trajectory[0]: {self.joint_trajectories[0]}")

            self.pub.publish(self.joint_trajectories[0])
            self.joint_trajectories.pop(0)
            # if self.poses:
            #     self.poses.pop(0)

            msg = String(data="Executing Trajectory!")
            self.execute_trajectory_status_pub.publish(msg)

            # if self.i % 50 == 0:
            #     self.allowed_to_replan = True

        # if we've reached the goal, send a message to draw.py that says we're done.
        elif len(self.joint_trajectories) == 0 and self.state == State.PUBLISH:

            msg = String(data="done")
            self.execute_trajectory_status_pub.publish(msg)

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
