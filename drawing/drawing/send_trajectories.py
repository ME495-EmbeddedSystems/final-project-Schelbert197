import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String

from brain_interfaces.msg import EEForce

from brain_interfaces.srv import ExecuteJointTrajectories, Cartesian, Replan

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

        if request.state == "publish":
            self.state = State.PUBLISH
        else:
            self.state = State.STOP

        await self.future

        self.future = Future()

        return response

    def move_point_along_quaterion(self, point, quaternion, distance):
        # turn the quaternion into a unit quaternion
        # Normalize the quaternion
        quaternion /= np.linalg.norm(quaternion)
        quaternion = [quaternion[3], quaternion[0],
                      quaternion[1], quaternion[2]]

        # Create rotation matrix from quaternion
        rotation_matrix = tf.quaternions.quat2mat(quaternion)

        # Rotate the point
        rotated_point = np.dot(rotation_matrix, point)

        # Translate the point by the specific distance
        translated_point = rotated_point - distance * rotation_matrix[:, 2]

        self.get_logger().info(f"original point: {point}")
        self.get_logger().info(f"translated point: {translated_point}")

        return translated_point

    async def timer_callback(self):

        # if force is above threshold, stop executing.
        # self.get_logger().info(f"ee_force: {self.ee_force}")
        if self.ee_force > self.ee_force_threshold and self.use_force_control and self.joint_trajectories:
            self.get_logger().info(
                f"FORCE THRESHOLD EXCEEDED, EE_FORCE: {self.ee_force}")

            if len(self.poses) == 0:
                self.joint_trajectories.clear()
                self.get_logger().info("joint_trajectories cleared")
                return

            transform = self.buffer.lookup_transform(
                'panda_link0', 'panda_hand_tcp', rclpy.time.Time()
            )
            translation = transform.transform.translation
            current_point = np.array(
                [translation.x, translation.y, translation.z])
            attempted_point = np.array(
                [self.poses[0].position.x, self.poses[0].position.y, self.poses[0].position.z])

            self.distance = current_point - attempted_point

            new_poses = []

            for pose in self.poses:
                point = np.array(
                    [pose.position.x, pose.position.y, pose.position.z])
                quaternion = np.array([pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w])

                new_point = self.move_point_along_quaterion(
                    point, quaternion, self.distance)

                new_poses.append(Pose(position=Point(
                    x=float(new_point[0]), y=float(new_point[1]), z=float(new_point[2])),
                    orientation=pose.orientation))

            replan_response = await self.replan_client.call_async(Replan.Request(poses=new_poses))

            self.joint_trajectories = replan_response.joint_trajectories
            self.poses = new_poses

        # if list of waypoints is not empty, publish to the topic that executes
        # trajectories oof the panda
        elif len(self.joint_trajectories) != 0 and self.state == State.PUBLISH and self.i % 10 == 0:
            self.get_logger().info(f"publishing!!!!!!!!!!!!!!!")

            self.get_logger().info(
                f"joint_Trajectory[0]: {self.joint_trajectories[0]}")

            self.pub.publish(self.joint_trajectories[0])
            self.joint_trajectories.pop(0)
            if self.poses:
                self.poses.pop(0)

            msg = String(data="Executing Trajectory!")
            self.execute_trajectory_status_pub.publish(msg)

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
