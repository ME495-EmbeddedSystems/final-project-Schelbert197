import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from std_msgs.msg import String, Float32

from joint_interfaces.msg import JointTrajectories
from brain_interfaces.msg import EEForce

from brain_interfaces.srv import ExecuteJointTrajectories

from enum import Enum, auto


class State(Enum):

    PUBLISH = auto()
    STOP = auto()


class Executor(Node):

    def __init__(self):

        super().__init__("Execute")

        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.joint_trajectories_callback_group = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(
            0.1, self.timer_callback, callback_group=self.timer_callback_group)
        self.pub = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.execute_trajectory_status_pub = self.create_publisher(
            String, '/execute_trajectory_status', 10)

        self.joint_trajectories_service = self.create_service(
            ExecuteJointTrajectories, '/joint_trajectories', self.joint_trajectories_callback, callback_group=self.joint_trajectories_callback_group)

        self.force_sub = self.create_subscription(
            EEForce, '/ee_force', self.force_callback, 10)
        self.joint_trajectories = []
        self.ee_force = 0
        self.ee_force_threshold = 3  # N
        self.state = None
        self.clear = False

        self.future = Future()

        self.i = 0

    def force_callback(self, msg):
        self.ee_force = msg.ee_force
        self.use_force_control = msg.use_force_control

    async def joint_trajectories_callback(self, request, response):
        self.get_logger().info("message received!")
        self.i = 0

        self.joint_trajectories += request.joint_trajectories
        # self.get_logger().info(
        #     f"joint_trajectories: {self.joint_trajectories}")
        self.clear = request.clear

        self.get_logger().info(f"request.state: {request.state}")

        if request.state == "publish":
            self.state = State.PUBLISH
        else:
            self.state = State.STOP

        await self.future

        return response

    def timer_callback(self):

        # if force is above threshold, stop executing.
        if self.ee_force > self.ee_force_threshold and not self.use_force_control:
            self.get_logger().info(
                f"FORCE THRESHOLD EXCEEDED, EE_FORCE: {self.ee_force}")
            self.joint_trajectories.clear()
            self.clear = False

        if self.clear:
            self.joint_trajectories.clear()
            self.clear = False

        # if list of waypoints is not empty, publish to the topic that executes
        # trajectories oof the panda
        elif len(self.joint_trajectories) != 0 and self.state == State.PUBLISH:
            self.get_logger().info(f"publishing!!!!!!!!!!!!!!!")

            self.get_logger().info(
                f"joint_Trajectory[0]: {self.joint_trajectories[0]}")

            self.pub.publish(self.joint_trajectories[0])
            self.joint_trajectories.pop(0)

            msg = String(data="Executing Trajectory!")
            self.execute_trajectory_status_pub.publish(msg)

        # if we've reached the goal, send a message to draw.py that says we're done.
        elif len(self.joint_trajectories) == 0 and self.state == State.PUBLISH:

            msg = String(data="done")
            self.execute_trajectory_status_pub.publish(msg)

            self.future.set_result("done")

            self.state = State.STOP


def main(args=None):

    rclpy.init(args=args)

    executor = Executor()

    rclpy.spin(executor)


if __name__ == '__main__':
    main()
