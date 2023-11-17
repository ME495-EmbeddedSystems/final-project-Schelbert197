import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from std_msgs.msg import String, Float32

from joint_interfaces.msg import JointTrajectories

from enum import Enum, auto


class State(Enum):

    PUBLISH = auto()
    STOP = auto()


class Executor(Node):

    def __init__(self):

        super().__init__("Execute")

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.pub = self.create_publisher(
            JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.talk_to_draw = self.create_publisher(String, '/chatting', 10)
        self.sub = self.create_subscription(
            JointTrajectories, '/joint_trajectories', self.joint_trajectories_callback, 10)
        self.force_sub = self.create_subscription(
            Float32, '/ee_force', self.force_callback, 10)
        self.joint_trajectories = []
        self.ee_force = 0
        self.state = None
        self.clear = False

    def force_callback(self, msg):

        self.ee_force = msg.data

    def joint_trajectories_callback(self, msg):
        self.get_logger().info("message received!")

        self.joint_trajectories += msg.joint_trajectories
        # self.get_logger().info(
        #     f"joint_trajectories: {self.joint_trajectories}")
        self.clear = msg.clear

        if msg.state == "publish":
            self.state = State.PUBLISH
        else:
            self.state = State.STOP

    def timer_callback(self):

        if self.ee_force > 1:
            self.joint_trajectories.clear()
            self.clear = False

        if self.clear:
            self.joint_trajectories.clear()
            self.clear = False

        elif len(self.joint_trajectories) != 0 and self.state == State.PUBLISH:

            self.pub.publish(self.joint_trajectories[0])
            self.joint_trajectories.pop(0)

        elif len(self.joint_trajectories) == 0 and self.state == State.PUBLISH:

            msg = String()
            msg.data = "done"

            self.talk_to_draw.publish(msg)

            self.state = State.STOP


def main(args=None):

    rclpy.init(args=args)

    executor = Executor()

    rclpy.spin(executor)


if __name__ == '__main__':
    main()
