import rclpy
from rclpy.node import Node

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint

from sensor_msgs.msg import JointState

from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Header


class IK_Testing(Node):
    def __init__(self):
        super().__init__("IK_Testing")

        self.callback_groupu = ReentrantCallbackGroup()

        self.joint_states_subs = self.node.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.joint_states = JointState()

        self.ik_client = self.create_client(
            GetPositionIK, 'compute_ik', callback_group=self.callback_group)

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'IK service not available, waiting again...')

        self.create_timer(3, self.timer_callback,
                          callback_group=self.callback_group)

    def joint_states_callback(self, msg):
        self.joint_state = msg
        self.joint_names = msg.name
        self.joint_position = msg.position

    async def timer_callback(self):
        self.position_request = PositionIKRequest()
        self.position_request.group_name = 'panda_manipulator'

        self.robot_state = RobotState()
        self.robot_state.joint_state = JointState()
        self.robot_state.joint_state.header = Header(stamp=self.node.get_clock().now().to_msg(),
                                                     frame_id='')
        self.robot_state.joint_state.name = self.joint_names
        self.robot_state.joint_state.position = self.joint_position

        self.position_request.robot_state = self.robot_state

        self.position_request.avoid_collisions = True
        self.position_request.timeout = 5

        self.ik_request = GetPositionIK.Request(self.position_request)

        self.solution = await self.ik_client.call_async(self.ik_request)
