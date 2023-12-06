from drawing.path_plan_execute import Path_Plan_Execute

from geometry_msgs.msg import Pose, Point, Quaternion
import rclpy
import unittest


class TestTagsTf(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_identity(self, tags, proc_output):

        pose = Pose(position=Point(x=0.0, y=1.0, z=0.0),
                    orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0))
        path_planner = Path_Plan_Execute(self.node)
        test_goal_pose = path_planner.set_goal_pose(pose)

        rclpy.spin_once(self.node)

        assert pose == test_goal_pose
