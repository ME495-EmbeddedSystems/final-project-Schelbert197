import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


@pytest.mark.rostest
def generate_test_description():
    tags_action = Node(package="drawing",
                       executable="tags",
                       )
    return (
        LaunchDescription([
            tags_action,
            launch_testing.actions.ReadyToTest()
        ]),
        # These are extra parameters that get passed to the test functions
        {
            'tags': tags_action
        }
    )


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

    def test_static_transform(self, launch_service, tags, proc_output):
        buffer = Buffer()
        _ = TransformListener(buffer, self.node)
        proc_output.assertWaitFor(
            "Static Transform: panda_hand_tcp->camera_link", process=tags,
            timeout=15.0)
        rclpy.spin_once(self.node)
        xform = buffer.lookup_transform(
            "panda_hand_tcp", "camera_link", rclpy.time.Time())

        assert xform.transform.translation.x == 0.03524146
        assert xform.transform.translation.y == 0.015
        assert xform.transform.translation.z == -0.043029
        assert xform.transform.rotation.x == 7.07106765e-01
        assert xform.transform.rotation.y == 1.44018704e-04
        assert xform.transform.rotation.z == 7.07106768e-01
        assert xform.transform.rotation.w == -1.44018703e-04
