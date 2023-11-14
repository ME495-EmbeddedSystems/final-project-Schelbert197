import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros


class Tags(Node):

    def __init__(self):
        super().__init__("tags")
        self.freq = 200.0

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.brick_height = self.get_transform("platform", "brick")[2]
        self.timer = self.create_timer(1 / self.freq, self.timer_callback)

    def get_transform(self, parent_frame, child_frame):
        """
        Try catch block for Listning transforms between parent and child frame.

        Args:
        ----
            parent_frame (string): name of parent frame
            child_frame (string): name of child frame

        Returns
        -------
            brick_to_platform: the x,y,z of the translational transform

        """
        try:
            trans = self.buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            transl = trans.transform.translation
            brick_to_platform = [transl.x, transl.y, transl.z]
            # print(brick_to_platform[2])
            return brick_to_platform

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f"Lookup exception: {e}")
            return [0.0, 0.0, 0.0]
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
            return [0.0, 0.0, 0.0]
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")
            return [0.0, 0.0, 0.0]

    def timer_callback(self):
        ans = self.get_transform('camera_link', 'tag36h11:1')
        self.get_logger().info(f'{ans}')


def Tags_entry(args=None):
    rclpy.init(args=args)
    node = Tags()
    rclpy.spin(node)
    rclpy.shutdown()
