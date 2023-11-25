""" This node deals with the april tags and handels tf tree.
1. Publishes a static transform between camera and the robot (Done)
2. Calibrate service: takes the arm to a specified pose and looks at the april tags on the board and publishes a board to robot transform.
3. Letter pose service: gives the start pose of any letter wrt to the panda_link0 
"""
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros
from std_srvs.srv import Empty
import numpy as np
import transforms3d as tf
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
# from scipy.spatial.transform import Rotation
# from brain_interfaces.srv  import BoardTiles
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose
from path_planner.path_plan_execute import Path_Plan_Execute
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class Grid:
    def __init__(self, xrange, yrange, cell_size):
       
        
        self.xrange = xrange
        self.yrange = yrange
        self.cell_size = cell_size
        self.xnum = int(((self.xrange[1] - self.xrange[0]) / self.cell_size))
        self.ynum = int(((self.yrange[1] - self.yrange[0]) / self.cell_size))
        self.grid = np.zeros((self.ynum, self.xnum))

    def grid_to_world(self, point):
        point_x = (point[0])*self.cell_size + self.xrange[0]
        point_y = (point[1])*self.cell_size + self.yrange[0]
        return [point_x, point_y]


class Tags(Node):

    def __init__(self):
        super().__init__("tags")
        self.freq = 200.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        # self.brick_height = self.get_transform("platform", "brick")[2]
        self.timer = self.create_timer(1 / self.freq, self.timer_callback)
        self.file_path_A = 'A.csv'
        self.file_path_B = 'B.csv'
        self.grid = Grid((0, 80), (0, 60), 10)
        self.path_planner = Path_Plan_Execute(self)

        # creating services
        self.record_service = self.create_service(
            Empty, 'record_transform', self.record_callback)
        self.calibrate_service = self.create_service(
            Empty, 'calibrate', self.calibrate_callback)
        self.where_to_write = self.create_service(Empty,'where_to_write',self.where_to_write_callback)

        # making static transform
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transform()

        # ([0.4705757399661665, -0.7141633025201061, -0.006788132506325206], [-0.07728659290456086, 0.7088900493225068, 0.7007328395121232, -0.021798352185783326])

    
    
    
    def make_transform(self):

        self.robot_to_camera = TransformStamped()
        self.robot_to_camera.header.stamp = self.get_clock().now().to_msg()
        self.robot_to_camera.header.frame_id = "panda_hand_tcp"
        self.robot_to_camera.child_frame_id = "camera_link"

        self.robot_to_camera.transform.translation = Vector3(
            x=0.03524146, y=-0.015, z=-0.043029)
        self.robot_to_camera.transform.rotation = Quaternion(
            x=7.07106765e-01, y=1.44018704e-04, z=7.07106768e-01, w=-1.44018703e-04)
        # self.get_logger().info(type(self))
        self.tf_static_broadcaster.sendTransform(self.robot_to_camera)


    def transform_point(point, transformation_matrix):
        # Append a 1 to the point to make it homogeneous
        point_homogeneous = np.append(point, 1)

        # Apply the transformation matrix to the point
        transformed_point_homogeneous = np.dot(transformation_matrix, point_homogeneous)

        # Normalize the homogeneous coordinates
        transformed_point = transformed_point_homogeneous[:3] / transformed_point_homogeneous[3]

        return transformed_point

    def array_to_transform_matrix(self, translation, quaternion):
        # Normalize the quaternion
        quaternion /= np.linalg.norm(quaternion)

        # Create rotation matrix from quaternion
        rotation_matrix = tf.quaternions.quat2mat(quaternion)

        # Create the transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation

        return transform_matrix

    async def where_to_write_callback(self, request, response):
        self.path_planner.set_goal_pose(
            Point(x=0.3405757399661665, y=-0.6041633025201061, z=0.0))
        self.path_planner.set_goal_orientation(
            Quaternion(x=-0.5744922163820316, y=0.577965619924962,z= -0.3987692152610244,w= -0.42059190251516054))
        await self.path_planner.plan_path()
        # await self.path_planner.get_goal_joint_states()
        # self.path_planner.plan_path()
        return response

    async def calibrate_callback(self, request, response):
        ansT, ansR = self.get_transform('panda_link0', 'panda_hand_tcp')
        self.get_logger().info(f'Tbh: {ansT,ansR}')
        tags= ["tag11", "tag12", "tag13", "tag14"]
        
        # self.path_planner.set_goal_pose(Point(x=0.3091756571687885, y=-0.19003221403590845, z=0.4853135925515006))
        # self.path_planner.set_goal_orientation(Quaternion(x=0.7014271479691266, y=-0.4789569982144275, z=0.29657513433635807, w=0.43662723191149333))
        # await self.path_planner.plan_and_execute_path()
        # if self.path_planner.movegroup_result is not None:
        for tag in tags:
            ansT, ansR = self.get_transform('panda_link0', tag)
            self.get_logger().info(f'T panda0, {tag}: {ansT,ansR}')
            
        # Tbh: ([], [])
        # ([0.350475065643078, -0.636214306408102, 0.09925000881605822], [-0.5744922163820316, 0.577965619924962, -0.3987692152610244, -0.42059190251516054])

        return response

    def record_callback(self, request, response):
        At, Aq = self.get_transform('panda_link0', 'panda_hand_tcp')
        # self.get_logger().info(f'Tba: {ans1}')
        Bt, Bq = self.get_transform('camera_link', 'tag56')
        # self.get_logger().info(f'Tca: {ans2}')

        A = self.array_to_transform_matrix(At, Aq)
        B = self.array_to_transform_matrix(Bt, Bq)
        self.write_matrix_to_file(self.file_path_A, A)
        self.write_matrix_to_file(self.file_path_B, B)

        return response

    # Function to read matrices from a file
    def read_matrices_from_file(self, file_path):
        matrices = []
        delimiter = '---'

        with open(file_path, 'r') as file:
            matrix_strings = file.read().split(delimiter)

            for matrix_string in matrix_strings:
                if matrix_string.strip():
                    matrix_lines = matrix_string.strip().split('\n')
                    matrix_values = [
                        list(map(float, line.strip().split(','))) for line in matrix_lines]
                    matrices.append(np.array(matrix_values))

        return matrices

    # Function to write matrices to a file
    def write_matrix_to_file(self, file_path, matrix):
        with open(file_path, 'a') as file:
            for row in matrix:
                file.write(','.join(map(str, row)) + '\n')
            file.write('---\n')

    # File path for your file

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
            rot = trans.transform.rotation
            brick_to_platform = [transl.x, transl.y, transl.z]
            rotation = [rot.x, rot.y, rot.z, rot.w]

            # print(brick_to_platform[2])
            return brick_to_platform, rotation

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

    def timer_callback(self):
        pass
        # ansT, ansR = self.get_transform('panda_link0', 'panda_hand_tcp')
        # self.get_logger().info(f'Tbh: {ansT,ansR}')
        # ansT, ansR = self.get_transform('camera_link', 'tag56')
        # self.get_logger().info(f'Tca: {ansT,ansR }')


def Tags_entry(args=None):
    rclpy.init(args=args)
    node = Tags()
    rclpy.spin(node)
    rclpy.shutdown()
