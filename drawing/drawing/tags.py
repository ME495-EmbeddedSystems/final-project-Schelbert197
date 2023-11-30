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
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# from scipy.spatial.transform import Rotation
from brain_interfaces.srv import BoardTiles, MoveJointState
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose
from path_planner.path_plan_execute import Path_Plan_Execute
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from enum import Enum, auto
import modern_robotics as mr


class State(Enum):
    CALIBRATE = auto()
    OTHER = auto()


class Grid:
    def __init__(self, xrange, yrange, cell_size):

        self.xrange = xrange
        self.yrange = yrange
        self.cell_size = cell_size
        self.xnum = int(((self.xrange[1] - self.xrange[0]) / self.cell_size))
        self.ynum = int(((self.yrange[1] - self.yrange[0]) / self.cell_size))
        self.grid = np.zeros((self.ynum, self.xnum))

    def grid_to_world(self, mode, position):
        if mode == 0:
            point = (4, position+2)
        if mode == 1:
            point = (1, position+2)
        if mode == 2:
            if position == 0:
                point = (1, 0)
            if position == 1:
                point = (2, 1)
            if position == 2:
                point = (2, 0)
            if position == 3:
                point = (3, 0)
            if position == 4:
                point = (3, 1)

        point_x = (point[0])*self.cell_size + self.xrange[0]
        point_y = (point[1])*self.cell_size + self.yrange[0]
        return [point_x, point_y]


class Tags(Node):

    def __init__(self):
        super().__init__("tags")
        self.freq = 200.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(1 / self.freq, self.timer_callback)
        self.file_path_A = 'A.csv'
        self.file_path_B = 'B.csv'
        self.grid = Grid((0, .80), (0, .60), .10)
        self.path_planner = Path_Plan_Execute(self)
        self.state = State.OTHER
        self.execute_trajectory_status_callback_group = MutuallyExclusiveCallbackGroup()
        self.move_js_callback_group = MutuallyExclusiveCallbackGroup()

        # creating services
        self.record_service = self.create_service(
            Empty, 'record_transform', self.record_callback)
        self.calibrate_service = self.create_service(
            Empty, 'calibrate', self.calibrate_callback)
        self.where_to_write = self.create_service(
            BoardTiles, 'where_to_write', self.where_to_write_callback)

        # create publishers
        self.state_publisher = self.create_publisher(String, 'cal_state', 10)

        # create subscribers
        self.goal_reach_sub = self.create_subscription(
            String, 'execute_trajectory_status', self.goal_reach_sub_callback, 10, callback_group=self.execute_trajectory_status_callback_group)
        self.goal_state = "not"

        # create client
        self.move_js_client = self.create_client(
            MoveJointState, 'jointstate_mp', callback_group=self.move_js_callback_group)

        # making static transform
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transform()

        # making broadcast between board and robot
        self.broadcaster = TransformBroadcaster(self)
        self.robot_board = TransformStamped()
        self.robot_board.header.frame_id = "panda_link0"
        self.robot_board.child_frame_id = "board"
        self.robot_board.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.robot_board)

        
        self.robot_board_write = TransformStamped()
        self.robot_board_write.header.frame_id = "panda_link0"
        self.robot_board_write.child_frame_id = "point"
        self.robot_board_write.header.stamp = self.get_clock().now().to_msg()
        
        
        # wait for services
        while not self.move_js_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Move Joint State service not available, waiting again...')

    def goal_reach_sub_callback(self, msg):
        if msg == "done":
            self.goal_state = "done"

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

    def matrix_to_position_quaternion(self, matrix, point=0):
        translation = matrix[:3, 3]
        rotation_matrix = matrix[:3, :3]

        # Convert rotation matrix to quaternion using tf2
        quaternion = tf.quaternions.mat2quat(rotation_matrix)

        # Create Vector3 for position
        if point == 0:
            position = Vector3()
        elif point == 1:
            position = Point()
        position.x, position.y, position.z = translation

        # Create Quaternion for rotation
        rotation = Quaternion()
        rotation.w, rotation.x, rotation.y, rotation.z = quaternion

        return position, rotation

    def array_to_transform_matrix(self, translation, quaternion):
        # Normalize the quaternion
        quaternion /= np.linalg.norm(quaternion)
        quaternion = [quaternion[3], quaternion[0],
                      quaternion[1], quaternion[2]]

        # Create rotation matrix from quaternion
        rotation_matrix = tf.quaternions.quat2mat(quaternion)

        # Create the transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation

        return transform_matrix

    async def where_to_write_callback(self, request, response):
        self.get_logger().info("where_to_write1")
        response_a = []
        pos = Pose()
        self.get_logger().info("where_to_write2")
        ansT, ansR = self.get_transform('panda_link0', 'board')
        Trb = self.array_to_transform_matrix(ansT, ansR)
        lx, ly = self.grid.grid_to_world(request.mode, request.position)
        Tbl = np.array([[1, 0, 0, lx],
                        [0, 1, 0, ly],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        Trl = Trb @ Tbl

        x, y = request.x[0], request.y[0]
        z = 0.20
        
        Tla = np.array([[-0.03948997,  0.99782373,  0.05280484, x],
                            [ 0.06784999,  0.05540183, -0.99615612,  y],
                            [-0.9969137 , -0.03575537, -0.06989015,  z],
                            [ 0.  ,        0.  ,        0. ,         1.  ]]
                            )
        Tra = Trl @ Tla
        position, rotation = self.matrix_to_position_quaternion(Tra, 1)
        pos.position = position
        pos.orientation = rotation

        response.inital_pose = pos

        for i in range(len(request.x)):
            x, y = request.x[i], request.y[i]
            z = 0.2 if request.onboard[i] else 0.15

            # Tla = np.array([[0, 1, 0, x],
            #                 [0.5,  0.0 ,        -0.8660254, y],
            #                 [-0.8660254,  0.0  , -0.5, z],
            #                 [0, 0, 0, 1]])
            
            Tla = np.array([[-0.03948997,  0.99782373,  0.05280484, x],
                            [ 0.06784999,  0.05540183, -0.99615612,  y],
                            [-0.9969137 , -0.03575537, -0.06989015,  z],
                            [ 0.  ,        0.  ,        0. ,         1.  ]]
                            )
            Tra = Trl @ Tla
            position, rotation = self.matrix_to_position_quaternion(Tra, 1)
            pos.position = position
            pos.orientation = rotation

            response_a.append(pos)
        self.robot_board_write.transform.translation , self.robot_board_write.transform.rotation = self.matrix_to_position_quaternion(Tra)
        self.get_logger().info("where_to_write3")
        response.pose_list = response_a
        return response

    async def calibrate_callback(self, request, response):
        self.state = State.CALIBRATE
        goal_js = MoveJointState.Request()
        goal_js.joint_names = ["panda_joint4", "panda_joint5", "panda_joint7"]
        goal_js.joint_positions = [-2.61799, -1.04173, 2.11185]
        # ans = await self.move_js_client.call_async(goal_js)
        self.get_logger().info("calibrate")
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

    async def timer_callback(self):
        msg = String()
        # self.get_logger().info("timmer function")
        if self.state == State.CALIBRATE:
            # TODO: goto jointstate if reached then do this stuff

            ansT, ansR = self.get_transform('panda_link0', 'tag11')
            msg.data = "CALIBRATING"
            # if ansT[0] != 0.0 and self.goal_state == "done":
            if ansT[0] != 0.0:
                Ttb = np.array([[1, 0, 0, 0.063],
                                [0, 1, 0, 0.063],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
                # Ttb = np.array([0.063,0.063,0,1])
                Trt = self.array_to_transform_matrix(ansT, ansR)

                Trb = Trt @ Ttb
                self.get_logger().info(f'Trb: \n{Trb}')
                pos, rotation = self.matrix_to_position_quaternion(Trb)
                self.get_logger().info(f'Trt: \n{Trt}')
                self.get_logger().info(f'Trb: \n{Trb}')
                self.robot_board.transform.translation = pos
                self.robot_board.transform.rotation = rotation
                self.state = State.OTHER
                self.goal_state = "not"
                self.get_logger().info(f'Trb: {pos,rotation}')

        
        # ansTi, ansRi = self.get_transform('board', 'panda_hand_tcp')
        # pls = self.array_to_transform_matrix(ansTi, ansRi)
        # self.get_logger().info(f'{pls}')
        if self.state == State.OTHER:
            msg.data = "CALIBRATED"
        self.robot_board.header.stamp = self.get_clock().now().to_msg()
        self.robot_board_write.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.robot_board)
        self.broadcaster.sendTransform(self.robot_board_write)
        self.state_publisher.publish(msg)


def Tags_entry(args=None):
    rclpy.init(args=args)
    node = Tags()
    rclpy.spin(node)
    rclpy.shutdown()
