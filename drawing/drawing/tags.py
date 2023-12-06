""" This node deals with the april tags and handels tf tree.

1. Publishes a static transform between camera and the robot
2. Calibrate service: takes the arm to a specified pose and looks at the april
    tags on the board and publishes a board to robot transform.
3. Letter pose service: gives the start pose of any letter wrt to the
    panda_link0.
4. Update Trajectory service: Given a list of poses.
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose
from geometry_msgs.msg import TransformStamped
from brain_interfaces.srv import BoardTiles, MovePose, UpdateTrajectory
from drawing.grid import Grid, matrix_to_position_quaternion
from drawing.grid import array_to_transform_matrix
from enum import Enum, auto
import modern_robotics as mr
import numpy as np
import time

class State(Enum):
    """
    Declaring diffrent states the robot or brick can be in.

    two possible states
    """

    CALIBRATE = auto()
    OTHER = auto()


class Tags(Node):
    """Connects the robot with camera and real-world."""

    def __init__(self):
        super().__init__("tags")
        self.freq = 100.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.file_path_A = "A.csv"
        self.file_path_B = "B.csv"
        self.grid = Grid((0, 0.8), (0, 0.40), 0.1)
        self.state = State.OTHER
        self.move_js_callback_group = MutuallyExclusiveCallbackGroup()
        self.make_board_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_grp = MutuallyExclusiveCallbackGroup()
        self.calibrate_callback_grp = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(
            1/self.freq, self.timer_callback, callback_group=self.timer_cb_grp
        )
        # creating services
        self.record_service = self.create_service(
            Empty, "record_transform", self.record_callback
        )
        self.calibrate_service = self.create_service(
            Empty,
            "calibrate",
            self.calibrate_callback,
            callback_group=self.calibrate_callback_grp,
        )
        self.where_to_write = self.create_service(
            BoardTiles, "where_to_write", self.where_to_write_callback
        )
        self.update_trajectory = self.create_service(
            UpdateTrajectory, "update_trajectory", self.update_trajectory_cb
        )
        self.goal_state = "not"

        # create client
        self.move_js_client = self.create_client(
            MovePose, "moveit_mp", callback_group=self.move_js_callback_group
        )

        # making static transform
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transform()

        # making broadcast between board and robot
        self.broadcaster = TransformBroadcaster(self)
        self.robot_board = TransformStamped()
        self.robot_board.header.frame_id = "panda_link0"
        self.robot_board.child_frame_id = "board"
        self.robot_board.transform.translation.z = -0.9
        self.robot_board.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.robot_board)

        self.robot_board_write = TransformStamped()
        self.robot_board_write.header.frame_id = "panda_link0"
        self.robot_board_write.child_frame_id = "point"
        self.robot_board_write.header.stamp = self.get_clock().now().to_msg()

        # Transform to save the robot to board transform
        self.boardT = np.eye(4)

        # Create a new Future object.
        self.future = rclpy.task.Future()
        self.future_satate = rclpy.task.Future()
        # wait for services
        while not self.move_js_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Move Pose service not available, waiting")

    def make_transform(self):
        """Create a static transform between robot and the camera."""
        self.robot_to_camera = TransformStamped()
        self.robot_to_camera.header.stamp = self.get_clock().now().to_msg()
        self.robot_to_camera.header.frame_id = "panda_hand_tcp"
        self.robot_to_camera.child_frame_id = "camera_link"

        self.robot_to_camera.transform.translation = Vector3(
            x=0.03524146, y=-0.015, z=-0.043029
        )
        self.robot_to_camera.transform.rotation = Quaternion(
            x=7.0710676e-01, y=1.4401870e-04, z=7.0710676e-01, w=-1.4401870e-04
        )

        self.tf_static_broadcaster.sendTransform(self.robot_to_camera)

    async def calibrate_callback(self, request, response):
        """
        Search for tag 11 and create a board frame relative to that.

        First it moves to a position where camera can see the april tag.
        Then waits for an updated transform and creates a board transform.

        """
        self.state = State.CALIBRATE

        goal_js = MovePose.Request()
        goal_js.target_pose.position = Point(
            x=0.30744234834406486, y=-0.17674628233240325, z=0.5725350884705022
        )
        goal_js.target_pose.orientation = Quaternion(
            x=0.7117299678289105,
            y=-0.5285053338340909,
            z=0.268057323473255,
            w=0.37718408812611504,
        )
        goal_js.use_force_control = False
        # moving robot to calibrate position
        await self.move_js_client.call_async(goal_js)
        time.sleep(2)
        self.goal_state = "done"
        ansT, ansR = await self.future
        ansT1, ansT2 = ansT
        ansR1, ansR2 = ansR
        while ansT1[0] == 0.0 and ansT2[0]:
            ansT, ansR = await self.future
            ansT1, ansT2 = ansT
            ansR1, ansR2 = ansR
            self.get_logger().info("value set in service")
        Tt1b = np.array([[1, 0, 0, 0.05], [0, 1, 0, 0.05],
                        [0, 0, 1, 0], [0, 0, 0, 1]])
        Trt1 = array_to_transform_matrix(ansT1, ansR1)
        Trb1 = Trt1 @ Tt1b

        self.boardT = Trb1
        pos, rotation = matrix_to_position_quaternion(Trb1)

        self.robot_board.transform.translation = pos
        self.robot_board.transform.rotation = rotation

        self.state = State.OTHER
        self.goal_state = "not"
        return response

    async def where_to_write_callback(self, request, response):
        """
        Gives the pose of the end-effector to write a perticular letter.

        Args:
        ----
            mode (int): 0,1,2 based on right , wring letter or the drawing.
            position (int): poition of a letter in a perticular mode.
            x (float[]): list of x in jrajectory of a letter
            y (float[]): list of y in jrajectory of a letter
            onboard (bool[]): wether a point is on the board ot not

        Returns
        -------
            initial_pose: a standoff position for the start of letter off-board
            pose_list: list of poses to write a letter

        """
        self.get_logger().info("where_to_write1")
        response_a = []

        self.get_logger().info("where_to_write2")
        Trb = self.boardT
        lx, ly = self.grid.grid_to_world(request.mode, request.position)
        Tbl = np.array(
            [[1, 0, 0, lx * 0.667], [0, 1, 0, ly * 0.667], [0, 0, 1, 0], [0, 0, 0, 1]]
        )
        Trl = Trb @ Tbl

        x, y = request.x[0], request.y[0]
        z = 0.12

        Tla = np.array(
            [
                [-0.03948997, 0.99782373, 0.05280484, x],
                [0.06784999, 0.05540183, -0.99615612, y],
                [-0.9969137, -0.03575537, -0.06989015, z],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        pos = Pose()
        Tra = Trl @ Tla
        position, rotation = matrix_to_position_quaternion(Tra, 1)
        pos.position = position
        pos.orientation = rotation
        (
            self.robot_board_write.transform.translation,
            self.robot_board_write.transform.rotation,
        ) = matrix_to_position_quaternion(Tra)

        response.initial_pose = pos

        for i in range(len(request.x)):
            x, y = request.x[i], request.y[i]
            z = 0.004 if request.onboard[i] else 0.1

            Tla = np.array(
                [
                    [-0.03948997, 0.99782373, 0.05280484, x],
                    [0.06784999, 0.05540183, -0.99615612, y],
                    [-0.9969137, -0.03575537, -0.06989015, z],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
            Tra = Trl @ Tla
            position, rotation = matrix_to_position_quaternion(Tra, 1)
            pos = Pose()
            pos.position = position
            pos.orientation = rotation

            (
                self.robot_board_write.transform.translation,
                self.robot_board_write.transform.rotation,
            ) = matrix_to_position_quaternion(Tra)
            self.get_logger().info(f"pose is: {pos}")
            response_a.append(pos)
            self.get_logger().info(f"Now the list is: {response_a}")

        self.get_logger().info("where_to_write3")
        response.pose_list = response_a
        response.use_force_control = request.onboard
        return response

    def update_trajectory_cb(self, request, response):
        """
        Called for force-control, returns updated list of poses.

        Args:
        ----
            input_pose (Pose): list of poses
            input_pose (bool): wether to move towards or away from board

        Returns
        -------
            output_pose: updated list of poses

        """
        
        ansT, ansR = self.get_transform("board", "panda_hand_tcp")

        # positive z is out of the board
        if request.into_board:
            z = ansT[2] + 0.001
        else:
            z = ansT[2] - 0.003

        self.get_logger().info("reached update trajcetory callback")

        pose = request.input_pose
        self.get_logger().info(f"input pose: {pose}")

        # change the pose
        Trans_arr = [pose.position.x, pose.position.y, pose.position.z]
        Rot_arr = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        Tra = array_to_transform_matrix(Trans_arr, Rot_arr)
        Trb = self.boardT
        Tba = mr.TransInv(Trb) @ Tra
        new_Tba = Tba
        # new_Tba[2, 3] = z
        new_Tra = Trb @ new_Tba
        pos = Pose()
        position, rotation = matrix_to_position_quaternion(new_Tra, 1)
        pos.position = position
        pos.orientation = rotation

        response.output_pose = pos

        return response

    def record_callback(self, request, response):
        """
        Record the transforms to calibrate the camera to the robot.

        Args:
        ----
            matrix A : Transformation matrix from panda_link0 to panda_hand_tcp
            matrix B : Transformation matrix from camera_link to april tag

        """
        At, Aq = self.get_transform("panda_link0", "panda_hand_tcp")
        Bt, Bq = self.get_transform("camera_link", "tag56")

        A = array_to_transform_matrix(At, Aq)
        B = array_to_transform_matrix(Bt, Bq)
        self.write_matrix_to_file(self.file_path_A, A)
        self.write_matrix_to_file(self.file_path_B, B)

        return response

    def write_matrix_to_file(self, file_path, matrix):
        """Function to write matrices to a file."""
        with open(file_path, "a") as file:
            for row in matrix:
                file.write(",".join(map(str, row)) + "\n")
            file.write("---\n")

    def get_transform(self, parent_frame, child_frame):
        """
        Try catch block for Listning transforms between parent and child frame.

        Args:
        ----
            parent_frame (string): name of parent frame
            child_frame (string): name of child frame

        Returns
        -------
            trans: the x,y,z of the translational transform
            roration: the x,y,z,w of the rotational transform

        """
        try:
            trans = self.buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            transl = trans.transform.translation
            rot = trans.transform.rotation
            trans = [transl.x, transl.y, transl.z]
            rotation = [rot.x, rot.y, rot.z, rot.w]
            return trans, rotation

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
        """Publishes the panda_link0 to board transform constantly."""
        if self.state == State.CALIBRATE and self.goal_state == "done":
            ansT1, ansR1 = self.get_transform("panda_link0", "tag11")
            ansT2, ansR2 = self.get_transform("panda_link0", "tag12")
            # self.get_logger().info(f'{ansT, ansR}')
            self.future.set_result([[ansT1, ansT2], [ansR1, ansR2]])
        self.robot_board.header.stamp = self.get_clock().now().to_msg()
        self.robot_board_write.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.robot_board)
        self.broadcaster.sendTransform(self.robot_board_write)


def Tags_entry(args=None):
    rclpy.init(args=args)
    node = Tags()
    rclpy.spin(node)
    rclpy.shutdown()
