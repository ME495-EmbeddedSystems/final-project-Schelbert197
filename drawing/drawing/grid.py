import numpy as np
from geometry_msgs.msg import Point, Quaternion, Vector3
import transforms3d as tf


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
            point = (0, position+2)

        if mode == 1:
            point = (2, position+2)

        if mode == 2:
            if position == 0:
                point = (2, 1)
            if position == 1:
                point = (1, 1)
            if position == 2:
                point = (1, 0)
            if position == 3:
                point = (0, 0)
            if position == 4:
                point = (0, 1)
        if mode == 3:
            point = (3, 0)

        point_y = (point[0])*self.cell_size + self.yrange[0]
        point_x = (point[1])*self.cell_size + self.xrange[0]
        if mode == 0 or mode == 1:
            point_x += .1*position
        return [point_x, point_y]


def matrix_to_position_quaternion(matrix, point=0):
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


def array_to_transform_matrix(translation, quaternion):
    # Normalize the quaternion
    quaternion /= np.linalg.norm(quaternion)
    quaternion = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]

    # Create rotation matrix from quaternion
    rotation_matrix = tf.quaternions.quat2mat(quaternion)

    # Create the transformation matrix
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation

    return transform_matrix
