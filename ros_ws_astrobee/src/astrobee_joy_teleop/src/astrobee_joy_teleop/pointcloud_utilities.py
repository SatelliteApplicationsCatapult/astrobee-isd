import rospy

import numpy as np
# import pyransac3d

from tf import transformations

from std_msgs.msg import Header, Float32
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Vector3, Vector3Stamped, Quaternion, Pose, PoseStamped, Twist, TwistStamped
from sensor_msgs import point_cloud2


# -------------------------------------------------------------------------
# Conversion & preprocessing
# -------------------------------------------------------------------------

def pointcloud_msg_to_array(pc_msg: PointCloud2):
    """
    Convert sensor_msgs/PointCloud2 to Numpy array of XYZ points.
    """
    gen = point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
    arr = np.array(list(gen))

    return arr


def array_to_pointcloud_msg(header: Header, xyz: PointCloud2):

    return point_cloud2.create_cloud_xyz32(header, xyz.tolist())


def array_to_vector3_msg(header: Header, arr: np.ndarray):

    v_msg = Vector3Stamped()

    if np.size(arr) != 3:
        rospy.logwarn("Array size not 3, returning empty msg.")
        return v_msg

    v_msg.header = header
    # Unpack iterables
    v_msg.vector = Vector3(*arr)

    return v_msg


def float_to_float32_msg(f: float):

    f_msg = Float32()
    f_msg.data = f

    return f_msg


def pose_msg_to_array(pose: Pose):

    pos = [pose.position.x, pose.position.y, pose.position.z]
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    T = transformations.translation_matrix(pos) @ transformations.quaternion_matrix(q)

    return T


def array_to_pose_msg(t: np.ndarray):

    pos = transformations.translation_from_matrix(t)
    q = transformations.quaternion_from_matrix(t)

    pose = Pose()
    pose.position = Point(*pos)
    pose.orientation = Quaternion(*q)

    return pose


def twist_msg_to_array(twist: Twist):

    v_lin = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
    v_ang = np.array([twist.angular.x, twist.angular.y, twist.angular.z])

    T = np.concatenate((v_lin, v_ang))

    return T


def array_to_twist_msg(t: np.ndarray):

    v_lin = t[0:3]
    v_ang = t[3:6]

    twist = Twist()
    twist.linear = Vector3(*v_lin)
    twist.angular = Vector3(*v_ang)

    return twist


def transform_twist_msg(twist_old: Twist, R: np.ndarray):

    v_lin_old = np.array([twist_old.linear.x, twist_old.linear.y, twist_old.linear.z])
    v_ang_old = np.array([twist_old.angular.x, twist_old.angular.y, twist_old.angular.z])

    v_lin_new = R @ v_lin_old
    v_ang_new = R @ v_ang_old

    twist_new = Twist()
    twist_new.linear.x, twist_new.linear.y, twist_new.linear.z = v_lin_new
    twist_new.angular.x, twist_new.angular.y, twist_new.angular.z = v_ang_new
    return twist_new







def trim_to_box(points: np.ndarray, box_min: list, box_max: list):
    """
    Remove points outside the axis-aligned box defined by box_min and box_max.
    """
    mask = np.all((points >= box_min) & (points <= box_max), axis=1)
    trimmed_points = points[mask]

    return trimmed_points


# def remove_outliers(points: np.ndarray):
#     """
#     Remove large outliers from the point set (e.g., using simple statistical thresholds).
#     """
#     # TODO: implement outlier removal

#     return points


# -------------------------------------------------------------------------
# Line fitting and pose/twist computation
# -------------------------------------------------------------------------

# def fit_line(points: np.ndarray):
#     """
#     Fit a 3D line (axis) to the point cloud and estimate cylinder length
#     by capping the ends along the axis.
#     """

#     # cylinder = pyransac3d.Cylinder()
#     # center, axis, radius, inliers = cylinder.fit(points, thresh=0.2, maxIteration=100)

#     # return center, axis, radius


#     line = pyransac3d.Line()
#     A, B, inliers = line.fit(points, thresh=0.01, maxIteration=1000)

#     return A, B, inliers




def compute_pose_msg(cylinder_model, input_header):
    """
    Construct a geometry_msgs/PoseStamped describing the center of the cylinder
    and its orientation.

    - Position: cylinder_model['center']
    - Orientation: derived from cylinder_model['axis'] (convert to quaternion)
    """
    pose_msg = PoseStamped()
    # pose_msg.header.stamp = input_header.stamp  # or rospy.Time.now()
    # pose_msg.header.frame_id = self.output_frame
    # TODO: fill in pose_msg.pose.position.{x,y,z} from cylinder_model['center']
    # TODO: convert cylinder_model['axis'] to a quaternion and assign to pose_msg.pose.orientation

    return pose_msg


def compute_twist_msg(pose_msg):
    """
    Construct a geometry_msgs/TwistStamped representing the estimated linear
    and angular velocity of the cylinder based on a running average of
    past poses stored in self.pose_history.
    """
    twist_msg = TwistStamped()
    # twist_msg.header.stamp = pose_msg.header.stamp
    # twist_msg.header.frame_id = pose_msg.header.frame_id

    # TODO: compute linear and angular velocity using pose_msg and self.pose_history
    # and assign to twist_msg.twist.linear.{x,y,z} and twist_msg.twist.angular.{x,y,z}

    return twist_msg
