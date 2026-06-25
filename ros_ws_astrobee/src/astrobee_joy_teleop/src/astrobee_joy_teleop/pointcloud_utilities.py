import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs import point_cloud2


# -------------------------------------------------------------------------
# Conversion & preprocessing
# -------------------------------------------------------------------------

def pointcloud_to_array(pc_msg: PointCloud2):
    """
    Convert sensor_msgs/PointCloud2 to Numpy array of XYZ points.
    """
    gen = point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
    arr = np.array(list(gen))

    return arr


def array_to_pointcloud(header: Header, xyz: PointCloud2):

    return point_cloud2.create_cloud_xyz32(header, xyz.tolist())


def trim_to_box(points: np.array, box_min: list, box_max: list):
    """
    Remove points outside the axis-aligned box defined by box_min and box_max.
    """
    mask = np.all((points >= box_min) & (points <= box_max), axis=1)
    trimmed_points = points[mask]

    return trimmed_points


def remove_outliers(points: np.array):
    """
    Remove large outliers from the point set (e.g., using simple statistical thresholds).
    """
    # TODO: implement outlier removal

    return points


# -------------------------------------------------------------------------
# Cylinder fitting and pose/twist computation
# -------------------------------------------------------------------------

def fit_cylinder(points: np.array):
    """
    Fit a 3D line (axis) to the point cloud and estimate cylinder length
    by capping the ends along the axis.

    Return a structure containing everything needed to derive the pose:
    - center position of cylinder
    - orientation (unit direction vector of axis)
    - length estimate
    """
    # TODO: perform 3D linear regression / line fitting and length estimation
    cylinder_model = {
        'center': None,      # e.g., [x, y, z]
        'axis': None,        # e.g., [ax, ay, az] unit vector
        'length': None       # scalar
    }
    return cylinder_model


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
