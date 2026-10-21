#!/usr/bin/env python
import rospy
import rosbag
import rosparam

#import ros_numpy as rnp
#from tf2_geometry_msgs import do_transform_pose

from std_msgs.msg import Header, Float32
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TwistStamped, Vector3, Vector3Stamped
from visualization_msgs.msg import Marker

import numpy as np
# import eigenpy
from scipy.spatial.transform import Rotation

import pyransac3d

from copy import deepcopy
from astrobee_joy_teleop import pointcloud_utilities as pu


class PoseExtractor:
    def __init__(self):

        # Parameters
        self.bag_path = rospy.get_param('~bag_path')                                 # Required, folder
        self.common_params_yaml = rospy.get_param('~common_params_yaml')             # Required, file
        self.tool_name = rospy.get_param('~tool_name', 'ratchet_wrench')
        self.robot_name = rospy.get_param('~robot_name', 'honey')
        self.pc_frame = rospy.get_param('~pc_frame', 'honey/perch_cam')
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/pointcloud')

        self.box_min = rospy.get_param('~box_min', [-0.25, -0.25, 0.0])              # Box region limits [x_min, y_min, z_min]
        self.box_max = rospy.get_param('~box_max', [ 0.25,  0.25, 0.5])              # Box region limits [x_max, y_max, z_max]
        self.max_history_len = rospy.get_param('~max_history_len', 10)
        self.playback_rate = rospy.get_param('~playback_rate', 2.0)
        self.loop_playback = rospy.get_param('~loop_playback', True)

        # Ground truth midpoint: distance along the tool's own Z from its origin to the middle of its length [m].
        # (z_min + z_max) / 2 of the tool mesh. 0.0 means the tool origin is used.
        self.tool_z_mid = rospy.get_param('~tool_z_mid', 0.0)

        # Axis sign tracking
        self.startup_frames = rospy.get_param('~startup_frames', 10)                 # Frames voted over to fix the sign convention
        self.line_fit_thresh_deg = rospy.get_param('~line_fit_thresh_deg', 45.0)     # Reject a fit further than this from the previous axis
        self.max_rejections = rospy.get_param('~max_rejections', 5)                  # Consecutive rejections before forcing a re-lock

        # Derived values - To avoid continuously calculating
        self.cos_gate = np.cos(np.radians(self.line_fit_thresh_deg))

        # Publishers
        self.groundtruth_pose_pub = rospy.Publisher('~groundtruth_pose', PoseStamped, queue_size=10)
        self.groundtruth_pose_viz_pub = rospy.Publisher('~groundtruth_pose_viz', PoseStamped, queue_size=10)
        self.groundtruth_twist_pub = rospy.Publisher('~groundtruth_twist', TwistStamped, queue_size=10)

        self.original_pub = rospy.Publisher('~cloud_original', PointCloud2, queue_size=10)
        self.filtered_viz_pub = rospy.Publisher('~cloud_filtered_viz', PointCloud2, queue_size=10)
        self.linefit_viz_pub = rospy.Publisher('~cloud_linefit_viz', PointCloud2, queue_size=10)
        self.shape_viz_pub = rospy.Publisher("~cylinder_shape_viz", Marker, queue_size=10)

        # Pose: midpoint position, orientation with Z along the tracked axis. Roll about the axis is a convention, not measured.
        # Twist: linear velocity, plus angular velocity perpendicular to the axis (along-axis component is 0, unobservable).
        self.estimated_pose_pub = rospy.Publisher('~cylinder_pose', PoseStamped, queue_size=10)
        self.estimated_twist_pub = rospy.Publisher('~cylinder_twist', TwistStamped, queue_size=10)
        self.groundtruth_axis_pub = rospy.Publisher('~groundtruth_axis', Vector3Stamped, queue_size=10)
        self.fit_axis_pub = rospy.Publisher('~fit_axis', Vector3Stamped, queue_size=10)
        self.groundtruth_error_deg_pub = rospy.Publisher('~groundtruth_error_deg', Float32, queue_size=10)


        # Load params
        params = rosparam.load_file(self.common_params_yaml)[0][0]

        # Static transform from robot body to perch cam
        tf_params = params['robot_sim']['robot_in_perch_tf']
        self.robot_in_perch_tf = pu.pose_msg_to_array(
                    Pose(position=Point(*tf_params['position']),
                         orientation=Quaternion(*tf_params['orientation'])))


        # Internal state for velocity estimation (least-squares slope over past frames)
        self.pose_history = []    # List of (stamp [s], centre (3,), axis (3,)), accepted frames only

        # Axis sign tracking state
        self.startup_buffer = []  # (stamp, centre, raw axis) until the startup vote completes
        self.axis_prev = None     # Last accepted, sign-resolved axis; None until the vote completes
        self.rejections = 0       # Consecutive rejected frames

        # List of all frames for playback visualisation
        self.viz_frames = []


        # Run pose estimation
        self.run_estimate()

        # Play back results with original pointcloud data and overlaid estimation results (visualise in RViz)
        self.play_results()


    def run_estimate(self):
        """
        Main processing loop: iterate over PointCloud2 messages in the bag,
        perform pre-processing and estimation, and publish pose/twist.
        """

        groundtruth_pose_msg = PoseStamped()
        groundtruth_pose_viz_msg = PoseStamped()
        groundtruth_twist_msg = TwistStamped()
        noisy_cloud_msg = PointCloud2()
        filtered_cloud_viz_msg = PointCloud2()
        linefit_cloud_viz_msg = PointCloud2()
        estimated_pose_msg = PoseStamped()
        estimated_twist_msg = TwistStamped()


        results_offset_y = 1.0


        bag = rosbag.Bag(self.bag_path, 'r')
        try:
            for topic, msg, t in bag.read_messages(topics=["/gazebo/model_states", self.pointcloud_topic]):
                if rospy.is_shutdown():
                    break


                if topic == "/gazebo/model_states":

                    # All model states from gazebo are in World frame
                    # Put everything in pointcloud frame

                    # Ground truth
                    header = Header()
                    header.frame_id = self.pc_frame
                    header.stamp = t

                    robot_idx = msg.name.index(self.robot_name)
                    tool_idx = msg.name.index(self.tool_name)

                    # Convert

                    robot_in_world_tf = pu.pose_msg_to_array(msg.pose[robot_idx])
                    tool_in_world_tf = pu.pose_msg_to_array(msg.pose[tool_idx])

                    tool_in_robot_tf = np.linalg.inv(robot_in_world_tf) @ tool_in_world_tf
                    tool_in_pc_tf = self.robot_in_perch_tf @ tool_in_robot_tf


                    # Z is tool's long axis (reminder to keep this convention on all tools!)

                    # Calculate: The groundtruth direction of the tool’s long Z axis in the pointcloud frame

                    R_gt = tool_in_pc_tf[:3, :3]  # Rotation matrix
                    axis_gt = R_gt[:, 2]  # tool local Z axis
                    axis_gt = axis_gt / np.linalg.norm(axis_gt)  # Normalise vector


                    # if axis_gt[1] < 0:
                    #     axis_gt = -axis_gt


                    # Ground truth midpoint, to match what the estimator measures
                    mid_gt = tool_in_pc_tf[:3, 3] + self.tool_z_mid * axis_gt

                    # Ground truth pose: midpoint position, real orientation
                    groundtruth_pose_msg.header = header
                    groundtruth_pose_msg.pose = pu.array_to_pose_msg(tool_in_pc_tf)
                    groundtruth_pose_msg.pose.position = Point(*mid_gt)

                    # Copy with offset, for visualisation only (sits beside the cylinder marker in RViz)
                    groundtruth_pose_viz_msg = deepcopy(groundtruth_pose_msg)
                    groundtruth_pose_viz_msg.pose.position.y += (results_offset_y * 2)

                    # Twist of the tool relative to the camera, in pc axes.
                    # model_states twists are world-frame, at each model's origin.
                    # The camera is rigid on the robot, so subtract the velocity of the robot-fixed point
                    # currently at the tool's position: v_robot + w_robot x (p_tool - p_robot).
                    tw_tool = msg.twist[tool_idx]
                    tw_robot = msg.twist[robot_idx]
                    v_tool = np.array([tw_tool.linear.x, tw_tool.linear.y, tw_tool.linear.z])
                    w_tool = np.array([tw_tool.angular.x, tw_tool.angular.y, tw_tool.angular.z])
                    v_robot = np.array([tw_robot.linear.x, tw_robot.linear.y, tw_robot.linear.z])
                    w_robot = np.array([tw_robot.angular.x, tw_robot.angular.y, tw_robot.angular.z])

                    lever_w = tool_in_world_tf[:3, 3] - robot_in_world_tf[:3, 3]
                    v_rel_w = v_tool - v_robot - np.cross(w_robot, lever_w)
                    w_rel_w = w_tool - w_robot

                    # world -> pc rotation: p_pc = world_in_pc_rot @ p_w
                    world_in_pc_rot = self.robot_in_perch_tf[:3, :3] @ robot_in_world_tf[:3, :3].T
                    v_rel_pc = world_in_pc_rot @ v_rel_w
                    w_rel_pc = world_in_pc_rot @ w_rel_w

                    # Match the estimator: velocity of the midpoint, not the tool origin,
                    # and no spin about the long axis (the line fit cannot see it)
                    v_mid_pc = v_rel_pc + np.cross(w_rel_pc, self.tool_z_mid * axis_gt)
                    w_perp_pc = w_rel_pc - np.dot(w_rel_pc, axis_gt) * axis_gt

                    groundtruth_twist_msg.header = header
                    groundtruth_twist_msg.twist.linear = Vector3(*v_mid_pc)
                    groundtruth_twist_msg.twist.angular = Vector3(*w_perp_pc)



                elif topic == self.pointcloud_topic:

                    # Convert PointCloud2 to an internal point list / array
                    points = pu.pointcloud_msg_to_array(msg)

                    # Add noise here
                    gaussian_noise = np.random.normal(0.0 , 0.01, points.shape)  # mean, std, output size
                    noisy_points = points + gaussian_noise

                    # Trim points outside the defined box region
                    #trimmed_points = pu.trim_to_box(noisy_points, self.box_min, self.box_max)
                    filtered_points = pu.trim_to_box(noisy_points, self.box_min, self.box_max)

                    print(len(filtered_points))

                    # Remove big outliers
                    # filtered_points = pu.remove_outliers(trimmed_points)


                    # 3D linear regression fit to estimate cylinder axis and length (fit line, then cap ends)
                    # cyl_centre, cyl_axis, cyl_radius = pu.fit_cylinder(filtered_points)

                            # cyl_rot, _ = Rotation.align_vectors(cyl_axis.reshape(1,3), np.array([[0,0,1]]))
                            # cyl_rot, _ = Rotation.align_vectors(np.vstack([cyl_axis, [0,0,1]]), np.vstack([[0,0,1], [0,0,1]]))
                            # cyl_q = cyl_rot.as_quat()  # q xyzw

                            # q = eigenpy.Quaternion.FromTwoVectors(np.array([0.0, 0.0, 1.0]), cyl_axis)

                            # ROS quaternion (x, y, z, w):
                            # cyl_q = np.array([q.x, q.y, q.z, q.w])

                    # up_vec = np.array([0.0, 0.0, 1.0])
                    # right_vec = np.cross(cyl_axis, up_vec)
                    # right_norm = np.linalg.norm(right_vec)
                    # if right_norm > 1e-8:
                    #     right_vec = right_vec / right_norm
                    # else:
                    #     # Axis nearly parallel to up
                    #     right_vec = np.array([1.0, 0.0, 0.0])

                    # angle = -np.arccos(np.clip(np.dot(cyl_axis, up_vec), -1.0, 1.0))

                    # cyl_q = Rotation.from_rotvec(right_vec * angle).as_quat()  # q xyzw


                    # RANSAC line fit
                    line = pyransac3d.Line()
                    line_slope, line_intercept, line_inliers = line.fit(filtered_points, thresh=0.02, maxIteration=1000)


                    # Axis direction (from line fit), sign arbitrary
                    axis_raw = line_slope / np.linalg.norm(line_slope)


                    # if axis_fit[1] < 0:
                    #     axis_fit = -axis_fit




                    # Centre point on the axis
                    # inlier_pts = filtered_points[line_inliers]  # (N, 3)
                    # cyl_centre = inlier_pts.mean(axis=0)

                    # Use inliers only to estimate length along the axis
                    inlier_pts = filtered_points[line_inliers]  # (N, 3)

                    # Project inliers onto axis, using line_intercept as origin
                    # Length and centre do not depend on the axis sign, so the raw axis is fine here
                    p = (inlier_pts - line_intercept) @ axis_raw  # dot product of vector v with unit vector axis
                    # p_min, p_max = np.percentile(p, [5, 95])
                    p_min = p.min()
                    p_max = p.max()

                    # Get cylinder length, start, end, and centre
                    cyl_length = p_max - p_min
                    axis_start = line_intercept + p_min * axis_raw
                    axis_end   = line_intercept + p_max * axis_raw
                    cyl_centre = 0.5 * (axis_start + axis_end)


                    # Resolve the 180 deg ambiguity of the line fit.
                    # None during the startup vote; the last accepted axis if this fit is rejected.
                    axis_fit = self.track_axis(msg.header.stamp.to_sec(), cyl_centre, axis_raw)

                    # Marker only: a cylinder looks the same either way up, so draw the raw fit during startup
                    axis_draw = axis_raw if axis_fit is None else axis_fit


                    # Calculate orientation quaternion (q xyzw)
                    # Smallest rotation taking pc +Z onto the axis. Roll about the axis is this convention, not a measurement.
                    z_axis = np.array([0.0, 0.0, 1.0])
                    axis_dot = np.dot(z_axis, axis_draw)
                    axis_cross = np.cross(z_axis, axis_draw)
                    axis_cross_norm = np.linalg.norm(axis_cross)


                    if axis_cross_norm < 1e-6:
                        if axis_dot > 0.0:
                            # Axis pointing towards +z
                            cyl_q = np.array([0.0, 0.0, 0.0, 1.0])
                        else:
                            # Axis pointing towards -z (180 deg flip about X)
                            cyl_q = np.array([1.0, 0.0, 0.0, 0.0])
                    else:
                        rot_axis = axis_cross / axis_cross_norm
                        rot_angle = np.arccos(np.clip(axis_dot, -1.0, 1.0))
                        rot_vec = rot_axis * rot_angle
                        cyl_q = Rotation.from_rotvec(rot_vec).as_quat()




                    marker_viz = Marker()
                    marker_viz.header.frame_id = self.pc_frame
                    marker_viz.header.stamp = t
                    marker_viz.ns = "fitted_shapes"
                    marker_viz.id = 0
                    marker_viz.type = 3    # CYLINDER
                    # marker_viz.type = 0    # ARROW
                    marker_viz.action = 0  # ADD

                    marker_viz.pose.position.x = cyl_centre[0]
                    marker_viz.pose.position.y = cyl_centre[1] + (results_offset_y * 2)
                    marker_viz.pose.position.z = cyl_centre[2]
                    marker_viz.pose.orientation.x = cyl_q[0]
                    marker_viz.pose.orientation.y = cyl_q[1]
                    marker_viz.pose.orientation.z = cyl_q[2]
                    marker_viz.pose.orientation.w = cyl_q[3]

                    marker_viz.scale.x = 0.1
                    marker_viz.scale.y = 0.1
                    marker_viz.scale.z = cyl_length

                    # marker_viz.pose.position.x = cyl_centre[0]
                    # marker_viz.pose.position.y = cyl_centre[1] + results_offset_y
                    # marker_viz.pose.position.z = cyl_centre[2]
                    # marker_viz.pose.orientation.x = cyl_q[0]
                    # marker_viz.pose.orientation.y = cyl_q[1]
                    # marker_viz.pose.orientation.z = cyl_q[2]
                    # marker_viz.pose.orientation.w = cyl_q[3]

                    # marker_viz.scale.x = 0.3
                    # marker_viz.scale.y = 0.02
                    # marker_viz.scale.z = 0.02

                    marker_viz.color.r = 0.0
                    marker_viz.color.g = 1.0
                    marker_viz.color.b = 0.0
                    marker_viz.color.a = 0.5








                    # optional geometric residual of inliers to fitted line
                    #inlier_pts = filtered_points[line_inliers]
                    #d = np.linalg.norm(np.cross(inlier_pts - line_intercept, axis_fit), axis=1)
                    #mean_line_residual = d.mean()

                    #rospy.loginfo("axis err = %.3f deg, mean residual = %.4f m", axis_angle_err_deg, mean_line_residual)





                    header = Header()
                    header.frame_id = self.pc_frame
                    header.stamp = t




                    # Convert to messages
                    axis_gt_msg = pu.array_to_vector3_msg(header, axis_gt)

                    # Nothing to report until the startup vote completes
                    axis_fit_msg = None
                    axis_angle_err_deg_msg = None
                    if axis_fit is not None:
                        axis_fit_msg = pu.array_to_vector3_msg(header, axis_fit)

                        # Signed error. The convention is arbitrary per run, so a run sitting near 180 deg throughout is fine;
                        # a jump between ~0 and ~180 mid-run is a sign flip.
                        dot_val = np.clip(np.dot(axis_fit, axis_gt), -1.0, 1.0)
                        axis_angle_err_deg = np.degrees(np.arccos(dot_val))
                        axis_angle_err_deg_msg = pu.float_to_float32_msg(axis_angle_err_deg)





                    # Estimated pose: midpoint, no display offset. Only the Z column of the orientation is measured.
                    estimated_pose_msg = None
                    if axis_fit is not None:
                        estimated_pose_msg = PoseStamped()
                        estimated_pose_msg.header = header
                        estimated_pose_msg.pose.position = Point(*cyl_centre)
                        estimated_pose_msg.pose.orientation = Quaternion(*cyl_q)

                    # Twist from a least-squares slope over the accepted-frame history (filled by track_axis)
                    estimated_twist_msg = None
                    if len(self.pose_history) >= 2:
                        v_est, w_est = self.estimate_twist()
                        estimated_twist_msg = TwistStamped()
                        estimated_twist_msg.header = header
                        estimated_twist_msg.twist.linear = Vector3(*v_est)
                        estimated_twist_msg.twist.angular = Vector3(*w_est)

                    # Turn noisy data into pointcloud message
                    noisy_cloud_msg = pu.array_to_pointcloud_msg(header, noisy_points)

                    # In-place shift to display better in RViz (fine as we don't need them anymore)
                    filtered_points[:, 1] += results_offset_y
                    # Turn filtered data into pointcloud message
                    filtered_cloud_viz_msg = pu.array_to_pointcloud_msg(header, filtered_points)

                    # Do the same with inlier points
                    inlier_pts[:, 1] += (results_offset_y * 2)
                    linefit_cloud_viz_msg = pu.array_to_pointcloud_msg(header, inlier_pts)


                    self.viz_frames.append({
                        'groundtruth_pose': deepcopy(groundtruth_pose_msg),
                        'groundtruth_pose_viz': deepcopy(groundtruth_pose_viz_msg),
                        'groundtruth_twist': deepcopy(groundtruth_twist_msg),
                        'original_cloud': deepcopy(noisy_cloud_msg),
                        'filtered_cloud_viz': deepcopy(filtered_cloud_viz_msg),
                        'linefit_cloud_viz': deepcopy(linefit_cloud_viz_msg),
                        'cylinder_viz': deepcopy(marker_viz),
                        'estimated_pose': deepcopy(estimated_pose_msg),
                        'estimated_twist': deepcopy(estimated_twist_msg),
                        'groundtruth_axis': deepcopy(axis_gt_msg),
                        'fit_axis': deepcopy(axis_fit_msg),
                        'groundtruth_error_deg': deepcopy(axis_angle_err_deg_msg),
                    })




        finally:
            bag.close()


        rospy.loginfo("Processed %d frames", len(self.viz_frames))




    def play_results(self):

        rospy.loginfo("Playback started, at %.1f Hz", self.playback_rate)

        rate = rospy.Rate(self.playback_rate)
        i = 0
        while self.viz_frames and not rospy.is_shutdown():

            rospy.loginfo("Frame: %d", i)

            frame = self.viz_frames[i]

            self.groundtruth_pose_pub.publish(frame['groundtruth_pose'])
            self.groundtruth_pose_viz_pub.publish(frame['groundtruth_pose_viz'])
            self.groundtruth_twist_pub.publish(frame['groundtruth_twist'])
            self.original_pub.publish(frame['original_cloud'])
            self.filtered_viz_pub.publish(frame['filtered_cloud_viz'])
            self.linefit_viz_pub.publish(frame['linefit_cloud_viz'])
            self.groundtruth_axis_pub.publish(frame['groundtruth_axis'])
            self.shape_viz_pub.publish(frame['cylinder_viz'])

            # Absent during the startup vote
            if frame['estimated_pose'] is not None:
                self.estimated_pose_pub.publish(frame['estimated_pose'])
            if frame['estimated_twist'] is not None:
                self.estimated_twist_pub.publish(frame['estimated_twist'])
            if frame['fit_axis'] is not None:
                self.fit_axis_pub.publish(frame['fit_axis'])
            if frame['groundtruth_error_deg'] is not None:
                self.groundtruth_error_deg_pub.publish(frame['groundtruth_error_deg'])

            i += 1
            if i >= len(self.viz_frames):
                if self.loop_playback:
                    i = 0
                else:
                    break

            rate.sleep()

        rospy.loginfo("Playback finished")




    def track_axis(self, stamp, centre, axis):
        """
        Fix the sign of a line-fit axis, causally (online-safe).
        Returns None until the startup vote completes, then the signed axis, or the last accepted axis if this fit is gated out.
        Accepted frames are added to the twist history.
        """
        if self.axis_prev is None:
            self.startup_buffer.append((stamp, centre, axis))
            if len(self.startup_buffer) < self.startup_frames:
                return None
            return self.lock_convention()

        dot = np.dot(axis, self.axis_prev)

        if abs(dot) < self.cos_gate:
            if self.rejections < self.max_rejections:
                self.rejections += 1
                rospy.loginfo("Axis fit rejected: %.1f deg from previous",
                              np.degrees(np.arccos(np.clip(abs(dot), 0.0, 1.0))))
                return self.axis_prev

            # Five bad frames in a row: assume the tool really turned, and use this fit.
            # Clear the past positions, or the velocity would include the sudden jump.
            rospy.logwarn("Axis re-locked after %d rejected frames", self.rejections)
            self.pose_history = []

        self.rejections = 0
        if dot < 0.0:
            axis = -axis
        self.axis_prev = axis
        self.update_history(stamp, centre, axis)
        return axis


    def lock_convention(self):
        """
        Startup vote. Consensus axis = principal eigenvector of sum(a a^T), which ignores each fit's sign.
        Its sign is chosen so its largest-magnitude component (pc axes) is positive - arbitrary, but never a tie.
        Buffered fits within the gate are signed to match and enter the history; the rest are discarded as outliers.
        """
        axes = np.array([b[2] for b in self.startup_buffer])
        _, eigvecs = np.linalg.eigh(axes.T @ axes)
        ref = eigvecs[:, -1]
        if ref[np.argmax(np.abs(ref))] < 0.0:
            ref = -ref

        n_kept = 0
        for stamp, centre, axis in self.startup_buffer:
            dot = np.dot(axis, ref)
            if abs(dot) < self.cos_gate:
                continue
            if dot < 0.0:
                axis = -axis
            self.update_history(stamp, centre, axis)
            self.axis_prev = axis
            n_kept += 1

        self.startup_buffer = []

        # If every fit disagreed with the consensus, axis_prev is still None and the vote restarts
        rospy.loginfo("Axis convention: %d of %d startup fits agree", n_kept, self.startup_frames)
        return self.axis_prev


    def update_history(self, stamp, centre, axis):
        """
        Maintain a bounded history of past centres and axes for velocity estimation.
        """
        self.pose_history.append((stamp, centre.copy(), axis.copy()))
        if len(self.pose_history) > self.max_history_len:
            self.pose_history.pop(0)


    def estimate_twist(self):
        """
        Least-squares slope over the history window, valid at the window's mid time (lag = half the window).
        Angular velocity is d x d_dot: the component perpendicular to the axis only.
        """
        t = np.array([h[0] for h in self.pose_history])
        centres = np.array([h[1] for h in self.pose_history])
        axes = np.array([h[2] for h in self.pose_history])

        t = t - t.mean()
        denom = np.sum(t * t)
        if denom <= 0.0:
            rospy.logwarn("Twist estimate skipped: all stamps in the window are identical")
            return np.zeros(3), np.zeros(3)

        v = t @ (centres - centres.mean(axis=0)) / denom
        axis_rate = t @ (axes - axes.mean(axis=0)) / denom

        axis_mid = axes.mean(axis=0)
        axis_mid = axis_mid / np.linalg.norm(axis_mid)
        w = np.cross(axis_mid, axis_rate)

        return v, w


if __name__ == '__main__':
    try:
        rospy.init_node('pose_extractor')
        node = PoseExtractor()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
