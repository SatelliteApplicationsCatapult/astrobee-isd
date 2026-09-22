#!/usr/bin/env python
import rospy
import rosbag
import rosparam

#import ros_numpy as rnp
#from tf2_geometry_msgs import do_transform_pose

from std_msgs.msg import Header, Float32
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TwistStamped, Vector3Stamped
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


        # Publishers
        self.groundtruth_pose_pub = rospy.Publisher('~groundtruth_pose', PoseStamped, queue_size=10)
        self.groundtruth_twist_pub = rospy.Publisher('~groundtruth_twist', TwistStamped, queue_size=10)

        self.original_pub = rospy.Publisher('~cloud_original', PointCloud2, queue_size=10)
        self.filtered_pub = rospy.Publisher('~cloud_filtered', PointCloud2, queue_size=10)
        self.linefit_pub = rospy.Publisher('~cloud_linefit', PointCloud2, queue_size=10)

        self.shape_pub = rospy.Publisher("~cylinder_shape", Marker, queue_size=10)

        # Publishing a cylinder pose/twist is pointless since the line fit does not have twist or direction components
        # Publish the axis value itself, and the error compared to groundtruh
        #self.estimated_pose_pub = rospy.Publisher('~cylinder_pose', PoseStamped, queue_size=10)
        #self.estimated_twist_pub = rospy.Publisher('~cylinder_twist', TwistStamped, queue_size=10)
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


        # Internal state for velocity estimation (running average over past frames)
        self.pose_history = []  # list of PoseStamped, TwistStamped

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
        groundtruth_twist_msg = TwistStamped()
        noisy_cloud_msg = PointCloud2()
        filtered_cloud_msg = PointCloud2()
        linefit_cloud_msg = PointCloud2()
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

                    groundtruth_pose_msg.header = header
                    groundtruth_pose_msg.pose = pu.array_to_pose_msg(tool_in_pc_tf)

                    # Offset for visualisation
                    groundtruth_pose_msg.pose.position.y += (results_offset_y * 2)

                    groundtruth_twist_msg.header = header
                    groundtruth_twist_msg.twist =  pu.transform_twist_msg(msg.twist[tool_idx], tool_in_pc_tf[:3, :3])




                    # Z is tool's long axis (reminder to keep this convention on all tools!)

                    # Calculate: The groundtruth direction of the tool’s long Z axis in the pointcloud frame

                    R_gt = tool_in_pc_tf[:3, :3]  # Rotation matrix
                    axis_gt = R_gt[:, 2]  # tool local Z axis
                    axis_gt = axis_gt / np.linalg.norm(axis_gt)  # Normalise vector


                    # if axis_gt[1] < 0:
                    #     axis_gt = -axis_gt



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


                    # Axis direction (from line fit)
                    axis_fit = line_slope / np.linalg.norm(line_slope)


                    # if axis_fit[1] < 0:
                    #     axis_fit = -axis_fit




                    # Calculate orientation quaternion (q xyzw)
                    z_axis = np.array([0.0, 0.0, 1.0])
                    axis_dot = np.dot(z_axis, axis_fit)
                    axis_cross = np.cross(z_axis, axis_fit)
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






                    # Centre point on the axis
                    # inlier_pts = filtered_points[line_inliers]  # (N, 3)
                    # cyl_centre = inlier_pts.mean(axis=0)

                    # Use inliers only to estimate length along the axis
                    inlier_pts = filtered_points[line_inliers]  # (N, 3)

                    # Project inliers onto axis, using line_intercept as origin
                    p = (inlier_pts - line_intercept) @ axis_fit  # dot product of vector v with unit vector axis
                    # p_min, p_max = np.percentile(p, [5, 95])
                    p_min = p.min()
                    p_max = p.max()

                    # Get cylinder length, start, end, and centre
                    cyl_length = p_max - p_min
                    axis_start = line_intercept + p_min * axis_fit
                    axis_end   = line_intercept + p_max * axis_fit
                    cyl_centre = 0.5 * (axis_start + axis_end)




                    marker = Marker()
                    marker.header.frame_id = self.pc_frame
                    marker.header.stamp = t
                    marker.ns = "fitted_shapes"
                    marker.id = 0
                    marker.type = 3    # CYLINDER
                    # marker.type = 0    # ARROW
                    marker.action = 0  # ADD

                    marker.pose.position.x = cyl_centre[0]
                    marker.pose.position.y = cyl_centre[1] + (results_offset_y * 2)
                    marker.pose.position.z = cyl_centre[2]
                    marker.pose.orientation.x = cyl_q[0]
                    marker.pose.orientation.y = cyl_q[1]
                    marker.pose.orientation.z = cyl_q[2]
                    marker.pose.orientation.w = cyl_q[3]

                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = cyl_length

                    # marker.pose.position.x = cyl_centre[0]
                    # marker.pose.position.y = cyl_centre[1] + results_offset_y
                    # marker.pose.position.z = cyl_centre[2]
                    # marker.pose.orientation.x = cyl_q[0]
                    # marker.pose.orientation.y = cyl_q[1]
                    # marker.pose.orientation.z = cyl_q[2]
                    # marker.pose.orientation.w = cyl_q[3]

                    # marker.scale.x = 0.3
                    # marker.scale.y = 0.02
                    # marker.scale.z = 0.02

                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.5








                    # ignore sign flip: axis and -axis are equivalent for a line
                    dot_val = np.clip(np.abs(np.dot(axis_fit, axis_gt)), -1.0, 1.0)
                    axis_angle_err_deg = np.degrees(np.arccos(dot_val))


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
                    axis_fit_msg = pu.array_to_vector3_msg(header, axis_fit)
                    axis_angle_err_deg_msg = pu.float_to_float32_msg(axis_angle_err_deg)





                    # Extract pose for current frame (center of cylinder and orientation)
                    # estimated_pose_msg = pu.compute_pose_msg("", msg.header)
                    estimated_pose_msg.header = header
                    estimated_pose_msg.pose = marker.pose

                    # Estimate twist (linear/angular velocity) from running average of past poses
                    # estimated_twist_msg = pu.compute_twist_msg(estimated_pose_msg)
                    # estimated_twist_msg.header = header

                    # # Publish PoseStamped and TwistStamped
                    # self.pose_pub.publish(estimated_pose_msg)
                    # self.twist_pub.publish(estimated_twist_msg)

                    # # Update history for future twist estimation
                    # self.update_history(estimated_pose_msg)

                    # Turn noisy data into pointcloud message
                    noisy_cloud_msg = pu.array_to_pointcloud_msg(header, noisy_points)

                    # In-place shift to display better in RViz (fine as we don't need them anymore)
                    filtered_points[:, 1] += results_offset_y
                    # Turn filtered data into pointcloud message
                    filtered_cloud_msg = pu.array_to_pointcloud_msg(header, filtered_points)

                    # Do the same with inlier points
                    inlier_pts[:, 1] += (results_offset_y * 2)
                    linefit_cloud_msg = pu.array_to_pointcloud_msg(header, inlier_pts)


                    self.viz_frames.append({
                        'groundtruth_pose': deepcopy(groundtruth_pose_msg),
                        'groundtruth_twist': deepcopy(groundtruth_twist_msg),
                        'original_cloud': deepcopy(noisy_cloud_msg),
                        'filtered_cloud': deepcopy(filtered_cloud_msg),
                        'linefit_cloud': deepcopy(linefit_cloud_msg),
                        'cylinder': deepcopy(marker),
                        # 'estimated_pose': deepcopy(estimated_pose_msg),
                        # 'estimated_twist': deepcopy(estimated_twist_msg),
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
            self.groundtruth_twist_pub.publish(frame['groundtruth_twist'])
            self.original_pub.publish(frame['original_cloud'])
            self.filtered_pub.publish(frame['filtered_cloud'])
            self.linefit_pub.publish(frame['linefit_cloud'])
            # self.estimated_pose_pub.publish(frame['estimated_pose'])
            # self.estimated_twist_pub.publish(frame['estimated_twist'])
            self.groundtruth_axis_pub.publish(frame['groundtruth_axis'])
            self.fit_axis_pub.publish(frame['fit_axis'])
            self.groundtruth_error_deg_pub.publish(frame['groundtruth_error_deg'])
            self.shape_pub.publish(frame['cylinder'])

            i += 1
            if i >= len(self.viz_frames):
                if self.loop_playback:
                    i = 0
                else:
                    break

            rate.sleep()

        rospy.loginfo("Playback finished")






    def update_history(self, pose_msg):
        """
        Maintain a bounded history of past poses for velocity estimation.
        """
        self.pose_history.append((pose_msg.header.stamp, pose_msg))
        if len(self.pose_history) > self.max_history_len:
            self.pose_history.pop(0)


if __name__ == '__main__':
    try:
        rospy.init_node('pose_extractor')
        node = PoseExtractor()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
