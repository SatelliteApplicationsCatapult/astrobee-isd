#!/usr/bin/env python
import rospy
import rosbag

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped

from astrobee_joy_teleop import pointcloud_utilities as pu


class PoseExtractor(object):
    def __init__(self):

        # Parameters
        self.bag_path = rospy.get_param('~bag_path', 'input.bag')
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/pointcloud')
        self.box_min = rospy.get_param('~box_min', [-0.25, -0.25, 0.0])  # Box region limits [x_min, y_min, z_min]
        self.box_max = rospy.get_param('~box_max', [ 0.25,  0.25, 0.5])     # Box region limits [x_max, y_max, z_max]
        self.max_history_len = rospy.get_param('~max_history_len', 10)
        self.playback_rate = rospy.get_param('~playback_rate', 1.0)
        self.loop_playback = rospy.get_param('~loop_playback', True)


        # Publishers

        self.original_pub = rospy.Publisher('~cloud_original', PointCloud2, queue_size=10)
        self.filtered_pub = rospy.Publisher('~cloud_filtered', PointCloud2, queue_size=10)

        self.pose_pub = rospy.Publisher('~cylinder_pose', PoseStamped, queue_size=10)
        self.twist_pub = rospy.Publisher('~cylinder_twist', TwistStamped, queue_size=10)


        # Internal state for velocity estimation (running average over past frames)
        self.pose_history = []  # list of PoseStamped, TwistStamped



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
        bag = rosbag.Bag(self.bag_path, 'r')
        try:
            for topic, msg, t in bag.read_messages(topics=[self.pointcloud_topic]):
                if rospy.is_shutdown():
                    break

                # Convert PointCloud2 to an internal point list / array
                points = pu.pointcloud_to_array(msg)

                # Add noise here
                # gaussian_noise = np.random.normal(mean, std_deviation, shape)

                # Trim points outside the defined box region
                trimmed_points = pu.trim_to_box(points, self.box_min, self.box_max)

                # Remove big outliers
                filtered_points = pu.remove_outliers(trimmed_points)

                # 3D linear regression fit to estimate cylinder axis and length (fit line, then cap ends)
                cylinder_model = pu.fit_cylinder(filtered_points)

                # Extract pose for current frame (center of cylinder and orientation)
                pose_msg = pu.compute_pose_msg(cylinder_model, msg.header)

                # Estimate twist (linear/angular velocity) from running average of past poses
                twist_msg = pu.compute_twist_msg(pose_msg)

                # # Publish PoseStamped and TwistStamped
                # self.pose_pub.publish(pose_msg)
                # self.twist_pub.publish(twist_msg)

                # # Update history for future twist estimation
                # self.update_history(pose_msg)


                # In-place shift to display better in RViz (fine as we don't need them anymore)
                filtered_points[:, 1] += 1.0
                # Turn into pointcloud message
                filtered_cloud_msg = pu.array_to_pointcloud(msg.header, filtered_points)



                self.viz_frames.append({
                    'original_cloud': msg,
                    'filtered_cloud': filtered_cloud_msg,
                    'pose': pose_msg,
                    'twist': twist_msg
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

            self.original_pub.publish(frame['original_cloud'])
            self.filtered_pub.publish(frame['filtered_cloud'])
            self.pose_pub.publish(frame['pose'])
            self.twist_pub.publish(frame['twist'])

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
