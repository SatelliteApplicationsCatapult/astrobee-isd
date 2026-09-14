#!/usr/bin/env python
import rospy
import rosbag


import numpy as np
# import gymnasium as gym
# from imitation.algorithms import bc
# from imitation.data.types import TransitionsMinimal

from geometry_msgs.msg import Pose, Point, Quaternion, WrenchStamped

from astrobee_joy_teleop import pointcloud_utilities as pu


class BcData:
    def __init__(self):

        # Static TF
        self.robot_in_perch_tf = []

        # Tool Info
        self.tool_found = False
        self.tool_name = ""

        # Arrays
        self.tool_in_pc_tf = []
        self.pointcloud_points = []
        self.gamepad_raw_axes = []
        self.gamepad_raw_buttons = []
        self.gamepad_wrench = []
        self.arm_joint_state = []
        self.arm_gripper_state = []


class BcFirstTest:
    def __init__(self):

        # Parameters
        self.bag_path = rospy.get_param('~bag_path', 'input.bag')
        self.robot_name = rospy.get_param('~robot_name', 'honey')
        self.pc_frame = rospy.get_param('~pc_frame', 'honey/perch_cam')
        self.model_states_topic = rospy.get_param('~model_states', "/gazebo/model_states")            # gazebo_msgs/ModelStates
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/honey/hw/depth_perch/points')  # sensor_msgs/PointCloud2
        self.gamepad_raw_topic = rospy.get_param('~gamepad_raw_topic', '/joy')                        # sensor_msgs/Joy
        self.gamepad_wrench_topic = rospy.get_param('~gamepad_topic', '/joy_wrench')                  # geometry_msgs/WrenchStamped
        self.arm_state_topic = rospy.get_param('~arm_state_topic', '/honey/beh/arm/arm_state')        # ff_msgs/ArmStateStamped, contains:
                                                                                                      #   - ff_msgs/ArmJointState
                                                                                                      #   - ff_msgs/ArmGripperState
        self.topics_of_interest = [self.model_states_topic,
                                   self.pointcloud_topic,
                                   self.gamepad_raw_topic,
                                   self.gamepad_wrench_topic,
                                   self.arm_state_topic,
                                ]

        self.tools_list = ["ratchet_wrench", "wrench_10mm"]



        self.load_data_from_file()

        self.preprocess_data_for_bc()




    def load_data_from_file(self):
        """
        Get data from bag file that is required for model training.
        """


        self.bc_data = BcData()



        # Static transform from robot body to perch cam
        # rosrun tf tf_echo honey/perch_cam honey/body
        # - Translation: [0.017, -0.051, -0.133]
        # - Rotation: in Quaternion [-0.000, 0.707, -0.000, 0.707]
        #             in RPY (radian) [0.000, 1.571, 0.000]
        #             in RPY (degree) [0.000, 90.000, 0.000]
        self.bc_data.robot_in_perch_tf = pu.pose_msg_to_array(
             Pose(position=Point(0.017, -0.051, -0.133),
                  orientation=Quaternion(-0.0, 0.707, -0.0, 0.707)))



        bag = rosbag.Bag(self.bag_path, 'r')



        # Print basic data from bag
        print("Bag info:")
        info = bag.get_type_and_topic_info()
        topics = info.topics
        w_name = max((len(t) for t in topics), default=0)
        w_type = max((len(m.msg_type) for m in topics.values()), default=0)
        for topic, meta in topics.items():
            freq = meta.frequency if meta.frequency is not None else float("nan")
            rospy.loginfo(f"Topic: {topic:<{w_name}}, type: {meta.msg_type:<{w_type}}, count: {meta.message_count:>6}, freq: {freq:6.1f}")



        try:
            for topic, msg, t in bag.read_messages(topics=self.topics_of_interest):

                # Groundtruth data
                if topic == self.model_states_topic:

                    model_names = msg.name

                    if not self.bc_data.tool_found:

                        matches = [item for item in model_names if item in self.tools_list]

                        if len(matches) == 0:
                            rospy.logerr("Cannot find a tool")
                            return
                        elif len(matches) > 1:
                            rospy.logwarn("More than one tools found, picking first")

                        self.bc_data.tool_name = matches[0]
                        self.bc_data.tool_found = True
                        rospy.loginfo("Tool found: %s", self.bc_data.tool_name)


                    # Gest and convert frames

                    robot_idx = model_names.index(self.robot_name)
                    tool_idx = model_names.index(self.tool_name)

                    robot_in_world_tf = pu.pose_msg_to_array(msg.pose[robot_idx])
                    tool_in_world_tf = pu.pose_msg_to_array(msg.pose[tool_idx])

                    tool_in_robot_tf = np.linalg.inv(robot_in_world_tf) @ tool_in_world_tf

                    self.bc_data.tool_in_pc_tf = self.robot_in_perch_tf @ tool_in_robot_tf


                # #############
                # Here: This is where pose estimation should later be done, instead of relying on temp groundtruth data
                elif topic == self.pointcloud_topic:
                    # Convert PointCloud2 to an internal point list / array
                    self.bc_data.pointcloud_points = pu.pointcloud_msg_to_array(msg)
                # #############


                # Gamepad raw inputs
                elif topic == self.gamepad_raw_topic:
                    self.bc_data.gamepad_raw_axes = msg.axes
                    self.bc_data.gamepad_raw_buttons = msg.buttons



                # Gamepad wrench inputs
                elif topic == self.gamepad_wrench_topic:
                    self.bc_data.gamepad_wrench = msg.wrench


                # Arm and gripper state
                elif topic == self.arm_state_topic:
                    self.bc_data.arm_joint_state = msg.joint_state
                    self.bc_data.arm_gripper_state = msg.gripper_state



        finally:
            bag.close()

        rospy.loginfo("Loading done.")







    def preprocess_data_for_bc(self):
        """
        Get data ready for the format required for Behavioural Cloning, which uses the Imitation library.
        """

        self.bc_data


        pass








if __name__ == '__main__':
    try:
        rospy.init_node('bc_first_test')
        node = BcFirstTest()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
