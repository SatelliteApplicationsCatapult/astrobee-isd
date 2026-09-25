#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped
from astrobee_joy_teleop.msg import JoyArm

import numpy as np
#import onnx
import onnxruntime as ort
#from onnx import numpy_helper

####
# TEMPORARY!!!
import rosparam
from gazebo_msgs.msg import ModelStates
from astrobee_joy_teleop import pointcloud_utilities as pu
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import (translation_from_matrix, rotation_from_matrix)
####


class AstrobeeInferenceArmWrench:
    def __init__(self):

        # Namespace
        self.ns = rospy.get_param("~ns", "honey")

        # Max limits based on Astrobee's physical blower limits
        self.max_force = rospy.get_param("/wrench_command/max_force", 0.8)
        self.max_torque = rospy.get_param("/wrench_command/max_torque", 0.05)

        # Publishers to be used by another Astrobee client that processes these commands
        # WrenchStamped for body
        self.wrench_cmd_pub = rospy.Publisher('/joy_wrench', WrenchStamped, queue_size=10)
        # Custom for arm/gripper
        self.arm_pub = rospy.Publisher('/joy_arm', JoyArm, queue_size=10)

        rospy.loginfo("Astrobee 6-DoF inference controller initialised. " \
                      "To control Astrobee's body/arm, make sure a client is running which " \
                      "subscribes to the WrenchStamped and JoyArm messages sent from here.")




        self.tool_pose_in_pc = None
        self.tool_twist_in_pc = None



        ####
        # TEMPORARY!!! - Gazebo model states

        self.common_params_yaml = "/src/astrobee-isd/ros_ws_astrobee/src/astrobee_ros_demo/config/common_params.yaml"

        # Load params
        params = rosparam.load_file(self.common_params_yaml)[0][0]
        self.max_force  = params['wrench_command']['max_force']
        self.max_torque = params['wrench_command']['max_torque']

        # Static transform from robot body to perch cam
        tf_params = params['robot_sim']['robot_in_perch_tf']
        self.robot_in_perch_tf = pu.pose_msg_to_array(
                    Pose(position=Point(*tf_params['position']),
                         orientation=Quaternion(*tf_params['orientation'])))

        self.robot_name = "honey"
        self.tool_name = "ratchet_wrench"
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        ####


        self.load_model()


        rate = 30.0
        rospy.Timer(rospy.Duration(1/rate), self.timer_callback)






    ####
    # TEMPORARY!!!
    def model_states_callback(self, msg):


        model_names = msg.name
        #print("model_names:", model_names)

        if self.robot_name in model_names:
            robot_idx = model_names.index(self.robot_name)
            # print("Robot index:", robot_idx)
        else:
            return


        if self.tool_name in model_names:
            tool_idx = model_names.index(self.tool_name)
            # print("Tool index:", tool_idx)
        else:
            return



        # Relative pose

        #print("msg.pose:", msg.pose[robot_idx])

        robot_in_world_tf = pu.pose_msg_to_array(msg.pose[robot_idx])
        tool_in_world_tf = pu.pose_msg_to_array(msg.pose[tool_idx])

        #print("robot_in_world_tf: ", robot_in_world_tf)
        #print("tool_in_world_tf: ", tool_in_world_tf)


        world_in_robot_tf = np.linalg.inv(robot_in_world_tf)
        tool_in_robot_tf = world_in_robot_tf @ tool_in_world_tf

        # Relative twist

        robot_in_world_twist = pu.twist_msg_to_array(msg.twist[robot_idx])
        tool_in_world_twist = pu.twist_msg_to_array(msg.twist[tool_idx])

        v_body, w_body = robot_in_world_twist[0:3], robot_in_world_twist[3:6]
        v_tool, w_tool = tool_in_world_twist[0:3], tool_in_world_twist[3:6]

        # Lever arm runs from the BODY origin, not perch_cam: the camera offset cancels exactly
        r = tool_in_world_tf[:3, 3] - robot_in_world_tf[:3, 3]

        w_rel_world = w_tool - w_body
        v_rel_world = v_tool - v_body - np.cross(w_body, r)

        # Rotation only. A velocity has no position, so translation does not apply.
        R_wp = (self.robot_in_perch_tf @ world_in_robot_tf)[:3, :3]



        self.tool_pose_in_pc  = self.robot_in_perch_tf @ tool_in_robot_tf
        self.tool_twist_in_pc = np.concatenate([R_wp @ v_rel_world, R_wp @ w_rel_world])


        #print("pose: ", self.tool_pose_in_pc)
        #print("twist: ", self.tool_twist_in_pc)


    ####







    def load_model(self):

        # Load ONNX Runtime inference session
        # Strictly, this is the only thing needed for inference
        onnx_path = "/src/astrobee-isd/data/training/il_first_data/bc_policy/policy.onnx"
        self.ort_sess = ort.InferenceSession(onnx_path)

        # Load model, if you want to extract saved metadata
        # m = onnx.load(onnx_path)
        # init = {i.name: numpy_helper.to_array(i) for i in m.graph.initializer}
        # mean, sigma = init["mean"], init["sigma"]





    def timer_callback(self, event):



        if self.tool_pose_in_pc is None or self.tool_twist_in_pc is None:
            return


        # Extract translation and rotation from pose
        P = np.asarray(self.tool_pose_in_pc, dtype=float)
        #pos = translation_from_matrix(P)
        pos = P[:3, 3]
        rot = P[:3, :3].flatten()

        # Extract linear and angular velocity from twist
        T = np.asarray(self.tool_twist_in_pc, dtype=float)
        lin_vel = T[0:3]
        ang_vel = T[3:6]

        # print("P:", P)
        # print("pos:", pos)
        # print("rot:", rot)
        # print("T:", T)
        # print("lin_vel:", lin_vel)
        # print("ang_vel:", ang_vel)


        #obs = [pos, rot, lin_vel, ang_vel]

        obs = np.concatenate([pos, rot, lin_vel, ang_vel]).astype(np.float32).reshape(1, -1)  # (1, 18)

        print("obs:", obs)
        print("obs.shape:", obs.shape)



        #actions = self.ort_sess.run(None, {'obs': obs})[0]
        actions = self.ort_sess.run(None, {'obs': obs})[0][0]


        print("actions: ", actions)
        print("actions.shape: ", actions.shape)


        now = rospy.Time.now()



        # # Wrench output for Astrobee body control

        cmd = WrenchStamped()
        cmd.header.stamp = now
        # Apply forces relative to robot body frame
        if self.ns:
            cmd.header.frame_id = "{}/body".format(self.ns)
        else:
            cmd.header.frame_id = "body"

        cmd.wrench.force.x = actions[0]
        cmd.wrench.force.y = actions[1]
        cmd.wrench.force.z = actions[2]
        cmd.wrench.torque.x = actions[3]
        cmd.wrench.torque.y = actions[4]
        cmd.wrench.torque.z = actions[5]

        self.wrench_cmd_pub.publish(cmd)



        # Perch arm state

        if actions[6] > 0.5:

            arm = JoyArm()
            arm.header.stamp  = now
            arm.gripper_close = 1

            self.arm_pub.publish(arm)



if __name__ == '__main__':
    try:
        rospy.init_node('astrobee_inference_arm_wrench')
        node = AstrobeeInferenceArmWrench()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
