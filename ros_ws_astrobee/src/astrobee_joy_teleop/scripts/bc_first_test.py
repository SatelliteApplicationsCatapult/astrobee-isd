#!/usr/bin/env python
import rospy
import rosbag

import numpy as np
from tf.transformations import translation_from_matrix, quaternion_from_matrix, quaternion_slerp

from geometry_msgs.msg import Pose, Point, Quaternion, WrenchStamped

from astrobee_joy_teleop import pointcloud_utilities as pu

# import gymnasium as gym
# from imitation.algorithms import bc
# from imitation.data.types import TransitionsMinimal


class BcData:
    """Uniformly-sampled arrays on one clock."""

    def __init__(self):
        self.tool_name = ""
        self.t = None                  # (N,)      s
        self.tool_pos = None           # (N, 3)    m, in perch_cam
        self.tool_quat = None          # (N, 4)    x y z w, in perch_cam
        self.tool_lin_vel = None       # (N, 3)    m/s, in perch_cam
        self.tool_ang_vel = None       # (N, 3)    rad/s, in perch_cam
        self.wrench_cmd = None         # (N, 6)    Fx Fy Fz Tx Ty Tz
        self.joy_axes = None           # (N, n_axes)
        self.joy_buttons = None        # (N, n_buttons)
        self.arm_joint_state = None    # (N,)
        self.arm_gripper_state = None  # (N,)


class BcFirstTest:
    def __init__(self):

        # Parameters
        self.bag_path = rospy.get_param('~bag_path', 'input.bag')
        self.rate_hz = rospy.get_param('~rate_hz', 30.0)
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

        # Static transform from robot body to perch cam
        # rosrun tf tf_echo honey/perch_cam honey/body
        # - Translation: [0.017, -0.051, -0.133]
        # - Rotation: in Quaternion [-0.000, 0.707, -0.000, 0.707]
        #             in RPY (radian) [0.000, 1.571, 0.000]
        #             in RPY (degree) [0.000, 90.000, 0.000]
        self.robot_in_perch_tf = pu.pose_msg_to_array(
            Pose(position=Point(0.017, -0.051, -0.133),
                 orientation=Quaternion(0.0, np.sqrt(2.0) / 2.0, 0.0, np.sqrt(2.0) / 2.0)))


        self.load_data_from_file()

        self.preprocess_data_for_bc()


    def load_data_from_file(self):
        """
        Read the bag and return a BcData of uniformly-sampled arrays.
        """

        tool_name = ""
        pc_t, pc_points = [], []
        tool_t, tool_pose_in_pc, tool_twist_in_pc = [], [], []
        joy_t, joy_axes, joy_buttons = [], [], []
        wrench_t, wrench_cmd = [], []
        arm_t, arm_joint, arm_grip = [], [], []

        bag = rosbag.Bag(self.bag_path, 'r')

        # Print basic data from bag
        print("Bag info:")
        info = bag.get_type_and_topic_info()
        topics = info.topics
        w_name = max((len(t) for t in topics), default=0)
        w_type = max((len(m.msg_type) for m in topics.values()), default=0)
        for topic, meta in topics.items():
            freq = meta.frequency if meta.frequency is not None else float("nan")
            rospy.loginfo(f"Topic: {topic:<{w_name}}, type: {meta.msg_type:<{w_type}}, "
                          f"count: {meta.message_count:>6}, freq: {freq:6.1f}")

        try:
            for topic, msg, t in bag.read_messages(topics=self.topics_of_interest):

                # Shared clock
                ts = t.to_sec()




                # Pose estimation
                # #############
                # Here: This is where pose estimation should later be done, instead of relying on temp groundtruth data
                if topic == self.pointcloud_topic:

                    pc_t.append(ts)
                    pc_points.append(pu.pointcloud_msg_to_array(msg))

                # ... Eventually replace 'tool_pose_in_pc' and 'tool_twist_in_pc' below
                # #############


                # TEMPORARY!!!
                # Groundtruth data
                elif topic == self.model_states_topic:

                    model_names = msg.name

                    if not tool_name:
                        matches = [item for item in model_names if item in self.tools_list]
                        if not matches:
                            # Tool is spawned after the robot; frames before the spawn are dropped
                            continue
                        if len(matches) > 1:
                            rospy.logwarn("More than one tool found, picking first")
                        tool_name = matches[0]
                        rospy.loginfo("Tool found: %s", tool_name)

                    robot_idx = model_names.index(self.robot_name)
                    tool_idx = model_names.index(tool_name)

                    # Relative pose

                    robot_in_world_tf = pu.pose_msg_to_array(msg.pose[robot_idx])
                    tool_in_world_tf = pu.pose_msg_to_array(msg.pose[tool_idx])

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


                    tool_t.append(ts)
                    tool_pose_in_pc.append(self.robot_in_perch_tf @ tool_in_robot_tf)
                    tool_twist_in_pc.append(np.concatenate([R_wp @ v_rel_world, R_wp @ w_rel_world]))






                # Gamepad raw inputs
                elif topic == self.gamepad_raw_topic:
                    joy_t.append(ts)
                    joy_axes.append(msg.axes)
                    joy_buttons.append(msg.buttons)

                # Gamepad wrench inputs
                elif topic == self.gamepad_wrench_topic:
                    wrench_t.append(ts)
                    wrench_cmd.append([msg.wrench.force.x,
                                       msg.wrench.force.y,
                                       msg.wrench.force.z,
                                       msg.wrench.torque.x,
                                       msg.wrench.torque.y,
                                       msg.wrench.torque.z])

                # Arm and gripper state
                elif topic == self.arm_state_topic:
                    arm_t.append(ts)
                    arm_joint.append(msg.joint_state.state)
                    arm_grip.append(msg.gripper_state.state)

        finally:
            bag.close()

        if not tool_name:
            raise RuntimeError("No tool from %s appeared in %s" % (self.tools_list, self.model_states_topic))

        for name, stamps in (('pc', pc_t), ('tool', tool_t), ('joy', joy_t), ('wrench', wrench_t), ('arm', arm_t)):
                    if not stamps:
                        raise RuntimeError("No messages on the %s stream in %s" % (name, self.bag_path))


        # Use pointcloud data to get pose estimates
        # TODO:
        # tool_t, tool_pose_in_pc, tool_twist_in_pc = self.run_pose_estimation(pc_t, pc_points)

        raw = dict(tool_name=tool_name,
                   tool=(tool_t, tool_pose_in_pc, tool_twist_in_pc),
                   joy=(joy_t, joy_axes, joy_buttons),
                   wrench=(wrench_t, wrench_cmd),
                   arm=(arm_t, arm_joint, arm_grip))

        self.bc_data = self.resample(raw, self.rate_hz)

        rospy.loginfo("Loading done: %d frames at %.1f Hz", len(self.bc_data.t), self.rate_hz)

        return self.bc_data


    def run_pose_estimation(self, pc_t, pc_points):

        # TODO:
        tool_t = pc_t
        tool_pose_in_pc = []
        tool_twist_in_pc = []

        return tool_t, tool_pose_in_pc, tool_twist_in_pc


    # Resampling helpers
    @staticmethod
    def _increasing(t):
        """Drop repeated/backward timestamps. Gazebo emits several messages per sim tick, so ~30% of model_states share a stamp."""
        t = np.asarray(t, dtype=float)
        keep = np.concatenate(([True], np.diff(t) > 0))
        return t[keep], keep


    @staticmethod
    def _lerp(grid, t, v):
        """Linear interpolation, column by column. v is (N, k)."""
        return np.column_stack([np.interp(grid, t, v[:, i])
                                for i in range(v.shape[1])])


    @staticmethod
    def _zoh(grid, t, v):
        """Zero-order hold. For anything discrete or non-numeric."""
        idx = np.clip(np.searchsorted(t, grid, side='right') - 1, 0, len(t) - 1)
        return v[idx]


    @staticmethod
    def _quat_interp(grid, t, q):
        """Slerp each grid point between its two bracketing samples."""
        out = np.empty((len(grid), 4))
        for k, tk in enumerate(grid):
            i = np.searchsorted(t, tk, side='right') - 1
            i = min(max(i, 0), len(t) - 2)
            f = (tk - t[i]) / (t[i + 1] - t[i])
            out[k] = quaternion_slerp(q[i], q[i + 1], f)
        return out


    def resample(self, raw, rate_hz):
        """Put every stream on one uniform clock. Continuous quantities are linearly interpolated, discrete ones are held."""

        # Extract translation and quaternion from pose
        tool_t, keep = self._increasing(raw['tool'][0])
        P = np.asarray(raw['tool'][1], dtype=float)[keep]
        tool_pos = np.array([translation_from_matrix(p) for p in P])
        tool_quat = np.array([quaternion_from_matrix(p) for p in P])  # x y z w

        # Extract linear and angular velocity from twist
        T = np.asarray(raw['tool'][2], dtype=float)[keep]
        tool_lin_vel = T[:, 0:3]
        tool_ang_vel = T[:, 3:6]

        joy_t, keep = self._increasing(raw['joy'][0])
        joy_axes = np.asarray(raw['joy'][1], dtype=float)[keep]
        joy_buttons = np.asarray(raw['joy'][2], dtype=np.int8)[keep]

        wrench_t, keep = self._increasing(raw['wrench'][0])
        wrench_cmd = np.asarray(raw['wrench'][1], dtype=float)[keep]

        arm_t, keep = self._increasing(raw['arm'][0])
        arm_joint = np.asarray(raw['arm'][1], dtype=np.int8)[keep]
        arm_grip =  np.asarray(raw['arm'][2], dtype=np.int8)[keep]

        # Common window only. No extrapolation past any stream's ends.
        t0 = max(tool_t[0], wrench_t[0], joy_t[0], arm_t[0])
        t1 = min(tool_t[-1], wrench_t[-1], joy_t[-1], arm_t[-1])
        if not t1 > t0:
            raise RuntimeError("Topic time windows do not overlap")


        d = BcData()
        d.tool_name = raw['tool_name']

        d.t = np.arange(t0, t1, 1.0 / rate_hz)

        d.tool_pos = self._lerp(d.t, tool_t, tool_pos)
        d.tool_quat = self._quat_interp(d.t, tool_t, tool_quat)
        d.tool_lin_vel = self._lerp(d.t, tool_t, tool_lin_vel)
        d.tool_ang_vel = self._lerp(d.t, tool_t, tool_ang_vel)

        d.wrench_cmd = self._lerp(d.t, wrench_t, wrench_cmd)

        # Held, not interpolated: autorepeat republishes the joystick values.
        d.joy_axes = self._zoh(d.t, joy_t, joy_axes)
        d.joy_buttons = self._zoh(d.t, joy_t, joy_buttons)

        # Held: these are enums.
        d.arm_joint_state = self._zoh(d.t, arm_t, arm_joint)
        d.arm_gripper_state = self._zoh(d.t, arm_t, arm_grip)

        return d


    def preprocess_data_for_bc(self):
        """
        Get data ready for the format required for Behavioural Cloning, which uses the Imitation library.
        """

        pass


if __name__ == '__main__':
    try:
        rospy.init_node('bc_first_test')
        node = BcFirstTest()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
