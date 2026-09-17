#!/usr/bin/env python
import os
#import rospy
import rosbag
import rosparam

import argparse
import logging
import coloredlogs

import numpy as np
from tf.transformations import translation_from_matrix, quaternion_from_matrix, quaternion_slerp

from geometry_msgs.msg import Pose, Point, Quaternion, WrenchStamped

from astrobee_joy_teleop import pointcloud_utilities as pu

import gymnasium as gym
from imitation.algorithms import bc
from imitation.data.types import Transitions, TransitionsMinimal


# Logger with coloured outputs
#logging.basicConfig(level=logging.DEBUG, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)
coloredlogs.install(level=logging.DEBUG, fmt="%(asctime)s %(levelname)s %(message)s")


def get_config(argv=None):
    p = argparse.ArgumentParser(
        description="Prepare rosbag data for Behavioural Cloning",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # Required/positional
    p.add_argument('bag_folder',
                   help="Folder containing one or more rosbags (organised as sub-folders)")
    p.add_argument('common_params_yaml',
                   help="YAML file containing common params")
    # Optional
    p.add_argument('--rate-hz', type=float, default=30.0,
                   help="Common rate that all inputs will be interpolated against")
    p.add_argument('--robot-name', default='honey',
                   help="Robot name/namespace")
    p.add_argument('--pc-frame', default='honey/perch_cam',
                   help="Pointcloud data reference frame")
    p.add_argument('--model-states', default='/gazebo/model_states',
                   help="gazebo model states for groundtruth data - gazebo_msgs/ModelStates")
    p.add_argument('--pointcloud-topic', default='/honey/hw/depth_perch/points',
                   help="Pointcloud data topic - sensor_msgs/PointCloud2")
    p.add_argument('--gamepad-raw-topic', default='/joy',
                   help="Joystick raw data topic - sensor_msgs/Joy")
    p.add_argument('--gamepad-wrench-topic', default='/joy_wrench',
                   help="Joystick data converted to wrench values - geometry_msgs/WrenchStamped")
    p.add_argument('--arm-state-topic', default='/honey/beh/arm/arm_state',
                   help="Topic containing both arm and gripper states - ff_msgs/ArmStateStamped, contains ff_msgs/ArmJointState and ff_msgs/ArmGripperState")
    return p.parse_args(argv)


class BcData:
    """Uniformly-sampled arrays on one clock. One instance per run/bag."""

    def __init__(self):
        self.bag_path = ""
        self.tool_name = ""
        self.data_rate = 0
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
    def __init__(self, args):

        # # Parameters
        # self.bag_folder = rospy.get_param('~bag_folder', 'rosbags')
        # self.rate_hz = rospy.get_param('~rate_hz', 30.0)
        # self.robot_name = rospy.get_param('~robot_name', 'honey')
        # self.pc_frame = rospy.get_param('~pc_frame', 'honey/perch_cam')
        # self.model_states_topic = rospy.get_param('~model_states', "/gazebo/model_states")            # gazebo_msgs/ModelStates
        # self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/honey/hw/depth_perch/points')  # sensor_msgs/PointCloud2
        # self.gamepad_raw_topic = rospy.get_param('~gamepad_raw_topic', '/joy')                        # sensor_msgs/Joy
        # self.gamepad_wrench_topic = rospy.get_param('~gamepad_topic', '/joy_wrench')                  # geometry_msgs/WrenchStamped
        # self.arm_state_topic = rospy.get_param('~arm_state_topic', '/honey/beh/arm/arm_state')        # ff_msgs/ArmStateStamped, contains:
        #                                                                                               #   - ff_msgs/ArmJointState
        #                                                                                               #   - ff_msgs/ArmGripperState
        # Remove ROS node and use argparse instead
        # Parameters
        self.bag_folder = args.bag_folder
        self.common_params_yaml = args.common_params_yaml
        self.rate_hz = args.rate_hz
        self.robot_name = args.robot_name
        self.pc_frame = args.pc_frame
        self.model_states_topic = args.model_states
        self.pointcloud_topic = args.pointcloud_topic
        self.gamepad_raw_topic = args.gamepad_raw_topic
        self.gamepad_wrench_topic = args.gamepad_wrench_topic
        self.arm_state_topic = args.arm_state_topic

        self.topics_of_interest = [self.model_states_topic,
                                   self.pointcloud_topic,
                                   self.gamepad_raw_topic,
                                   self.gamepad_wrench_topic,
                                   self.arm_state_topic,
                                   ]

        self.tools_list = ["ratchet_wrench", "wrench_10mm"]

        # Load params
        params = rosparam.load_file(args.common_params_yaml)[0][0]
        self.max_force  = params['wrench_command']['max_force']
        self.max_torque = params['wrench_command']['max_torque']

        # Set action scaling param
        self.act_scale = np.array([self.max_force] * 3 + [self.max_torque] * 3 + [1.0], dtype=np.float32)

        # Static transform from robot body to perch cam
        # rosrun tf tf_echo honey/perch_cam honey/body
        # - Translation: [0.017, -0.051, -0.133]
        # - Rotation: in Quaternion [-0.000, 0.707, -0.000, 0.707]
        #             in RPY (radian) [0.000, 1.571, 0.000]
        #             in RPY (degree) [0.000, 90.000, 0.000]
        self.robot_in_perch_tf = pu.pose_msg_to_array(
            Pose(position=Point(0.017, -0.051, -0.133),
                 orientation=Quaternion(0.0, np.sqrt(2.0) / 2.0, 0.0, np.sqrt(2.0) / 2.0)))

        # Array of BcData objects, one per bag file
        self.bc_data = []


    def run(self):
        """Load and prep data."""

        # Load and check data
        self.load_data_from_files()
        self.check_runs_consistent()
        self.debug_print_bc_data()

        # BC prep and train test
        self.preprocess_data_for_bc()
        self.bc_train_test()
        # Save to file
        self.save_trained_policy()


    def load_data_from_files(self):
        """Load every SUCCESS run into its own BcData."""

        for bag_path in self.find_runs(self.bag_folder):
            logger.info("Loading %s", os.path.basename(bag_path))
            self.bc_data.append(self.load_run(bag_path))

        logger.info("Loaded %d run(s), %d frames total", len(self.bc_data), sum(len(d.t) for d in self.bc_data))


    def find_runs(self, bag_folder):
        """Every immediate subfolder holding a bag, filtered to the SUCCESS-marked ones."""

        outcome_markers = ('SUCCESS', 'FAILURE')

        if not os.path.isdir(bag_folder):
            raise RuntimeError("Not a folder: %s" % bag_folder)

        bag_paths = []
        counts = {'SUCCESS': 0, 'FAILURE': 0, 'unmarked': 0}

        for name in sorted(os.listdir(bag_folder)):
            path = os.path.join(bag_folder, name)

            if not os.path.isdir(path):
                continue

            bags = sorted(f for f in os.listdir(path) if f.endswith('.bag'))
            if not bags:
                continue
            if len(bags) > 1:
                raise RuntimeError("Expected one bag in %s, found %d: %s" % (name, len(bags), ", ".join(bags)))

            markers = [m for m in outcome_markers if os.path.isfile(os.path.join(path, m))]
            if len(markers) > 1:
                raise RuntimeError("Run %s is marked both SUCCESS and FAILURE" % name)
            if not markers:
                counts['unmarked'] += 1
                logger.warning("Run %s has no outcome marker, skipping", name)
                continue

            outcome = markers[0]
            counts[outcome] += 1
            if outcome == 'SUCCESS':
                bag_paths.append(os.path.join(path, bags[0]))

        logger.info("Found %d SUCCESS, %d FAILURE, %d unmarked run(s) in %s", counts['SUCCESS'], counts['FAILURE'], counts['unmarked'], bag_folder)

        if not bag_paths:
            raise RuntimeError("No SUCCESS runs found in %s" % bag_folder)

        return bag_paths


    def load_run(self, bag_path):
        """
        Read bag and return a BcData of uniformly-sampled arrays.
        """

        tool_name = ""
        pc_t, pc_points = [], []
        tool_t, tool_pose_in_pc, tool_twist_in_pc = [], [], []
        joy_t, joy_axes, joy_buttons = [], [], []
        wrench_t, wrench_cmd = [], []
        arm_t, arm_joint, arm_grip = [], [], []

        bag = rosbag.Bag(bag_path, 'r')

        # Print basic data from bag
        logger.info("Bag info:")
        info = bag.get_type_and_topic_info()
        topics = info.topics
        w_name = max((len(t) for t in topics), default=0)
        w_type = max((len(m.msg_type) for m in topics.values()), default=0)
        for topic, meta in topics.items():
            freq = meta.frequency if meta.frequency is not None else float("nan")
            logger.info(f"  Topic: {topic:<{w_name}}, type: {meta.msg_type:<{w_type}}, "
                        f"count: {meta.message_count:>6}, rate: {freq:6.1f}")

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
                            logger.warning("More than one tool found in %s, picking first", os.path.basename(bag_path))
                        tool_name = matches[0]
                        logger.info("Tool found: %s", tool_name)

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
            raise RuntimeError("No tool from %s appeared on %s in %s" % (self.tools_list, self.model_states_topic, os.path.basename(bag_path)))

        for name, stamps in (('pc', pc_t), ('tool', tool_t), ('joy', joy_t), ('wrench', wrench_t), ('arm', arm_t)):
            if not stamps:
                raise RuntimeError("No messages on the %s stream in %s" % (name, os.path.basename(bag_path)))


        # Use pointcloud data to get pose estimates
        # TODO:
        # tool_t, tool_pose_in_pc, tool_twist_in_pc = self.run_pose_estimation(pc_t, pc_points)

        raw = dict(bag_path = bag_path,
                   tool_name = tool_name,
                   data_rate = self.rate_hz,
                   tool = (tool_t, tool_pose_in_pc, tool_twist_in_pc),
                   joy = (joy_t, joy_axes, joy_buttons),
                   wrench = (wrench_t, wrench_cmd),
                   arm = (arm_t, arm_joint, arm_grip))

        d = self.resample(raw, self.rate_hz)

        logger.info("Loading done: %d frames at %.1f Hz", len(d.t), self.rate_hz)

        return d


    def check_runs_consistent(self):
        """Fail on anything that would silently corrupt a pooled dataset, warn on anything that only probably would."""

        if not self.bc_data:
            raise RuntimeError("No runs loaded")

        layouts = {}
        for d in self.bc_data:
            layouts.setdefault((d.joy_axes.shape[1], d.joy_buttons.shape[1]), []).append(d.bag_path)
        if len(layouts) > 1:
            groups = "; ".join("%d axes / %d buttons: %s" % (k[0], k[1], ", ".join(v)) for k, v in sorted(layouts.items()))
            raise RuntimeError("Gamepad layout is not consistent across runs. %s" % groups)

        tools = sorted({d.tool_name for d in self.bc_data})

        if len(tools) > 1:
            logger.warning("Runs span more than one tool (%s). The observation carries no tool identity, so identical observations can carry different correct actions.", ", ".join(tools))

        lengths = np.array([len(d.t) for d in self.bc_data], dtype=float)
        if lengths.max() > 3.0 * lengths.min():
            logger.warning("Run lengths vary by more than 3x (%d to %d frames). Under a flat per-sample loss the long runs dominate.", int(lengths.min()), int(lengths.max()))


    def debug_print_bc_data(self):

        logger.info("BC data info:")

        for d in self.bc_data:
            logger.info(f"Run: {os.path.basename(d.bag_path)}, tool: {d.tool_name}, frames: {len(d.t)}, rate: {d.data_rate:.1f} Hz, duration: {d.t[-1] - d.t[0]:.2f} s")

            for name in ('tool_pos', 'tool_quat', 'tool_lin_vel', 'tool_ang_vel', 'wrench_cmd', 'joy_axes', 'joy_buttons', 'arm_joint_state', 'arm_gripper_state'):
                a = np.asarray(getattr(d, name)).astype(float)
                logger.info(f"  {name:<18} shape: {str(a.shape):<12} min: {a.min():9.4f}, max: {a.max():9.4f}, nan: {int(np.isnan(a).sum())}")

        total_frames = sum(len(d.t) for d in self.bc_data)
        total_secs = sum(d.t[-1] - d.t[0] for d in self.bc_data)
        logger.info(f"Total: {len(self.bc_data)} run(s), {total_frames} frame(s), {total_secs:.1f} s")


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
            raise RuntimeError("Topic time windows do not overlap in %s" % raw['bag_path'])


        d = BcData()
        d.bag_path = raw['bag_path']
        d.data_rate = raw['data_rate']
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

        obs, acts = [], []

        for d in self.bc_data:
            gripper_cmd = np.maximum.accumulate(d.joy_buttons[:, 0].astype(np.int8))

            obs.append(np.column_stack([d.tool_pos, d.tool_quat, d.tool_lin_vel, d.tool_ang_vel]))
            acts.append(np.column_stack([d.wrench_cmd, gripper_cmd]))

        # Observations
        self.obs = np.concatenate(obs).astype(np.float32)
        # Actions, scaled
        self.acts = np.concatenate(acts).astype(np.float32) / self.act_scale

        self.obs_mean = self.obs.mean(axis=0)
        self.obs_sigma = self.obs.std(axis=0)

        # A channel that never moves standardises to 0/0. Leave it at its raw value rather than producing nan.
        flat = self.obs_sigma < 1e-8
        if flat.any():
            logger.warning("Observation channel(s) %s are constant across the corpus and will not be standardised", np.flatnonzero(flat).tolist())
        self.obs_sigma[flat] = 1.0

        self.obs = (self.obs - self.obs_mean) / self.obs_sigma

        logger.info("BC arrays: obs %s, acts %s, from %d run(s)", self.obs.shape, self.acts.shape, len(self.bc_data))
        logger.info("Gripper closed on %.1f%% of frames", 100.0 * self.acts[:, 6].mean())


    def bc_train_test(self):


        num_obs = self.obs.shape[1]
        num_acts = self.acts.shape[1]

        logger.info("Number of observations: %d", num_obs)
        logger.info("Number of actions: %d", num_acts)

        # Build the Spaces
        observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(num_obs,), dtype=np.float32)
        action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(num_acts,), dtype=np.float32)

        # Transitions - Not used for BC, but init with correct size is required
        # transitions = TransitionsMinimal(
        #     obs=self.obs,
        #     acts=self.acts,
        #     infos=np.array([{} for _ in range(len(self.obs))]),
        # )

        # Transitions - Not used for BC, but init with correct size is required
        transitions = Transitions(
            obs=self.obs,
            acts=self.acts,
            infos=np.array([{} for _ in range(len(self.obs))]),
            next_obs=np.roll(self.obs, -1, axis=0),
            dones=np.zeros(len(self.obs), dtype=bool),
        )

        # Random num generator
        rng = np.random.default_rng(0)

        self.bc_trainer = bc.BC(
            observation_space=observation_space,
            action_space=action_space,
            demonstrations=transitions,
            rng=rng,
        )

        #self.bc_trainer.train(n_epochs=1)
        self.n_epochs = 50
        self.bc_trainer.train(n_epochs=self.n_epochs, log_interval=200)


    def save_trained_policy(self):
        """Save policy to file."""

        import torch as th

        out_dir = os.path.join(self.bag_folder, 'bc_policy')
        os.makedirs(out_dir, exist_ok=True)

        policy = self.bc_trainer.policy
        policy_path = os.path.join(out_dir, 'policy.pt')
        #self.bc_trainer.save_policy(policy_path)
        #th.save(policy, policy_path)
        th.save(policy.state_dict(), policy_path)
        logger.info("Saved policy to %s", policy_path)


def main():
    args = get_config()
    bc_first_test = BcFirstTest(args)
    bc_first_test.run()
    return bc_first_test


if __name__ == '__main__':
    # try:
    #     rospy.init_node('bc_first_test')
    #     node = BcFirstTest()
    #     #rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass
    bc_first_test = main()
