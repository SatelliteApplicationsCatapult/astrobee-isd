#!/usr/bin/env python

import math
import numpy as np
import rospy

from astrobee_ros_demo.util import *
import astrobee_joy_teleop.msg

import geometry_msgs.msg
import sensor_msgs.msg
import std_srvs.srv
import ff_msgs.msg
import ff_msgs.srv
import actionlib


# Arm command constants (from ff_msgs/ArmGoal)
ARM_STOP        = 0
ARM_DEPLOY      = 1
ARM_STOW        = 2
ARM_PAN         = 3
ARM_TILT        = 4
ARM_MOVE        = 5
GRIPPER_SET     = 6
GRIPPER_OPEN    = 7
GRIPPER_CLOSE   = 8
DISABLE_SERVO   = 9

# Perch arm
#ARM_ACTION_NS      = "/honey/beh/arm"
#JOINT_STATES_TOPIC = "/honey/joint_states"
PAN_JOINT          = "top_aft_arm_distal_joint"
TILT_JOINT         = "top_aft_arm_proximal_joint"
TILT_JOINT_OFFSET_DEG = 90.0            # ArmGoal tilt = joint angle + 90. Deployed 0, stowed 180
PAN_LIMITS_DEG     = (-90.0, 90.0)
TILT_LIMITS_DEG    = (0.0, 180.0)
STEP_DEG           = 5.0                # D-pad step per ARM_MOVE
SEND_INTERVAL      = 0.1                # Seconds between continuous ARM_MOVE goals


class SimpleControlExample(object):
    """
    Class implementing a custom controller that sends body/arm commands to the NASA Astrobee.
    """

    def __init__(self):
        """
        Initialise controller class
        """

        # Namespace
        self.ns = rospy.get_param("~ns", "honey")

        # Set action/topic/service nammes
        rospy.loginfo("Namespace: %s", self.ns)
        if self.ns:
            self.action_ns = "/{}/beh/arm".format(self.ns)
            self.joint_states_topic = "/{}/joint_states".format(self.ns)
            #self.calibrate_srv = "/{}/hw/arm/calibrate_gripper".format(self.ns)
        else:
            self.action_ns = "/beh/arm"
            self.joint_states_topic = "/joint_states"
            #self.calibrate_srv = "/hw/arm/calibrate_gripper"

        # Initialise parameters from file
        self.max_force = rospy.get_param("/wrench_command/max_force", 0.8)
        self.max_torque = rospy.get_param("/wrench_command/max_torque", 0.05)
        self.mass = rospy.get_param("/robot_sim/mass", 9.7756)
        self.inertia = np.array(rospy.get_param("/robot_sim/inertia", [0.1737, 0.1649, 0.1865]))
        self.lin_vel_decay_time = rospy.get_param("/flight_assist/lin_vel_decay_time", 1.0)
        self.ang_vel_decay_time = rospy.get_param("/flight_assist/ang_vel_decay_time", 1.0)

        # Initialise basic parameters
        self.dt = 1
        self.rate = rospy.Rate(5)
        self.start = False
        self.state = np.zeros((13, 1))
        self.state[9] = 1
        self.t0 = 0.0
        self.twist = None
        self.pose = None

        # Initialise gamepad params
        # Body wrench
        self.joy_wrench = np.zeros((6, ))
        # Perch arm state
        self.joy_arm_prev = None    # last JoyArm message, for edge detection
        self.arm_pan = None         # measured, ArmGoal degrees
        self.arm_tilt = None
        self.cmd_pan = 0.0          # commanded while the D-pad is held, ArmGoal degrees
        self.cmd_tilt = 0.0
        self.dpad_active = False
        self.last_move_ts = 0.0

        # Data timestamps and validity threshold
        self.ts_threshold = 1.0
        self.pose_ts = 0.0
        self.twist_ts = 0.0
        self.joy_wrench_ts = 0.0

        # Set publishers and subscribers
        self.set_services()
        self.set_subscribers_publishers()

        # Change onboard timeout
        new_timeout = ff_msgs.srv.SetFloatRequest()
        new_timeout.data = 1.5
        ans = self.pmc_timeout(new_timeout)
        if not ans.success:
            rospy.logerr("Couldn't change PMC timeout.")
            exit()
        else:
            rospy.loginfo("Timeout updated.")

        self.run()


        # TODO ?
        # Gripper calibration service

        # rospy.loginfo("Waiting for calibrate service: %s", calibrate_srv)

        # try:
        #     rospy.wait_for_service(calibrate_srv, timeout=5.0)
        #     self._cal_srv = rospy.ServiceProxy(calibrate_srv, CalibrateGripper)
        #     rospy.loginfo("Calibrate service ready.")
        # except rospy.ROSException:
        #     rospy.logwarn(
        #         "Calibrate service '%s' not found.")
        #     self._cal_srv = None

        # Always calibrate at start
        # self._calibrate()




    # TODO ?
    # def _calibrate(self):
    #     if self._cal_srv is None:
    #         rospy.logwarn("Calibrate service unavailable — check CALIBRATE_SRV topic name.")
    #         return
    #     try:
    #         resp = self._cal_srv(CalibrateGripperRequest())
    #         rospy.loginfo("Gripper calibration response: %s", resp)
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Calibrate service call failed: %s", e)




    # ---------------------------------
    # BEGIN: Callbacks Section
    # ---------------------------------

    def pose_sub_cb(self, msg=geometry_msgs.msg.PoseStamped()):
        """
        Pose callback to update the agent's position and attitude.

        :param msg: estimated pose, defaults to geometry_msgs.msg.PoseStamped()
        :type msg: geometry_msgs.msg.PoseStamped
        """

        self.pose_ts = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        self.pose = np.array([[msg.pose.position.x,
                               msg.pose.position.y,
                               msg.pose.position.z,
                               msg.pose.orientation.x,
                               msg.pose.orientation.y,
                               msg.pose.orientation.z,
                               msg.pose.orientation.w]]).T
        # Update state variable
        self.state[0:3] = self.pose[0:3]
        self.state[6:10] = self.pose[3:7]
        return


    def twist_sub_cb(self, msg=geometry_msgs.msg.TwistStamped()):
        """
        Twist callback to update the agent's linear and angular velocities.

        :param msg: estimated velocities
        :type msg: geometry_msgs.msg.TwistStamped
        """

        self.twist_ts = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        self.twist = np.array([[msg.twist.linear.x,
                                msg.twist.linear.y,
                                msg.twist.linear.z,
                                msg.twist.angular.x,
                                msg.twist.angular.y,
                                msg.twist.angular.z]]).T
        # Update state variable
        self.state[3:6] = self.twist[0:3]
        self.state[10:13] = self.twist[3:6]
        return


    def joy_wrench_sub_cb(self, msg=geometry_msgs.msg.WrenchStamped()):
        """
        Joystick callback to update the force/torque control messages.

        :param msg: wrench command from joystick
        :type msg: geometry_msgs.msg.WrenchStamped
        """

        self.joy_wrench_ts = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
        self.joy_wrench = np.array([msg.wrench.force.x,
                                    msg.wrench.force.y,
                                    msg.wrench.force.z,
                                    msg.wrench.torque.x,
                                    msg.wrench.torque.y,
                                    msg.wrench.torque.z])
        return


    def joint_states_sub_cb(self, msg=sensor_msgs.msg.JointState()):
        """
        Joint state callback to track the perch arm's measured pan and tilt,
        converted to the ArmGoal convention (degrees).

        :param msg: robot joint states
        :type msg: sensor_msgs.msg.JointState
        """

        try:
            pan = msg.position[msg.name.index(PAN_JOINT)]
            tilt = msg.position[msg.name.index(TILT_JOINT)]
        except (ValueError, IndexError):
            return

        self.arm_pan = math.degrees(pan)
        self.arm_tilt = math.degrees(tilt) + TILT_JOINT_OFFSET_DEG
        return


    def joy_arm_sub_cb(self, msg=astrobee_joy_teleop.msg.JoyArm()):
        """
        Perch arm joystick callback. Input is held button levels; goals are
        sent on 0->1 transitions. D-pad steps pan/tilt while held, starting
        from the arm's measured pose.

        :param msg: perch arm joystick state
        :type msg: astrobee_joy_teleop.msg.JoyArm
        """

        ts = msg.header.stamp.to_sec()
        prev = self.joy_arm_prev
        self.joy_arm_prev = msg

        if prev is None:
            self.dpad_active = False
            return

        if msg.deploy and not prev.deploy:
            rospy.loginfo("ARM: Deploy")
            self.send_arm_goal(ARM_DEPLOY)

        if msg.stow and not prev.stow:
            rospy.loginfo("ARM: Stow")
            self.send_arm_goal(ARM_STOW)

        if msg.gripper_open and not prev.gripper_open:
            rospy.loginfo("ARM: Gripper Open")
            self.send_arm_goal(GRIPPER_OPEN)

        if msg.gripper_close and not prev.gripper_close:
            rospy.loginfo("ARM: Gripper Close")
            self.send_arm_goal(GRIPPER_CLOSE)

        # D-pad pan/tilt
        dpad = msg.pan != 0 or msg.tilt != 0
        prev_dpad = prev.pan != 0 or prev.tilt != 0

        if not dpad:
            self.dpad_active = False
            return

        # Neutral -> pressed: start stepping from the measured pose
        if not prev_dpad:
            if self.arm_pan is None:
                rospy.logwarn("ARM: no joint states on " + self.joint_states_topic + ", D-pad ignored")
                return
            self.cmd_pan = self.arm_pan
            self.cmd_tilt = self.arm_tilt
            self.dpad_active = True
            self.last_move_ts = 0.0

        # Held since baseline, or no joint states at press
        if not self.dpad_active:
            return

        if ts - self.last_move_ts < SEND_INTERVAL:
            return

        self.cmd_pan = float(np.clip(self.cmd_pan + msg.pan * STEP_DEG, *PAN_LIMITS_DEG))
        self.cmd_tilt = float(np.clip(self.cmd_tilt + msg.tilt * STEP_DEG, *TILT_LIMITS_DEG))
        self.last_move_ts = ts
        rospy.loginfo("ARM_MOVE pan=%.1f tilt=%.1f", self.cmd_pan, self.cmd_tilt)
        self.send_arm_goal(ARM_MOVE, pan=self.cmd_pan, tilt=self.cmd_tilt)
        return


    def start_srv_callback(self, req=std_srvs.srv.SetBoolRequest()):
        """
        Service to start the operation of the autonomous control.

        :param req: request state
        :type req: std_srvs.srv.SetBoolRequest
        :return: success at starting
        :rtype: std_srvs.srv.SetBoolResponse
        """
        state = req.data
        ans = std_srvs.srv.SetBoolResponse()
        if state:
            ans.success = True
            ans.message = "Node started!"

            self.t0 = rospy.get_time()
            # Disable onboard controller
            obc = std_srvs.srv.SetBoolRequest()
            obc.data = False
            self.onboard_ctl(obc)
            self.start = True
        else:
            ans.success = True
            ans.message = "Node stopped!"
            # Enable onboard controller
            obc = std_srvs.srv.SetBoolRequest()
            obc.data = True
            self.onboard_ctl(obc)
            self.start = False

        return ans

    # ---------------------------------
    # END: Callbacks Section
    # ---------------------------------


    def set_subscribers_publishers(self):
        """
        Helper function to create all publishers and subscribers.
        """

        # Subscribers
        self.pose_sub = rospy.Subscriber("~pose_topic",
                                         geometry_msgs.msg.PoseStamped,
                                         self.pose_sub_cb)
        self.twist_sub = rospy.Subscriber("~twist_topic",
                                          geometry_msgs.msg.TwistStamped,
                                          self.twist_sub_cb)
        self.joy_wrench_sub = rospy.Subscriber("/joy_wrench",
                                                geometry_msgs.msg.WrenchStamped,
                                                self.joy_wrench_sub_cb)
        self.joy_arm_sub = rospy.Subscriber("/joy_arm",
                                            astrobee_joy_teleop.msg.JoyArm,
                                            self.joy_arm_sub_cb,
                                            queue_size=10)
        self.joint_states_sub = rospy.Subscriber(self.joint_states_topic,
                                                 sensor_msgs.msg.JointState,
                                                 self.joint_states_sub_cb,
                                                 queue_size=10)

        # Publishers
        self.control_pub = rospy.Publisher("~control_topic",
                                           ff_msgs.msg.FamCommand,
                                           queue_size=1, latch=True)

        self.flight_mode_pub = rospy.Publisher("~flight_mode",
                                               ff_msgs.msg.FlightMode,
                                               queue_size=1)


    def set_services(self):
        """
        Helper function to create all services.
        """

        # Astrobee control disable and timeout change service
        self.onboard_ctl = rospy.ServiceProxy("~onboard_ctl_enable_srv",
                                              std_srvs.srv.SetBool)
        self.pmc_timeout = rospy.ServiceProxy("~pmc_timeout_srv",
                                              ff_msgs.srv.SetFloat)

        # Perch arm action client
        rospy.loginfo("Waiting for arm action server: %s", self.action_ns)
        self.arm_client = actionlib.SimpleActionClient(self.action_ns, ff_msgs.msg.ArmAction)
        if not self.arm_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Arm action server not available: %s" % self.action_ns)
            exit()
        else:
            rospy.loginfo("Arm action server connected.")

        # Start service
        self.start_service = rospy.Service("~start_srv", std_srvs.srv.SetBool,
                                           self.start_srv_callback)

        # Wait for services
        self.onboard_ctl.wait_for_service()
        self.pmc_timeout.wait_for_service()


    # def _send_goal(self, command, pan=0.0, tilt=0.0, gripper=0.0):
    #     # Fire-and-forget: send a new goal, preempting the current one
    #     goal = ArmGoal()
    #     goal.command  = command
    #     goal.pan      = float(pan)
    #     goal.tilt     = float(tilt)
    #     goal.gripper  = float(gripper)
    #     self.client.send_goal(goal)



    def send_arm_goal(self, command, pan=0.0, tilt=0.0):
        """
        Fire-and-forget: send a new arm goal, preempting the current one.

        :param command: ArmGoal command
        :type command: int
        :param pan: pan angle, degrees
        :type pan: float
        :param tilt: tilt angle, degrees
        :type tilt: float
        """

        goal = ff_msgs.msg.ArmGoal()
        goal.command = command
        goal.pan = float(pan)
        goal.tilt = float(tilt)
        self.arm_client.send_goal(goal)


    def check_data_validity(self):
        """
        Helper function to check the data validity.

        :return: True if data is valid, False otherwise
        :rtype: boolean
        """

        pos_val = False
        vel_val = False

        # Check state validity
        if rospy.get_time() - self.pose_ts < self.ts_threshold:
            pos_val = True
        if rospy.get_time() - self.twist_ts < self.ts_threshold:
            vel_val = True
        # TODO(@User): Add further conditions that are needed for control update

        if pos_val is False or vel_val is False:
            rospy.logwarn("Skipping control. Validity flags:\nPos: "
                          + str(pos_val) + "; Vel: " + str(vel_val))

        return pos_val and vel_val


    def create_control_message(self):
        """
        Helper function to create the control message to be published

        :return: control input to vehicle
        :rtype: ff_msgs.msg.FamCommand()
        """

        # Create message
        u = ff_msgs.msg.FamCommand()

        # Fill header
        u.header.frame_id = 'body'
        u.header.stamp = rospy.Time.now()

        # Fill force / torque messages
        u.wrench.force.x = self.u_traj[0]
        u.wrench.force.y = self.u_traj[1]
        u.wrench.force.z = self.u_traj[2]
        u.wrench.torque.x = self.u_traj[3]
        u.wrench.torque.y = self.u_traj[4]
        u.wrench.torque.z = self.u_traj[5]

        # Set control mode and status
        u.status = 3
        u.control_mode = 2

        return u


    def world_to_body(self, v):
        """
        Rotate a world-frame vector into the body frame using the current
        attitude estimate.

        :param v: world-frame vector
        :type v: numpy.ndarray, shape (3,)
        :return: body-frame vector
        :rtype: numpy.ndarray, shape (3,)
        """

        # state[6:10] is the body-to-world quaternion, (x, y, z, w).
        # Conjugating it gives world-to-body.
        q = self.state[6:10, 0]
        u = -q[0:3]
        w = q[3]

        t = 2.0 * np.cross(u, v)
        return v + w * t + np.cross(u, t)


    def damping_wrench(self):
        """
        Flight assist. Returns a body-frame wrench opposing the current
        velocity, clamped per axis to match the joystick converter's limits.

        :return: damping wrench, force then torque
        :rtype: numpy.ndarray, shape (6,)
        """

        v_world = self.state[3:6, 0]
        w_body = self.state[10:13, 0]

        force = self.world_to_body(-(self.mass / self.lin_vel_decay_time) * v_world)
        torque = - (self.inertia / self.ang_vel_decay_time) * w_body

        return np.concatenate((np.clip(force, -self.max_force, self.max_force),
                               np.clip(torque, -self.max_torque, self.max_torque)))


    def create_flight_mode_message(self):
        """
        Helper function to create the flight mode message.

        Here we use the flight-mode difficult to have full access to the
        actuation capabilities of the Astrobee.
        """

        fm = ff_msgs.msg.FlightMode()

        fm.name = "difficult"

        fm.control_enabled = False

        fm.att_ki = Vec(0.002, 0.002, 0.002)
        fm.att_kp = Vec(4.0, 4.0, 4.0)
        fm.omega_kd = Vec(3.2, 3.2, 3.2)

        fm.pos_kp = Vec(.6, .6, .6)
        fm.pos_ki = Vec(0.0001, 0.0001, 0.0001)
        fm.vel_kd = Vec(1.2, 1.2, 1.2)

        fm.speed = 3

        fm.tolerance_pos = 0.2
        fm.tolerance_vel = 0
        fm.tolerance_att = 0.3490
        fm.tolerance_omega = 0
        fm.tolerance_time = 1.0

        fm.hard_limit_accel = 0.0200
        fm.hard_limit_omega = 0.5236
        fm.hard_limit_alpha = 0.2500
        fm.hard_limit_vel = 0.4000

        return fm


    def run(self):
        """
        Main operation loop.
        """

        while not rospy.is_shutdown():

            # Only do something when started
            if self.start is False:
                self.rate.sleep()
                rospy.loginfo("Sleeping...")
                continue

            # Use self.pose, self.twist to generate a control input
            t = rospy.get_time() - self.t0
            val = self.check_data_validity()
            if val is False:
                self.rate.sleep()
                continue

            # - Start of the control section
            #
            # Here the user should modify the variable u_traj that is posteriorly sent
            # to the robot. The variable u_traj is an array containing a 3D force
            # (on u_traj[0:3]) and 3D torque (on u_traj[3:]).
            tin = rospy.get_time()

            # self.u_traj = np.zeros((6, ))  # TODO(@User): use your controller here
            self.u_traj = self.joy_wrench + self.damping_wrench()

            tout = rospy.get_time() - tin
            rospy.loginfo("Time for control: " + str(tout))

            # Create control input message
            u = self.create_control_message()
            fm = self.create_flight_mode_message()

            # Publish control
            self.control_pub.publish(u)
            self.flight_mode_pub.publish(fm)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("node_template")
    dmpc = SimpleControlExample()
    rospy.spin()
