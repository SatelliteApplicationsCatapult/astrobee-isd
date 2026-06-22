#!/usr/bin/env python3
import rospy
import actionlib
from sensor_msgs.msg import Joy
##from ff_msgs.msg import FamCommand
from geometry_msgs.msg import WrenchStamped
from ff_msgs.msg import ArmAction, ArmGoal
from ff_hw_msgs.srv import CalibrateGripper, CalibrateGripperRequest



NS = rospy.get_param("~ns", "honey")


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


PAN_MAX_DEG    = 90.0
TILT_MAX_DEG   = 90.0
#DEADZONE       = 1.0
SEND_INTERVAL  = 0.1       # seconds between continuous arm goals



BTN_CLOSE       = 0   # A
BTN_OPEN        = 1   # B
BTN_STOW        = 2   # X
BTN_DEPLOY      = 3   # Y
# BTN_CALIBRATE   = 7   # START



AXIS_PAN   = 6        # D-Pad Left/Right, +ve is Left
AXIS_TILT   = 7       # D-Pad Up/Down, +ve is Up
AXIS_FORCE_X = 1      # Left Stick Up/Down (Forward/Backward)
AXIS_FORCE_Y = 0      # Left Stick Left/Right (Strafe Left/Right), +ve is Left
AXIS_FORCE_Z_POS = 5  # Right Trigger (Up), -ve is fully-pressed
AXIS_FORCE_Z_NEG = 4  # Left Trigger (Down), -ve is fully-pressed
BTN_TORQUE_X_POS = 4  # Left Shoulder Button (Roll Left)
BTN_TORQUE_X_NEG = 5  # Right Shoulder Button (Roll Right)
AXIS_TORQUE_Y = 3     # Right Stick Up/Down (Pitch)
AXIS_TORQUE_Z = 2     # Right Stick Left/Right (Yaw)


# If facing "forwards", i.e. where nav cam is pointing, then in Astrobee body frame:
#   forward: +X
#   upward:  -Z
# CONTROL_DIRECTION = 1
# If facing "backwards", i.e. where dock cam is pointing, for better control of perch arm, then in Astrobee body frame:
#   forward: -X
#   upward:  -Z
CONTROL_DIRECTION = -1



# To Invert Y axis for aircraft-style control of pitch, set this to -1
Y_AXIS_DIRECTION = 1






class AstrobeeDirectForceTeleop:
    def __init__(self):
        rospy.init_node('astrobee_direct_teleop', anonymous=True)

        # Max limits based on Astrobee's physical blower limits
        self.max_force = 0.8   # Newtons
        self.max_torque = 0.05 # Newton-meters


        self._last_buttons = []
        self._last_send    = rospy.Time(0)
        self._current_pan  = 0.0
        self._current_tilt = 0.0



#    def post_init(self):

        rospy.loginfo("NS: %s", NS)

        if NS:
            action_ns = "/{}/beh/arm".format(NS)
            calibrate_srv = "/" + NS + "/hw/arm/calibrate_gripper"
        else:
            action_ns = "/beh/arm".format(NS)
            calibrate_srv = "/hw/arm/calibrate_gripper"

        # Arm action client
        rospy.loginfo("Waiting for arm action server: %s", action_ns)
        self.client = actionlib.SimpleActionClient(action_ns, ArmAction)
        self.client.wait_for_server(rospy.Duration(10.0))
        rospy.loginfo("Arm action server connected.")


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



        # Send a stop command, also get the initial pan/tilt values:
        # self._send_goal(GRIPPER_OPEN)
        # self._send_goal(GRIPPER_CLOSE)
        # self._send_goal(ARM_STOP)

        # self.client.wait_for_result(rospy.Duration(2))

        # self.client.get_result()
        # print(self.client.get_result())

        # print("here")



        # Publisher to Astrobee's actual GNC / FAM subsystem
        self.cmd_pub = rospy.Publisher('/joy_wrench', WrenchStamped, queue_size=10)

        # Subscribe to your host joystick node
        rospy.Subscriber('/joy', Joy, self.joy_callback)




        rospy.loginfo("Astrobee Direct Force/Torque 6 DoF Controller Initialized.")





    def joy_callback(self, msg):

        # # Safety switch: If deadman isn't held down, command complete zero wrench
        # if not msg.buttons[self.BUTTON_DEADMAN]:
        #     self.publish_zero_wrench()
        #     return

        buttons = list(msg.buttons)
        axes    = list(msg.axes)




        # if self._button_pressed(buttons, BTN_CALIBRATE):
        #     rospy.loginfo("ARM: Calibrate gripper (service call)")
        #     self._calibrate()

        if self._button_pressed(buttons, BTN_DEPLOY):
            rospy.loginfo("ARM: Deploy")
            # self._current_pan  = 0.0
            # self._current_tilt = 0.0
            self._send_goal(ARM_DEPLOY)

        if self._button_pressed(buttons, BTN_STOW):
            rospy.loginfo("ARM: Stow")
            self._send_goal(ARM_STOW)

        if self._button_pressed(buttons, BTN_OPEN):
            rospy.loginfo("ARM: Gripper OPEN")
            self._send_goal(GRIPPER_OPEN)

        if self._button_pressed(buttons, BTN_CLOSE):
            rospy.loginfo("ARM: Gripper CLOSE")
            self._send_goal(GRIPPER_CLOSE)




        # # D-Pad buttons are part of the axes list, but only have value 0 or 1
        # if self._button_pressed(axes, AXIS_PAN):
        #     rospy.loginfo("ARM: Pan")
        #     self._send_goal()

        # if self._button_pressed(axes, AXIS_TILT):
        #     rospy.loginfo("ARM: Tilt")
        #     self._send_goal(GRIPPER_CLOSE)


        # pan_raw  = self._deadzone(axes[AXIS_PAN])#  if len(axes) > AXIS_PAN  else 0.0
        # tilt_raw = self._deadzone(axes[AXIS_TILT])# if len(axes) > AXIS_TILT else 0.0


        pan_raw = - axes[AXIS_PAN]
        tilt_raw = axes[AXIS_TILT]

        # rospy.loginfo("pan_raw: " + str(pan_raw))
        # rospy.loginfo("pan_tilt: " + str(tilt_raw))



        if pan_raw != 0.0 or tilt_raw != 0.0:
            now = rospy.Time.now()
            if (now - self._last_send).to_sec() >= SEND_INTERVAL:
                self._current_pan  = max(-PAN_MAX_DEG,
                                        min( PAN_MAX_DEG,
                                            self._current_pan  + pan_raw  * 5.0))
                self._current_tilt = max(-TILT_MAX_DEG,
                                        min( TILT_MAX_DEG,
                                            self._current_tilt + tilt_raw * 5.0))
                rospy.logdebug("ARM_MOVE pan=%.1f tilt=%.1f",
                                self._current_pan, self._current_tilt)
                self._send_goal(ARM_MOVE,
                                pan=self._current_pan,
                                tilt=self._current_tilt)
                self._last_send = now






        cmd = WrenchStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "body"  # Apply forces relative to robot body frame

        # Control mode from GNC ICD (1 to parse wrench fields)
        # cmd.control_mode = 2
        # cmd.status = 0

        # 1. Map Analog Sticks to Linear Forces (Newtons)
        cmd.wrench.force.x = axes[AXIS_FORCE_X] * self.max_force
        cmd.wrench.force.y = - axes[AXIS_FORCE_Y] * self.max_force
        #cmd.wrench.force.z = axes[AXIS_FORCE_Z] * self.max_force
        cmd.wrench.force.z = - ((1 - axes[AXIS_FORCE_Z_POS])/2.0 - (1 - axes[AXIS_FORCE_Z_NEG])/2.0) * self.max_force


        # 2. Map Analog/D-pad to Rotational Torques (Nm)
        #cmd.wrench.torque.x = axes[AXIS_TORQUE_X] * self.max_torque  # Roll
        cmd.wrench.torque.x = - (buttons[BTN_TORQUE_X_POS] - buttons[BTN_TORQUE_X_NEG]) * self.max_torque  # Roll
        cmd.wrench.torque.y = axes[AXIS_TORQUE_Y] * self.max_torque  # Pitch
        cmd.wrench.torque.z = - axes[AXIS_TORQUE_Z] * self.max_torque  # Yaw

        # Invert pitch if desired
        cmd.wrench.torque.y *= Y_AXIS_DIRECTION

        # Flip X & Y axes if facing backwards
        cmd.wrench.force.x *= CONTROL_DIRECTION
        cmd.wrench.force.y *= CONTROL_DIRECTION
        cmd.wrench.torque.x *= CONTROL_DIRECTION
        cmd.wrench.torque.y *= CONTROL_DIRECTION






        # rospy.loginfo("cmd.wrench.force.x: " + str(cmd.wrench.force.x))
        # rospy.loginfo("cmd.wrench.force.y: " + str(cmd.wrench.force.y))
        # rospy.loginfo("cmd.wrench.force.z: " + str(cmd.wrench.force.z))
        # rospy.loginfo("cmd.wrench.torque.x: " + str(cmd.wrench.torque.x))
        # rospy.loginfo("cmd.wrench.torque.y: " + str(cmd.wrench.torque.y))
        # rospy.loginfo("cmd.wrench.torque.z: " + str(cmd.wrench.torque.z))

        self.cmd_pub.publish(cmd)


        self._last_buttons = buttons




    # def publish_zero_wrench(self):
    #     stop_cmd = FamCommand()
    #     stop_cmd.header.stamp = rospy.Time.now()
    #     stop_cmd.header.frame_id = "body"
    #     stop_cmd.control_mode = 1
    #     stop_cmd.status = 0
    #     # Nested geometry_msgs/Wrench fields default to 0.0 automatically
    #     self.cmd_pub.publish(stop_cmd)



    def _send_goal(self, command, pan=0.0, tilt=0.0, gripper=0.0):
        """Fire-and-forget: send a new goal, preempting the current one."""
        goal = ArmGoal()
        goal.command  = command
        goal.pan      = float(pan)
        goal.tilt     = float(tilt)
        goal.gripper  = float(gripper)
        self.client.send_goal(goal)




    def _button_pressed(self, buttons, idx):
        if idx >= len(buttons):
            return False
        prev = self._last_buttons[idx] if idx < len(self._last_buttons) else 0
        return buttons[idx] == 1 and prev == 0






    # def _calibrate(self):
    #     if self._cal_srv is None:
    #         rospy.logwarn("Calibrate service unavailable — check CALIBRATE_SRV topic name.")
    #         return
    #     try:
    #         resp = self._cal_srv(CalibrateGripperRequest())
    #         rospy.loginfo("Gripper calibration response: %s", resp)
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Calibrate service call failed: %s", e)






if __name__ == '__main__':
    try:
        teleop = AstrobeeDirectForceTeleop()
        # teleop.post_init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass