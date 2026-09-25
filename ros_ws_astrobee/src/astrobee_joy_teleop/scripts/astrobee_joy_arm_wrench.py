#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped
from astrobee_joy_teleop.msg import JoyArm


# Perch arm control button mappings
BTN_CLOSE       = 0   # A
BTN_OPEN        = 1   # B
BTN_STOW        = 2   # X
BTN_DEPLOY      = 3   # Y
BTN_CALIBRATE   = 7   # START

# Astrobee control button mappings
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

# Thresholds and parameters
CONTROL_DIRECTION = -1  # -1 for backwards driving, useful when controlling perch arm
Y_AXIS_DIRECTION = 1    # To Invert Y axis for aircraft-style control of pitch, set this to -1

# CONTROL_DIRECTION = 1:
#   If facing "forwards", i.e. where nav cam is pointing, then in Astrobee body frame:
#   forward: +X
#   upward:  -Z
# CONTROL_DIRECTION = -1:
#   If facing "backwards", i.e. where perch cam is pointing, then in Astrobee body frame:
#   forward: -X
#   upward:  -Z


class AstrobeeJoyArmWrench:
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

        # Subscribe to host joystick node
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        rospy.loginfo("Astrobee 6-DoF joystick controller initialised. " \
                       "To control Astrobee's body/arm, make sure a client is running which " \
                       "subscribes to the WrenchStamped and JoyArm messages sent from here.")


    def joy_callback(self, msg):

        buttons = list(msg.buttons)
        axes    = list(msg.axes)
        now = rospy.Time.now()


        # Perch arm state

        arm = JoyArm()
        arm.header.stamp  = now
        arm.calibrate     = buttons[BTN_CALIBRATE] == 1
        arm.deploy        = buttons[BTN_DEPLOY] == 1
        arm.stow          = buttons[BTN_STOW] == 1
        arm.gripper_open  = buttons[BTN_OPEN] == 1
        arm.gripper_close = buttons[BTN_CLOSE] == 1

        # Flip raw values if necessary
        arm.pan  = int(round(- axes[AXIS_PAN]))
        arm.tilt = int(round(axes[AXIS_TILT]))

        self.arm_pub.publish(arm)


        # Wrench output for Astrobee body control

        cmd = WrenchStamped()
        cmd.header.stamp = now
        # Apply forces relative to robot body frame
        if self.ns:
            cmd.header.frame_id = "{}/body".format(self.ns)
        else:
            cmd.header.frame_id = "body"

        # Map analog sticks to linear forces (N)
        cmd.wrench.force.x = axes[AXIS_FORCE_X] * self.max_force
        cmd.wrench.force.y = - axes[AXIS_FORCE_Y] * self.max_force
        cmd.wrench.force.z = - ((1 - axes[AXIS_FORCE_Z_POS])/2.0 - (1 - axes[AXIS_FORCE_Z_NEG])/2.0) * self.max_force

        # Map analog sticks and shoulder buttons to torques (Nm)
        cmd.wrench.torque.x = - (buttons[BTN_TORQUE_X_POS] - buttons[BTN_TORQUE_X_NEG]) * self.max_torque  # Roll
        cmd.wrench.torque.y = axes[AXIS_TORQUE_Y] * self.max_torque                                        # Pitch
        cmd.wrench.torque.z = - axes[AXIS_TORQUE_Z] * self.max_torque                                      # Yaw

        # Invert pitch if desired
        cmd.wrench.torque.y *= Y_AXIS_DIRECTION

        # Flip X & Y axes if facing backwards
        cmd.wrench.force.x *= CONTROL_DIRECTION
        cmd.wrench.force.y *= CONTROL_DIRECTION
        cmd.wrench.torque.x *= CONTROL_DIRECTION
        cmd.wrench.torque.y *= CONTROL_DIRECTION

        self.wrench_cmd_pub.publish(cmd)


if __name__ == '__main__':
    try:
        rospy.init_node('astrobee_joy_arm_wrench')
        node = AstrobeeJoyArmWrench()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
