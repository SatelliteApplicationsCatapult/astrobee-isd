#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
from ff_msgs.msg import CommandStamped, CommandArg, CommandConstants
from tf.transformations import quaternion_from_euler


class AstrobeeWrenchToTeleop(object):
    def __init__(self):

        # Namespace (must match how you started the robot/sim, e.g. "honey")
        self.ns = rospy.get_param("~ns", "honey")

        # Teleop parameters
        self.rate_hz =    1.0
        self.lin_scale =  0.1/0.8   # m   / N
        self.rot_scale =  0.1/0.05  # rad / Nm
        self.deadband =   0.001

        # State
        self.last_wrench = None
        self.last_wrench_time = None
        self.last_cmd_time = rospy.Time.now()

        if self.ns:
            self.command = self.ns + "/command"
        else:
            self.command = "command"

        # Publisher to Astrobee command bus
        self.cmd_pub = rospy.Publisher(self.command, CommandStamped, queue_size=10)

        # Subscribe to joystick wrench
        rospy.Subscriber("/joy_wrench", WrenchStamped, self.wrench_cb)

        rospy.loginfo("Astrobee wrench to teleop command converter initialised.")

        self.loop()


    def wrench_cb(self, msg):
        self.last_wrench = msg
        self.last_wrench_time = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()


    def loop(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.last_wrench is not None and self.last_wrench_time is not None:

                w = self.last_wrench.wrench

                dx = w.force.x * self.lin_scale
                dy = w.force.y * self.lin_scale
                dz = w.force.z * self.lin_scale

                droll  = w.torque.x * self.rot_scale
                dpitch = w.torque.y * self.rot_scale
                dyaw   = w.torque.z * self.rot_scale

                # Only send if non‑trivial motion requested
                if abs(dx) + abs(dy) + abs(dz) + abs(droll) +abs(dpitch) + abs(dyaw) > self.deadband:
                    cmd = self.build_simple_move_6dof(dx, dy, dz, droll, dpitch, dyaw)
                    self.cmd_pub.publish(cmd)

            rate.sleep()


    def build_simple_move_6dof(self, dx, dy, dz, droll, dpitch, dyaw):
        """
        Build a Mobility.simpleMove6DOF CommandStamped for a RELATIVE move in the body frame.
        """

        cmd = CommandStamped()
        cmd.header.stamp = rospy.Time.now()

        cmd.cmd_name = CommandConstants.CMD_NAME_SIMPLE_MOVE6DOF
        cmd.cmd_id = "wrench_to_teleop_%d" % cmd.header.stamp.to_nsec()
        cmd.cmd_src = "guest_science"
        cmd.cmd_origin = ""
        cmd.subsys_name = "Astrobee"

        cmd.args = [CommandArg() for _ in range(4)]

        # Arg 0: referenceFrame (string)
        cmd.args[0].data_type = CommandArg.DATA_TYPE_STRING
        if self.ns:
            cmd.args[0].s = self.ns + "/body"
        else:
            cmd.args[0].s = "body"

        # Arg 1: xyz (vec3d) – relative translation in body frame (meters)
        cmd.args[1].data_type = CommandArg.DATA_TYPE_VEC3d
        cmd.args[1].vec3d[0] = dx
        cmd.args[1].vec3d[1] = dy
        cmd.args[1].vec3d[2] = dz

        # Arg 2: xyzTolerance (vec3d) – currently unused, set to zeros
        cmd.args[2].data_type = CommandArg.DATA_TYPE_VEC3d
        cmd.args[2].vec3d[0] = 0.0
        cmd.args[2].vec3d[1] = 0.0
        cmd.args[2].vec3d[2] = 0.0

        qx, qy, qz, qw = quaternion_from_euler(droll, dpitch, dyaw)  # RPY -> q xyzw

        # Arg 3: rot (mat33f) – quaternion (x,y,z,w) in first 4 entries
        # For a pure relative translation with no attitude change, use identity.
        cmd.args[3].data_type = CommandArg.DATA_TYPE_MAT33f
        cmd.args[3].mat33f[0] = qx
        cmd.args[3].mat33f[1] = qy
        cmd.args[3].mat33f[2] = qz
        cmd.args[3].mat33f[3] = qw
        # Remaining entries unused but must exist
        for i in range(4, 9):
            cmd.args[3].mat33f[i] = 0.0

        return cmd


if __name__ == "__main__":
    try:
        rospy.init_node("astrobee_wrench_to_teleop")
        node = AstrobeeWrenchToTeleop()
        #rospy.spin()  # Uses rospy.Rate
    except rospy.ROSInterruptException:
        pass
