#!/usr/bin/env python3
"""
Usage: tf_echo_precise.py source_frame target_frame [precision] [rate_hz]
Prints the pose of target_frame expressed in source_frame (same as tf_echo).
"""

import sys
import math
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion


def main():
    args = rospy.myargv(argv=sys.argv)
    if len(args) < 3:
        print("Usage: tf_echo_precise.py source_frame target_frame [precision] [rate_hz]")
        sys.exit(1)

    src, tgt = args[1], args[2]
    prec = int(args[3]) if len(args) > 3 else 6
    rate_hz = float(args[4]) if len(args) > 4 else 1.0

    rospy.init_node("tf_echo_precise", anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rate = rospy.Rate(rate_hz)
    f = f".{prec}f"

    while not rospy.is_shutdown():
        try:
            t = buf.lookup_transform(src, tgt, rospy.Time(0), rospy.Duration(1.0))
            p, q = t.transform.translation, t.transform.rotation
            r, pi, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
            print(f"At time {t.header.stamp.to_sec():.3f}")
            print(f"- Translation: [{p.x:{f}}, {p.y:{f}}, {p.z:{f}}]")
            print(f"- Rotation: in Quaternion [{q.x:{f}}, {q.y:{f}}, {q.z:{f}}, {q.w:{f}}]")
            print(f"            in RPY (radian) [{r:{f}}, {pi:{f}}, {y:{f}}]")
            print(f"            in RPY (degree) [{math.degrees(r):{f}}, "
                  f"{math.degrees(pi):{f}}, {math.degrees(y):{f}}]")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(str(e))
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == "__main__":
    main()
