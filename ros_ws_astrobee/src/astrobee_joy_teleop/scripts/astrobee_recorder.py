#!/usr/bin/env python3
import os
from datetime import datetime

import rospy
import rosbag
import subprocess
import threading

from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger, TriggerResponse
from astrobee_joy_teleop.srv import PlayBag, PlayBagResponse


class AstrobeeRecorder:
    def __init__(self):

        self.ns = rospy.get_param("~ns", "honey")
        self.output_dir = rospy.get_param("~output_dir", "/tmp/astrobee_trials")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "hw/depth_perch/points")

        self.is_recording = False
        self.bag_lock = threading.Lock()
        self.play_proc = None
        self.bag = None
        self.bag_path = None

        os.makedirs(self.output_dir, exist_ok=True)

        self.topics = {
            self.pointcloud_topic: PointCloud2,
            #"/tf": TFMessage,
            #"/tf_static": TFMessage,
        }

        self.subscribers = []
        for topic, msg_type in self.topics.items():
            sub = rospy.Subscriber(topic, msg_type, self.rec_callback, callback_args=topic, queue_size=50)
            self.subscribers.append(sub)

        self.start_srv = rospy.Service("~start", Trigger, self.handle_start)
        self.stop_srv = rospy.Service("~stop", Trigger, self.handle_stop)
        self.play_srv = rospy.Service("~play_bag", PlayBag, self.handle_play_bag)

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Namespace: %s", self.ns)
        rospy.loginfo("Record folder location: %s", self.output_dir)
        rospy.loginfo("Pointcloud topic: %s", self.pointcloud_topic)
        rospy.loginfo("Astrobee recorder initialised.")


    def handle_start(self, req):
        with self.bag_lock:
            if self.is_recording:
                return TriggerResponse(
                    success=False,
                    message="Already recording"
                )

            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            trial_name = f"trial_{stamp}"
            self.bag_path = os.path.join(self.output_dir, f"{trial_name}.bag")

            try:
                self.bag = rosbag.Bag(self.bag_path, "w")
                self.is_recording = True
                rospy.loginfo(f"Started recording: {self.bag_path}")
                return TriggerResponse(
                    success=True,
                    message=f"Started recording: {self.bag_path}"
                )
            except Exception as e:
                self.bag = None
                self.bag_path = None
                self.is_recording = False
                return TriggerResponse(
                    success=False,
                    message=f"Failed to start recording: {e}"
                )


    def handle_stop(self, req):
        with self.bag_lock:
            if not self.is_recording:
                return TriggerResponse(
                    success=False,
                    message="Not currently recording"
                )

            try:
                self.is_recording = False
                if self.bag is not None:
                    self.bag.close()
                saved_path = self.bag_path
                self.bag = None
                self.bag_path = None
                rospy.loginfo(f"Stopped recording: {saved_path}")
                return TriggerResponse(
                    success=True,
                    message=f"Stopped recording: {saved_path}"
                )
            except Exception as e:
                return TriggerResponse(
                    success=False,
                    message=f"Failed to stop recording cleanly: {e}"
                )


    def handle_play_bag(self, req):

        bag_name = req.bag_name.strip()
        if not bag_name:
            return PlayBagResponse(False, "Empty bag name")

        if self.play_proc is not None and self.play_proc.poll() is None:
            return PlayBagResponse(False, "A bag is already playing")

        if not bag_name.endswith(".bag"):
            bag_name += ".bag"

        bag_path = bag_name
        if not os.path.isabs(bag_path):
            bag_path = os.path.join(self.output_dir, bag_name)

        if not os.path.exists(bag_path):
            return PlayBagResponse(False, f"Bag not found: {bag_path}")

        cmd = [
            "rosbag", "play",
            "--clock",
            bag_path
        ]

        try:
            self.play_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            rospy.loginfo("Started playback: %s", bag_path)
            return PlayBagResponse(True, f"started playback: {bag_path}")
        except Exception as e:
            return PlayBagResponse(False, f"failed to start playback: {e}")


    def rec_callback(self, msg, topic):
        with self.bag_lock:
            if not self.is_recording or self.bag is None:
                return
            stamp = msg.header.stamp if hasattr(msg, "header") and msg.header.stamp != rospy.Time() else rospy.Time.now()
            self.bag.write(topic, msg, stamp)


    def shutdown(self):
        with self.bag_lock:
            if self.bag is not None:
                try:
                    self.bag.close()
                finally:
                    self.bag = None
                    self.bag_path = None
                    self.is_recording = False

            if self.play_proc is not None and self.play_proc.poll() is None:
                self.play_proc.terminate()
                try:
                    self.play_proc.wait(timeout=2)
                except Exception:
                    self.play_proc.kill()
            self.play_proc = None


if __name__ == "__main__":
    try:
        rospy.init_node("astrobee_recorder")
        node = AstrobeeRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
