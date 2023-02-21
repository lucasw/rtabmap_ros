#!/usr/bin/env python
# Visualize OdomInfo

import message_filters
import rospy
from rtabmap_ros.msg import OdomInfo
from sensor_msgs.msg import Image


def visualize_odom_info(image: Image, last_odom_info: OdomInfo, cur_odom_info: OdomInfo):
    if last_odom_info is None:
        return None
    rospy.loginfo(f"sync {(cur_odom_info.header.stamp - image.header.stamp).to_sec()}")


class OdomInfoViz:
    def __init__(self):
        self.last_odom_info = None

        image_sub = message_filters.Subscriber("image", Image)
        odom_info_sub = message_filters.Subscriber("odom_info", OdomInfo)
        # TODO(lucasw) the odom info is unsynchronized by a very small amount, which is either because rtabmap is introducing
        # a small offset or the python time conversion is faulty https://github.com/ros/geometry2/issues/543
        self.sync_sub = message_filters.ApproximateTimeSynchronizer([image_sub, odom_info_sub], queue_size=200, slop=0.001)
        self.sync_sub.registerCallback(self.sync_callback)

    def sync_callback(self, image, odom_info):
        visualize_odom_info(image, self.last_odom_info, odom_info)
        self.last_odom_info = odom_info

if __name__ == "__main__":
    rospy.init_node("odom_info_viz")
    node = OdomInfoViz()
    rospy.spin()
