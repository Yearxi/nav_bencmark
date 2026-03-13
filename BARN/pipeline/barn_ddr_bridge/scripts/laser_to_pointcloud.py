#!/usr/bin/env python3
"""
Converts BARN Gazebo LaserScan (/front/scan) to PointCloud2 for DDR-opt SDF map.
Publishes in the same frame as the scan (e.g. front_laser); SDF map will transform to world via tf.
"""
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

pub = None
lp = None


def callback(scan_msg):
    global pub, lp
    try:
        cloud = lp.projectLaser(scan_msg)
        pub.publish(cloud)
    except Exception as e:
        rospy.logerr_throttle(1.0, "laser_to_pointcloud error: %s", e)


def main():
    global pub, lp
    rospy.init_node("laser_to_pointcloud", anonymous=False)
    scan_topic = rospy.get_param("~scan_topic", "/front/scan")
    pc_topic = rospy.get_param("~pointcloud_topic", "/laser_simulator/local_pointcloud")
    lp = LaserProjection()
    pub = rospy.Publisher(pc_topic, PointCloud2, queue_size=1)
    rospy.Subscriber(scan_topic, LaserScan, callback, queue_size=1)
    rospy.loginfo("laser_to_pointcloud: %s -> %s", scan_topic, pc_topic)
    rospy.spin()


if __name__ == "__main__":
    main()
