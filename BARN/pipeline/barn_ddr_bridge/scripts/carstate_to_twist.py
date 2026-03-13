#!/usr/bin/env python3
"""
Subscribes to DDR-opt MPC output (CarState) and publishes geometry_msgs/Twist for Jackal.
BARN/Jackal expects cmd_vel (Twist); DDR-opt publishes CarState with v and omega.
"""
import rospy
from geometry_msgs.msg import Twist
from carstatemsgs.msg import CarState


def main():
    rospy.init_node("carstate_to_twist", anonymous=False)
    cmd_sub_topic = rospy.get_param("~carstate_topic", "/ddr_cmd")
    twist_pub_topic = rospy.get_param("~twist_topic", "/cmd_vel")
    pub = rospy.Publisher(twist_pub_topic, Twist, queue_size=1)

    def callback(msg):
        t = Twist()
        t.linear.x = msg.v
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = msg.omega
        pub.publish(t)

    rospy.Subscriber(cmd_sub_topic, CarState, callback, queue_size=1)
    rospy.loginfo("carstate_to_twist: %s -> %s", cmd_sub_topic, twist_pub_topic)
    rospy.spin()


if __name__ == "__main__":
    main()
