#!/usr/bin/env python
# A simple ROS subscriber node in Python

import rospy
from std_msgs.msg import String

class Subscriber:

    def callback(self, data):
        print("Subscriber obtained the following message: \"{}\"".format(data.data))

    def __init__(self):
        rospy.init_node('subscriber_node', anonymous=True)
        self.sub = rospy.Subscriber("natter", String, self.callback)
        rospy.loginfo("subscriber node is active...")

    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()
