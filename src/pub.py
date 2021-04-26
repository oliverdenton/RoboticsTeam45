#!/usr/bin/env python
# A simple ROS publisher node in Python

import rospy
from std_msgs.msg import String

class Publisher:

    def __init__(self):
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('publisher_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("publisher node is active...")

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        print("stopping publisher node at: {}".format(rospy.get_time()))

    def main_loop(self):
        while not self.ctrl_c:
            publisher_message = "rospy time is: {}".format(rospy.get_time())
            self.pub.publish(publisher_message)
            self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
