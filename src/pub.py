#!/usr/bin/env python
# A simple ROS publisher node in Python

import rospy
from std_msgs.msg import String

class Publisher:

    def __init__(self):
        self.startup = True
        #change params
        self.pub = rospy.Publisher('cmd_vel', String, queue_size=10)
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

    def callback_function(self,odom_data):
        # obtain the orientation co-ords:
        x = odom_data.pose.pose.orientation.x
        y = odom_data.pose.pose.orientation.y
        z = odom_data.pose.pose.orientation.z
        w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        self.x = odom_data.pose.pose.position.x
        self.y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch yaw (theta_x, theta_y, theta_z):
        (roll, pitch, self.yaw) = euler_from_quaternion([x, y, z, w],'sxyz')

        # set the initial robot pose if this node has just been launched
        if self.startup:
            # don't initialise again:
            self.startup = False

            # set the robot starting position:
            self.init_x = self.x
            self.init_y = self.y
            self.init_yaw = self.yaw

if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
