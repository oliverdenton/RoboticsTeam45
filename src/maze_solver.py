#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_tb3 import MoveTB3

class maze_solver(object):
    def __init__(self):
        self.FRONT = 0
        self.LEFT = 0
        self.RIGHT = 0


        rospy.init_node('maze_navigation')
        self.CMD_PUB = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.robot_controller = MoveTB3()


        self.command = Twist()
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0

        self.rate = rospy.Rate(10)
        time.sleep(1)  # wait for node to initialize

        self.near_wall = 0  # start with 0, when we get to a wall, change to 1
        self.distance = 0.4

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def scan_callback(self,msg):
        self.FRONT = min(min(msg.ranges[0:5]), min(msg.ranges[354:359]))
        self.RIGHT = min(msg.ranges[300:345])
        self.LEFT = min(msg.ranges[15:60])

    def shutdownhook(self):
        print("Shutting down")
        self.robot_controller.stop()
        self.ctrl_c = True

    def main(self):
        end_time = time.time() + 150
        while time.time() < end_time:
            while(self.near_wall == 0 and not not rospy.is_shutdown()) :
                print("Moving towards a wall.")
                if(self.FRONT > self.distance and self.RIGHT > self.distance and self.LEFT > self.distance):  # Nothing there, go straight
                    self.command.angular.z = 0
                    self.command.linear.x = 0.20
                elif(self.RIGHT < self.distance):
                    self.near_wall = 1

                self.CMD_PUB.publish(self.command)

            else:   # left wall detected
                if(self.FRONT > self.distance * 1.1):
                    if(self.RIGHT < (self.distance * 0.75)):
                        print(
                            "Range: {:.2f}m - Too close. Backing up.".format(self.RIGHT))
                        self.command.angular.z = 0.8 #1.2
                        self.command.linear.x = 0.21
                    elif(self.RIGHT > (self.distance )): #0.75
                        print(
                            "Range: {:.2f}m - Wall-following; turn left.".format(self.RIGHT))
                        self.command.angular.z = -0.8 #0.8
                        self.command.linear.x = 0.21 #0.22
                    else:
                        print(
                            "Range: {:.2f}m - Wall-following; turn right.".format(self.RIGHT))
                        self.command.angular.z = 0.6
                        self.command.linear.x = 0.21

                else:  # 5
                    print("Front obstacle detected. Turning away.")
                    self.command.angular.z = 1.0
                    self.command.linear.x = 0.0
                    self.CMD_PUB.publish(self.command)
                    while(self.FRONT < 0.3 and not self.ctrl_c):
                        self.CMD_PUB.publish(self.command)

                # publish command
                self.CMD_PUB.publish(self.command)
            # wait for the loop
            self.rate.sleep()
        self.command.angular.z = 0.0
        self.command.linear.x = 0.0
        self.CMD_PUB.publish(self.command)

if __name__ == '__main__':
    search_ob = maze_solver()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
