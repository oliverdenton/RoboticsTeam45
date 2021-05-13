#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Closest values in each region
FRONT = 0
LEFT = 0
RIGHT = 0

# Publisher node
CMD_PUB = None


def scan_callback(msg):
    global FRONT, LEFT, RIGHT

    FRONT = min(min(msg.ranges[0:5]), min(msg.ranges[354:359]))
    RIGHT = min(msg.ranges[300:345])
    LEFT = min(msg.ranges[15:60])


def main():
    global CMD_PUB, FRONT, LEFT, RIGHT

    CMD_PUB = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
    rospy.init_node('maze_navigation')

    command = Twist()
    command.linear.x = 0.0
    command.angular.z = 0.0

    rate = rospy.Rate(10)
    time.sleep(1)  # wait for node to initialize

    near_wall = 0  # start with 0, when we get to a wall, change to 1
    distance = 0.4

    # print("Turning...")
    # command.angular.z = 0
    # command.linear.x = 0
    # CMD_PUB.publish(command)
    # time.sleep(2)

    while not rospy.is_shutdown():
        while(near_wall == 0 and not rospy.is_shutdown()):
            print("Moving towards a wall.")
            if(FRONT > distance and RIGHT > distance and LEFT > distance):  # Nothing there, go straight
                command.angular.z = 0
                command.linear.x = 0.20
            elif(RIGHT < distance):
                near_wall = 1
            # else:
            #     command.angular.z = 0.0 #25
            #     command.linear.x = 0.0

            CMD_PUB.publish(command)

        else:   # left wall detected
            if(FRONT > distance):
                if(RIGHT < (distance * 0.75)):
                    print(
                        "Range: {:.2f}m - Too close. Backing up.".format(RIGHT))
                    command.angular.z = 0.8 #1.2
                    command.linear.x = 0.22
                elif(RIGHT > (distance )): #0.75
                    print(
                        "Range: {:.2f}m - Wall-following; turn left.".format(RIGHT))
                    command.angular.z = -0.8 #0.8
                    command.linear.x = 0.22 #0.22
                else:
                    print(
                        "Range: {:.2f}m - Wall-following; turn right.".format(RIGHT))
                    command.angular.z = 0.6
                    command.linear.x = 0.22

            else:  # 5
                print("Front obstacle detected. Turning away.")
                command.angular.z = 1.0
                command.linear.x = 0.0
                CMD_PUB.publish(command)
                while(FRONT < 0.3 and not rospy.is_shutdown()):
                    CMD_PUB.publish(command)

            # publish command
            CMD_PUB.publish(command)
        # wait for the loop
        rate.sleep()


if __name__ == '__main__':
    main()
