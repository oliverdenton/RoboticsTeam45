#!/usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

#import some other python modules
import numpy as np
import math

class colour_search(object):

    def __init__(self):
        rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0
        #self.sub_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)

        self.m00 = 0
        self.m00_min = 100000

        self.turn = False
        self.start_color = ""
        self.start_color_lower = []
        self.start_color_upper = []
        self.mask = None
        self.hsv_img = None
        self.find_target = False
        self.lower = None
        self.upper = None
        self.mask1 = None
        self.stop_at_target = False

        self.FRONT = 0
        self.LEFT = 0
        self.RIGHT = 0

        self.CMD_PUB = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        #self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.command = Twist()
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0

        self.near_wall = 0  # start with 0, when we get to a wall, change to 1
        self.distance = 0.4

        self.distance1 = 0.7
        self.color_forwards1 = None

        self.start_x = None
        self.start_y = None
        self.dist = None

        self.hsv_values = {
            "Red":    ([-2, 240, 120], [5, 255, 255]),
            "Blue":   ([115, 224, 100],   [130, 255, 255]),
            "Green":   ([35, 80, 100], [70, 255, 255]),
            "Turquoise":   ([75, 150, 100], [100, 255, 255]),
            "Yellow": ([28, 180, 100], [32, 255, 255]),
            "Purple":   ([145, 100, 100], [150, 255, 255])

        }


    def distance_calc(self):
        #self.dist = math.sqrt(self.robot_odom.posx - self.start_x)**2 + (self.robot_odom.posy- self.start_y)**2)
        self.dist = np.linalg.norm((self.robot_odom.posx-self.start_x) - (self.robot_odom.posy-self.start_y))
        #print(self.dist)
        st_bec = True
        if self.dist > 1.7:
            st_bec = False
        else:
            st_bec = True

        return st_bec

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def robot_forward(self,speed,time):
        self.robot_controller.set_move_cmd(speed,0)
        self.robot_controller.publish()
        rospy.sleep(time)
        self.robot_controller.stop()

    def robot_rotate(self,speed,time):
        rospy.sleep(1)
        self.robot_controller.set_move_cmd(0.0,speed)
        self.robot_controller.publish()
        rospy.sleep(time)
        self.robot_controller.stop()

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[500:1080, crop_x:crop_x+crop_width] #crop_x:crop_x+crop_width
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.hsv_img = hsv_img

        if np.array(self.start_color_lower).size and np.array(self.start_color_upper).size :
            self.mask = cv2.inRange(hsv_img, self.start_color_lower, self.start_color_upper)

        m = cv2.moments(self.mask)

        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)
        self.cz = m['m01'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), int(self.cz)), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def scan_callback(self, scan_data):
        self.FRONT = min(min(scan_data.ranges[0:5]), min(scan_data.ranges[354:359]))
        self.RIGHT = min(scan_data.ranges[300:345])
        self.LEFT = min(scan_data.ranges[15:60])

    def move_around(self):
        while not (self.m00 > self.m00_min and self.distance_calc() == False):
            while(self.near_wall == 0 and not rospy.is_shutdown()) :
                print("Moving towards a wall.")
                if(self.FRONT > self.distance and self.RIGHT > self.distance and self.LEFT > self.distance):  # Nothing there, go straight
                    self.command.angular.z = 0
                    self.command.linear.x = 0.20
                elif(self.RIGHT < self.distance):
                    self.near_wall = 1

                self.CMD_PUB.publish(self.command)

            else:   # left wall detected
                if(self.FRONT > self.distance *1.1):
                    if(self.RIGHT < (self.distance * 0.75)):
                        print(
                            "Range: {:.2f}m - Too close. Backing up.".format(self.RIGHT))
                        self.command.angular.z = 0.75 #1.2
                        self.command.linear.x = 0.17
                    elif(self.RIGHT > (self.distance )): #0.75
                        print(
                            "Range: {:.2f}m - Wall-following; turn left.".format(self.RIGHT))
                        self.command.angular.z = -0.75 #0.8
                        self.command.linear.x = 0.17 #0.22
                    else:
                        print(
                            "Range: {:.2f}m - Wall-following; turn right.".format(self.RIGHT))
                        self.command.angular.z = 0.6
                        self.command.linear.x = 0.17

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
        print("Move around end")

    def move_around1(self):
        while not (self.m00 > self.m00_min and self.color_forwards1 == self.start_color and self.FRONT > self.distance and self.RIGHT > self.distance and self.LEFT > self.distance ):
            while(self.near_wall == 0 and not rospy.is_shutdown()) :
                #print("Moving towards a wall.")
                if(self.FRONT > self.distance1 and self.RIGHT > self.distance1 and self.LEFT > self.distance1):  # Nothing there, go straight
                    self.command.angular.z = 0
                    self.command.linear.x = 0.1
                elif(self.RIGHT < self.distance1):
                    self.near_wall = 1

                self.CMD_PUB.publish(self.command)

            else:   # left wall detected
                if(self.FRONT > self.distance1 * 1.1):
                    if(self.RIGHT < (self.distance1 * 0.5)):
                        #print(
                        #    "Range: {:.2f}m - Too close. Backing up.".format(self.RIGHT))
                        self.command.angular.z = 0.75 #1.2
                        self.command.linear.x = 0.15
                    elif(self.RIGHT > (self.distance1 )): #0.75
                        #print(
                        #    "Range: {:.2f}m - Wall-following; turn left.".format(self.RIGHT))
                        self.command.angular.z = -0.75 #0.8
                        self.command.linear.x = 0.15 #0.22
                    else:
                        #print(
                        #    "Range: {:.2f}m - Wall-following; turn right.".format(self.RIGHT))
                        self.command.angular.z = 0.6
                        self.command.linear.x = 0.15

                else:  # 5
                    #print("Front obstacle detected. Turning away.")
                    self.command.angular.z = 1.0
                    self.command.linear.x = 0.0
                    self.CMD_PUB.publish(self.command)
                    while(self.FRONT < 0.3 and not self.ctrl_c):
                        self.CMD_PUB.publish(self.command)

                # publish command
                self.CMD_PUB.publish(self.command)
            # wait for the loop
            self.color_forwards1 = self.get_beacon_color()
            self.rate.sleep()
        self.command.angular.z = 0.0
        self.command.linear.x = 0.0
        self.CMD_PUB.publish(self.command)

    def get_beacon_color(self):
        for hsv_color, (lower_hsv, upper_hsv) in self.hsv_values.items():
            lower_bound = np.array(lower_hsv)
            upper_bound = np.array(upper_hsv)
            mask = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            if mask.any():
                return hsv_color


    def land_beacon(self):
        if self.stop_at_target == False:
            if self.cy >= 560-100 and self.cy <= 560+100 :
                color_forwards = self.get_beacon_color()
                self.robot_controller.set_move_cmd(0.1, 0)
                print("Moving ahead")
                if self.FRONT < 0.6 and color_forwards == self.start_color:
                        self.robot_forward(0.1,1.5)
                        print("BEACONING COMPLETE: The robot has now stopped.")
                        self.stop_at_target = True
                        self.robot_controller.stop()
                else:
                    self.move_around1()
            elif 0 < self.cy and self.cy <= 560-100 : #and self.front > 0.5 and self.right > 0.5 and self.left > 0.5:
                self.robot_controller.set_move_cmd(0.1, 0.1)
                self.robot_controller.publish()
                print("Adjusting left")
            elif self.cy > 560+100: #and self.front > 0.5 and self.right > 0.5 and self.left > 0.5:
                self.robot_controller.set_move_cmd(0.1, -0.1)
                self.robot_controller.publish()
                print("Adjusting Right")
            else:
                self.move_around1()
                #print("Go forward")
                #self.robot_controller.set_move_cmd(0.1, 0)
        else:
            self.robot_controller.stop()

    def main(self):
        while not self.ctrl_c:
            if self.turn == False:
                # Get the current robot odometry:
                self.start_x = self.robot_odom.posx
                self.start_y = self.robot_odom.posy
                #print(self.start_x , self.start_y)

                self.robot_rotate(-0.9,1.8)

                self.lower = np.array([0, 185, 120])
                self.upper = np.array([10, 255, 255])
                self.mask1 = cv2.inRange(self.hsv_img, self.lower, self.upper)
                if self.mask1.any():
                    self.start_color="Red"
                    self.start_color_lower = self.lower
                    self.start_color_upper = self.upper
                    print("SEARCH INITIATED: The target colour is {}".format (self.start_color))

                self.lower = np.array([115, 224, 100])
                self.upper = np.array([130, 255, 255])
                self.mask1 = cv2.inRange(self.hsv_img, self.lower, self.upper)
                if self.mask1.any():
                    self.start_color="Blue"
                    self.start_color_lower = self.lower
                    self.start_color_upper = self.upper
                    print("SEARCH INITIATED: The target colour is {}".format (self.start_color))

                self.lower = np.array([35, 80, 100])
                self.upper = np.array([70, 255, 255])
                self.mask1 = cv2.inRange(self.hsv_img, self.lower, self.upper)
                if self.mask1.any():
                    self.start_color="Green"
                    self.start_color_lower = self.lower
                    self.start_color_upper = self.upper
                    print("SEARCH INITIATED: The target colour is {}".format (self.start_color))

                self.lower = np.array([75, 150, 100])
                self.upper = np.array([100, 255, 255])
                self.mask1 = cv2.inRange(self.hsv_img, self.lower, self.upper)
                if self.mask1.any():
                    self.start_color="Turquoise"
                    self.start_color_lower = self.lower
                    self.start_color_upper = self.upper
                    print("SEARCH INITIATED: The target colour is {}".format (self.start_color))

                self.lower = np.array([28, 180, 100])
                self.upper = np.array([32, 255, 255])
                self.mask1 = cv2.inRange(self.hsv_img, self.lower, self.upper)
                if self.mask1.any():
                    self.start_color="Yellow"
                    self.start_color_lower = self.lower
                    self.start_color_upper = self.upper
                    print("SEARCH INITIATED: The target colour is {}".format (self.start_color))

                self.lower = np.array([145, 100, 100])
                self.upper = np.array([150, 255, 255])
                self.mask1 = cv2.inRange(self.hsv_img, self.lower, self.upper)
                if self.mask1.any():
                    self.start_color="Purple"
                    self.start_color_lower = self.lower
                    self.start_color_upper = self.upper
                    print("SEARCH INITIATED: The target colour is {}".format (self.start_color))

                self.robot_rotate(0.9,1.8)
                self.robot_controller.stop()
                self.turn = True

            else:
                self.distance_calc()
                if self.m00 > self.m00_min and self.find_target == False and self.distance_calc() == False:
                    self.move_rate = 'stop'
                    print("BEACON DETECTED: Beaconing initiated.")
                    self.find_target = True

                elif self.find_target == True:
                    self.move_rate = 'found_beacon'
                elif self.stop_at_target == True:
                    break
                else:
                    self.move_rate = 'fast'


                if self.move_rate == 'fast':
                    self.move_around()
                #elif self.move_rate == 'slow':
                #    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                elif self.move_rate == 'stop':
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                elif self.move_rate == 'found_beacon':
                    if self.stop_at_target == False :
                        self.move_around1()
                    self.land_beacon()
                else:
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

                self.robot_controller.publish()
                self.rate.sleep()

if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass