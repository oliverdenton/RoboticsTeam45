#!/usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

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
        self.sub_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
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

        self.command = Twist()
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0

        self.near_wall = 0  # start with 0, when we get to a wall, change to 1
        self.distance = 0.5


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

        crop_img = cv_img[500:1080, crop_x:crop_x+crop_width]
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

        self.front = min(min(scan_data.ranges[0:5]), min(scan_data.ranges[354:359]))
        self.right = min(scan_data.ranges[260:350])
        self.left = min(scan_data.ranges[15:65])

    def move_around(self):
        while not (self.m00 > self.m00_min):
            while(self.near_wall == 0 and not rospy.is_shutdown()) :
                print("Moving towards a wall.")
                if(self.FRONT > self.distance and self.RIGHT > self.distance and self.LEFT > self.distance):  # Nothing there, go straight
                    self.command.angular.z = 0
                    self.command.linear.x = 0.20
                elif(self.RIGHT < self.distance):
                    self.near_wall = 1

                self.CMD_PUB.publish(self.command)

            else:   # left wall detected
                if(self.FRONT > self.distance):
                    if(self.RIGHT < (self.distance * 0.75)):
                        print(
                            "Range: {:.2f}m - Too close. Backing up.".format(self.RIGHT))
                        self.command.angular.z = 0.6 #1.2
                        self.command.linear.x = 0.19
                    elif(self.RIGHT > (self.distance )): #0.75
                        print(
                            "Range: {:.2f}m - Wall-following; turn left.".format(self.RIGHT))
                        self.command.angular.z = -0.6 #0.8
                        self.command.linear.x = 0.19 #0.22
                    else:
                        print(
                            "Range: {:.2f}m - Wall-following; turn right.".format(self.RIGHT))
                        self.command.angular.z = 0.6
                        self.command.linear.x = 0.19

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

    def move_around1(self):
        while not self.stop_at_target:
            while(self.near_wall == 0 and not rospy.is_shutdown()) :
                #print("Moving towards a wall.")
                if(self.FRONT > self.distance and self.RIGHT > self.distance and self.LEFT > self.distance):  # Nothing there, go straight
                    self.command.angular.z = 0
                    self.command.linear.x = 0.20
                elif(self.RIGHT < self.distance):
                    self.near_wall = 1

                self.CMD_PUB.publish(self.command)

            else:   # left wall detected
                if(self.FRONT > self.distance):
                    if(self.RIGHT < (self.distance * 0.75)):
                        #print(
                        #    "Range: {:.2f}m - Too close. Backing up.".format(self.RIGHT))
                        self.command.angular.z = 0.6 #1.2
                        self.command.linear.x = 0.19
                    elif(self.RIGHT > (self.distance )): #0.75
                        #print(
                        #    "Range: {:.2f}m - Wall-following; turn left.".format(self.RIGHT))
                        self.command.angular.z = -0.6 #0.8
                        self.command.linear.x = 0.19 #0.22
                    else:
                        print(
                            "Range: {:.2f}m - Wall-following; turn right.".format(self.RIGHT))
                        self.command.angular.z = 0.6
                        self.command.linear.x = 0.19

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
            self.rate.sleep()
        self.command.angular.z = 0.0
        self.command.linear.x = 0.0
        self.CMD_PUB.publish(self.command)


    def get_color(self):
        color_hsv = {
            "Red":    ([0, 185, 100], [10, 255, 255]),
            "Blue":   ([115, 224, 100],   [130, 255, 255]),
            "Green":   ([25, 150, 100], [70, 255, 255]),
            "Turquoise":   ([75, 150, 100], [100, 255, 255]),
            "Yellow": ([28, 180, 100], [32, 255, 255]),
            "Purple":   ([145, 185, 100], [150, 250, 255])
        }

        for color_name, (lower, upper) in color_hsv.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            if mask.any():
                return color_name
    
    def beacon(self):
        if self.stop_at_target == False:
            if self.cy >= 560-100 and self.cy <= 560+100 and self.cz < 350:
                color_forwards = self.get_color()
                self.robot_controller.set_move_cmd(0.1, 0)
                print("Moving forward")
                if self.front <= 0.24 or self.right <= 0.24 or self.left <= 0.24:
                    #self.move_around()
                    if self.right <= 0.24:
                        self.robot_controller.set_move_cmd(-0.3, 0.2)
                        self.robot_controller.publish()
                        try_left = False
                        
                    elif self.left <= 0.24:
                        self.robot_controller.set_move_cmd(-0.3, -0.2)
                        self.robot_controller.publish()
                        try_left = True
                        
                    else:
                        self.robot_controller.set_move_cmd(-0.3, 0)
                        self.robot_controller.publish()
                        
                    #self.move_around()    
                    self.robot_controller.publish()
                    print("Too close to wall!!")
                elif (self.front <= 0.3 or self.right <= 0.3 or self.left <= 0.3) and color_forwards == self.start_color:
                    #self.robot_controller.stop()
                    #self.move_rate = "stop"
                    #self.find_target = True

                    if self.front < 0.6:
                        print("BEACONING COMPLETE: The robot has now stopped.")
                        self.robot_controller.stop()
                        self.stop_at_target = True   
                    else:
                        self.move_around1()
            elif 0 < self.cy and self.cy <= 560-100 : #and self.front > 0.5 and self.right > 0.5 and self.left > 0.5:
                self.robot_controller.set_move_cmd(0.1, 0.25)
                self.robot_controller.publish()
                print("Adjust left")
            elif self.cy > 560+100: #and self.front > 0.5 and self.right > 0.5 and self.left > 0.5:
                self.robot_controller.set_move_cmd(0.1, -0.25)
                self.robot_controller.publish()
                print("Adjust Right")
            else:
                self.move_around1()
        else: 
            self.robot_controller.stop()

    def main(self):
        while not self.ctrl_c:
            if self.turn == False:
                self.robot_rotate(0.9,1.8)


                self.lower = np.array([0, 185, 100])
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

                self.lower = np.array([25, 150, 100])
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

                self.robot_rotate(-0.9,1.85)
                self.robot_controller.stop()
                self.turn = True  

            else:
                if self.m00 > self.m00_min and self.find_target == False :
                    # blob detected
                    #if self.cy >= 560-100 and self.cy <= 560+100:
                        #if self.move_rate == 'slow':
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
                    #write code about robot moving around the map
                    #self.robot_controller.set_move_cmd(0.0, 0.0) # just here to stop terminal from giving an error
                    self.move_around()
                elif self.move_rate == 'slow':
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                elif self.move_rate == 'stop':
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                elif self.move_rate == 'found_beacon':
                    #write code about what to do when beacon is found
                    self.robot_controller.set_move_cmd(0.0, 0.0) # just here to stop terminal from giving an error
                    #self.beacon()
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
