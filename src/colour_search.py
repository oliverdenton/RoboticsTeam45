#!/usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3

#import some other python modules
import numpy as np

class colour_search(object):

    def __init__(self):
        rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0

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

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.hsv_img = hsv_img

        if np.array(self.start_color_lower).size and np.array(self.start_color_upper).size :
            self.mask = cv2.inRange(hsv_img, self.start_color_lower, self.start_color_upper)

        m = cv2.moments(self.mask)

        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 0), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            if self.turn == False:
                self.robot_rotate(0.2,8.7)


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


                self.robot_rotate(-0.2,8.7)
                self.robot_forward(0.2,5)
                self.robot_rotate(0.2,9.8)
                self.turn = True

            else:
                if self.m00 > self.m00_min:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'

                            print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                            self.find_target = True
                            self.robot_controller.stop()
                            rospy.sleep(2)
                            break
                    else:
                        self.move_rate = 'slow'
                elif self.find_target == True:
                    self.robot_controller.stop()
                    break
                else:
                    self.move_rate = 'fast'

                if self.find_target == False:
                    if self.move_rate == 'fast':
                        print("MOVING FAST: I can't see anything at the moment (blob size = {:.0f}), scanning the area...".format(self.m00))
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                    elif self.move_rate == 'slow':
                        print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                    elif self.move_rate == 'stop':
                        print("STOPPED: The blob of colour is now dead-ahead at y-position {:.0f} pixels... Counting down: {}".format(self.cy, self.stop_counter))
                        self.robot_controller.set_move_cmd(0.0, 0.0)
                    else:
                        print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

                    self.robot_controller.publish()
                    self.rate.sleep()

if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
