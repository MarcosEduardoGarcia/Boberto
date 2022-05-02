#!/usr/bin/env python 
import rospy 
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2  as cv
from cv_bridge import CvBridge, CvBridgeError

class colourFilterClass():
    '''
        This class will suscribe to a topic with an Image and
        process the same image to find the colour Red and Green
    '''
    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################ 
        rate = 100
        self.vel_msg = Twist()
        ###******* INIT PUBLISHERS *******### 
        self.pub_image = rospy.Publisher('colour_filter', Image, queue_size=1)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        ###******* INIT SUSCRIBERS *******### 
        self.bridge = CvBridge()
        rospy.Subscriber('video_source/raw',Image,self.image_cb)
        #********** INIT NODE **********### 
        r = rospy.Rate(rate) #1Hz 
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            r.sleep()
    def image_cb(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hue_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        result = cv_image.copy()
        # lower boundary RED color mask
        lower_red1 = np.array([0, 128, 100])
        upper_red1 = np.array([4, 255, 255])
        # upper boundary RED color range values
        lower_red2 = np.array([174, 128, 100])
        upper_red2 = np.array([180, 255, 255])

        lower_mask_red = cv.inRange(hue_image, lower_red1, upper_red1)
        upper_mask_red = cv.inRange(hue_image, lower_red2, upper_red2)
        full_mask_red = lower_mask_red + upper_mask_red

        # lower boundary GREEN color mask
        lower_green1 = np.array([60, 128, 100])
        upper_green1 = np.array([73, 255, 255])
        main_mask_green = cv.inRange(hue_image, lower_green1, upper_green1)
        full_mask_green = main_mask_green


        #result_Red = cv.bitwise_and(result,result, mask=full_mask_red)
        #result_Green = cv.bitwise_and(result,result, mask=full_mask_green)

        ret,thresh_green = cv.threshold(full_mask_green, 200, 255, cv.THRESH_BINARY_INV)
        ret,thresh_red = cv.threshold(full_mask_red, 200, 255, cv.THRESH_BINARY_INV)


        # Simple blob detector
        params = cv.SimpleBlobDetector_Params()
        # Filter by area
        params.filterByArea = False
        #params.minArea = 0.1
        # Filter by Cicularity
        params.filterByCircularity = True
        params.minCircularity = 0.74
        # #Filter by Convexity
        params.filterByConvexity = False
        # # Filter by Inertia
        params.filterByInertia = False

        detector = cv.SimpleBlobDetector_create(params)

        keypoints_green = detector.detect(thresh_green)
        keypoints_red = detector.detect(thresh_red)

        Green_Light = False
        Red_Light = False

        for point in keypoints_green:
            if point.size > 20:
                Green_Light = True
        for point in keypoints_red:
            if point.size > 20:
                Red_Light = True
        
        # Decition Making
        if Green_Light and not Red_Light:
            print("GO")
            self.vel_msg.linear.x = 0.2
        else:
            print("STOP")
            self.vel_msg.linear.x = 0.0

        self.pub_vel.publish(self.vel_msg)

        #blank = np.zeros((1,1))
        #im_with_keypoints = cv.drawKeypoints(thresh1, keypoints, blank, (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #self.pub_image.publish(self.bridge.cv2_to_imgmsg(im_with_keypoints, "bgr8"))
    def go_stop(self):
        # Detener
        stop = Twist()
        self.pub_vel.publish(stop)
    def cleanup(self):
        self.go_stop()
        print("\n-----FINALIZADO-----\n")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("colour_filter", anonymous=True) 
    try:
        colourFilterClass()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')