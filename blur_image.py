#!/usr/bin/env python 
import rospy 
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter():
    '''
        This class will project from a topic an image
    '''
    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################ 
        rate = 100
        ###******* INIT PUBLISHERS *******### 
        self.pub_image = rospy.Publisher('image_blur', Image, queue_size=1)
        ###******* INIT SUSCRIBERS *******### 
        self.bridge = CvBridge()
        rospy.Subscriber('video_source/raw',Image,self.image_cb)
        #********** INIT NODE **********### 
        r = rospy.Rate(rate) #1Hz 
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            r.sleep()
    def image_cb(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gaussian_image = cv2.GaussianBlur(self.cv_image, (9,9), cv2.BORDER_DEFAULT)
        resize_image = cv2.resize(gaussian_image, (1280,720), interpolation = cv2.INTER_AREA)
        gray_image = cv2.cvtColor(resize_image, cv2.COLOR_BGR2GRAY)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(gray_image, "mono8"))
    def cleanup(self):
        print("\n-----FINALIZADO-----\n")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("square_node", anonymous=True) 
    try:
        ImageConverter()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')