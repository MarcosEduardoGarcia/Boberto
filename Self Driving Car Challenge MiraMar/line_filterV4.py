#!/usr/bin/env python  
from fnmatch import fnmatch
import rospy 
import numpy as np
import cv2
from sensor_msgs.msg import Image 
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError



class line_filter3():
    '''
        This class will make the robot follow some points in the space
    '''


    def prepros(self,img):
        pass


    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################ 
        rate = 300
        self.image_received = 0 #Turn the flag on 
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.height = 360 
        self.width =  640
        memory_center = 0
        pos = Int16()
        ###******* INIT PUBLISHERS *******### 
        self.pub_image = rospy.Publisher('line_finder', Image, queue_size=1)
        self.pub_pos = rospy.Publisher('line_pos', Int16, queue_size=1)
        ###******* INIT SUSCRIBERS *******### 
        rospy.Subscriber("video_source/raw", Image, self.image_cb) 
        #********** INIT NODE **********### 
        r = rospy.Rate(rate) #1Hz 
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            if self.image_received:
                main_image = self.cv_image.copy()
                img_Gray = cv2.cvtColor(main_image , cv2.COLOR_BGR2GRAY)
                imgBlur = cv2.GaussianBlur(img_Gray , (9,9) , cv2.BORDER_DEFAULT)
                ret, imgThresh = cv2.threshold(imgBlur, 90, 255 , cv2.THRESH_BINARY_INV)
                
                stx = int(self.width*0.2)
                fnx = int(self.width*0.8)
                sty = int(self.height*0.9)
                fny = int(self.height-1*1)
                crop_image = imgThresh[sty:fny , stx:fnx]
                #print(crop_image.shape)
                conts, hierarchies = cv2.findContours(crop_image,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                if conts:
                    for c in conts:
                        M = cv2.moments(c)
                        if M["m00"] == 0: div = 1
                        else: div = M["m00"] 
                        cx = int(M["m10"]/div)
                        cy = int(M["m01"]/div)
                        pos.data = stx + cx
                        # print(cx)
                        # print(crop_image.shape)
                        # cv2.drawContours(main_image, [c], -1, (0, 255, 0), 2)
                        cv2.circle(crop_image, (cx, cy), 7, (0, 0, 0), -1)
                        break
                else:
                    # No line detected
                    pos.data = 1000
                print("Line Pos: " + str(pos.data))
                print("=======================================")
                
                self.pub_image.publish(self.bridge_object.cv2_to_imgmsg(crop_image, "mono8"))
                self.pub_pos.publish(pos)
                self.image_received = 0
            r.sleep()
    def image_cb(self, ros_image):  
        '''This function receives a ROS image and transforms it into opencv format'''
        if not self.image_received:   
            try:
                # We select bgr8 because it is the OpenCV encoding by default 
                self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8") 
                self.image_received = 1 #Turn the flag on 
            except CvBridgeError as e: print(e)

    def cleanup(self):
        print("\n-----FINALIZADO-----\n")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("line_filter3", anonymous=True) 
    try:
        line_filter3()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')