#!/usr/bin/env python  
import rospy  
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2
import numpy as np 
#This class will receive a ROS image and transform it to opencv format  

class ShowImage():  

    def __init__(self): 
        rospy.on_shutdown(self.cleanup)  
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image
        rate = 10 
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("camera/image_raw", Image, self.image_cb) 
        #********** INIT NODE **********###  
        r = rospy.Rate(rate) #10Hz 
        print("Node initialized " + str(rate)+ " hz") 
        while not rospy.is_shutdown():
            if self.image_received:  
                image = self.cv_image.copy()
                #I resized the image so it can be easier to work with 
                image = cv2.resize(image,(300,300)) 
                #Once we read the image we need to change the color space to HSV 
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
                #Hsv limits are defined 
                #here is where you define the range of the color youre looking for 
                #each value of the vector corresponds to the H,S & V values respectively 
                min_green = np.array([50,153,77]) 
                max_green = np.array([60,255,255]) 
                min_red1 = np.array([0,153,77]) 
                max_red1 = np.array([5,255,255])
                min_red2 = np.array([173,153,77]) 
                max_red2 = np.array([180,255,255])
                min_blue = np.array([110,153,77]) 
                max_blue = np.array([120,255,255])
                min_yell = np.array([30,153,77]) 
                max_yell = np.array([30,255,255])
                #This is the actual color detection  
                #Here we will create a mask that contains only the colors defined in your limits 
                #This mask has only one dimension, so its black and white
                mask_g = cv2.inRange(hsv, min_green, max_green) 
                mask_r1 = cv2.inRange(hsv, min_red1, max_red1) 
                mask_r2 = cv2.inRange(hsv, min_red2, max_red2)
                full_mask_r = mask_r1 + mask_r2
                mask_b = cv2.inRange(hsv, min_blue, max_blue) 
                mask_y = cv2.inRange(hsv, min_yell, max_yell) 
                #We use the mask with the original image to get the colored post-processed image 
                res_b = cv2.bitwise_and(image, image, mask= mask_b) 
                res_g = cv2.bitwise_and(image,image, mask= mask_g)
                res_r = cv2.bitwise_and(image,image, mask= full_mask_r)
                res_y = cv2.bitwise_and(image,image, mask= mask_y)
                cv2.imshow('Image',image) 
                cv2.imshow('Image Red',res_r) 
                cv2.imshow('Image Blue',res_b)
                cv2.imshow('Image Green',res_g)
                cv2.imshow('Image Yellow',res_y)  
                cv2.waitKey(1)
                self.image_received = 0
            r.sleep() 
    def image_cb(self, ros_image):  
        ## This function receives a ROS image and transforms it into opencv format
        if not self.image_received:   
            try: 
                print("received ROS image, I will convert it to opencv") 
                # We select bgr8 because it is the OpenCV encoding by default 
                self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8") 
                self.image_received = 1 #Turn the flag on 
            except CvBridgeError as e: 
                print(e) 
    def cleanup(self):
        #Save the image "img" in the current path  
        cv2.destroyAllWindows()
        print("\n-----FINALIZADO-----\n")

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("cv_bridge_example", anonymous=True)  
    try:
        ShowImage()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY') 