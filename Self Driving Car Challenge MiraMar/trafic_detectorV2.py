#!/usr/bin/env python  
from cv2 import circle
import rospy  
from sensor_msgs.msg import Image 
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np 
#This class will receive a ROS image and transform it to opencv format  

class drive_license():  

    def __init__(self): 
        rospy.on_shutdown(self.cleanup)  
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image
        rate = 10
        state = Bool()
        state.data = False
        self.height = 500 
        self.width =  500
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("video_source/raw", Image, self.image_cb) 
        ###******* INIT PUBLISHERS *******### 
        self.pub_image2 = rospy.Publisher('object_finder2', Image, queue_size=1)
        self.pub_state = rospy.Publisher('traffic_light', Bool, queue_size=1)
        #********** INIT NODE **********###  
        r = rospy.Rate(rate) #10Hz 
        print("Node initialized " + str(rate)+ " hz") 
        while not rospy.is_shutdown():
            if self.image_received:  
                image = self.cv_image.copy()
                #I resized the imares_gge so it can be easier to work with 
                image = cv2.resize(image,(self.width,self.height)) 
                #Once we read the image we need to change the color space to HSV 
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
                #here is where you define the range of the color youre looking for 
                #each value of the vector corresponds to the H,S & V values respectively 
                min_green = np.array([70,22,120]) 
                max_green = np.array([90,255,255])

                min_red1 = np.array([150,30,120]) 
                max_red1 = np.array([180,255,255])
                min_red2 = np.array([2,10,180]) 
                max_red2 = np.array([30,255,255])
                #This is the actual color detection  
                #Here we will create a mask that contains only the colors defined in your limits 
                #This mask has only one dimension, so its black and white
                mask_g = cv2.inRange(hsv, min_green, max_green) 
                mask_r1 = cv2.inRange(hsv, min_red1, max_red1) 
                mask_r2 = cv2.inRange(hsv, min_red2, max_red2) 
                mask_r = mask_r1 + mask_r2

                kernel = np.ones((7,7),np.uint8)
                kernel2 = np.ones((21,21),np.uint8)

                mask_r = cv2.morphologyEx(mask_r,cv2.MORPH_OPEN,kernel)
                mask_r = cv2.morphologyEx(mask_r,cv2.MORPH_CLOSE,kernel2)

                mask_g = cv2.morphologyEx(mask_g,cv2.MORPH_OPEN,kernel)
                mask_g = cv2.morphologyEx(mask_g,cv2.MORPH_CLOSE,kernel2)

                ret,thresh_red = cv2.threshold(mask_r, 127, 255, cv2.THRESH_BINARY_INV)
                ret,thresh_green = cv2.threshold(mask_g, 127, 255, cv2.THRESH_BINARY_INV)

                stx = int(self.width*0.61)
                fnx = int(self.width*1)
                sty= int(self.height*0)
                fny = int(self.height*1)

                start_point = (stx,sty)
                end_point = (fnx,fny)

                crop_green = cv2.rectangle(thresh_green,start_point,end_point,(255,255,255),-1)
                crop_red = cv2.rectangle(thresh_red,start_point,end_point,(255,255,255),-1)

                stx = int(self.width*0.0)
                fnx = int(self.width*1)
                sty= int(self.height*0.51)
                fny = int(self.height*1)

                start_point = (stx,sty)
                end_point = (fnx,fny)

                crop_green = cv2.rectangle(crop_green,start_point,end_point,(255,255,255),-1)
                crop_red = cv2.rectangle(crop_red,start_point,end_point,(255,255,255),-1)

                # Simple blob detector
                params = cv2.SimpleBlobDetector_Params()
                # Filter by area
                params.filterByArea = True
                params.minArea = 60
                # Filter by Cicularity
                params.filterByCircularity = False
                # #Filter by Convexity
                params.filterByConvexity = False
                # # Filter by Inertia
                params.filterByInertia = True
                params.minInertiaRatio = 0.095
                # params.minDistBetweenBlobs = 44
                detector = cv2.SimpleBlobDetector_create(params)

                green_point = detector.detect(crop_green)
                red_point = detector.detect(crop_red)

                Red_light = False
                Green_light = False
                
                if red_point is not None:
                    for circle in red_point:
                        # print("Red Size: " + str(circle.size))
                        if circle.size > 13:
                            Red_light = True
                            break
                if green_point is not None:
                    for circle in green_point:
                        # print("Green Size: " + str(circle.size))
                        if circle.size > 10:
                            Green_light = True
                            break
                
                if Red_light:
                    print("STOP")
                    # actual = False
                    state.data = False
                elif Green_light:
                    print("GO")
                    # actual = True
                    state.data = True
                else:
                    # actual = 2
                    print("UNDEFINED")

                self.pub_image2.publish(self.bridge_object.cv2_to_imgmsg(crop_red, "mono8"))
                # self.pub_image.publish(self.bridge_object.cv2_to_imgmsg(dilate_red, "mono8"))
                self.pub_state.publish(state)
                self.image_received = 0
            r.sleep() 
    def image_cb(self, ros_image):  
        ## This function receives a ROS image and transforms it into opencv format
        if not self.image_received:   
            try:
                #print("Recibi imagen")
                # We select bgr8 because it is the OpenCV encoding by default 
                self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8") 
                self.image_received = 1 #Turn the flag on 
            except CvBridgeError as e: 
                print(e) 
    def cleanup(self):
        final_state = Bool()
        final_state.data = False
        self.pub_state.publish(final_state)
        print("\n-----FINALIZADO-----\n")

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("trafic_view", anonymous=True)  
    try:
        drive_license()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY') 