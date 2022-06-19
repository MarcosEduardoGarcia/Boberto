#!/usr/bin/env python
import cv2
import rospy 
import numpy as np
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError

class crop_filter():
    '''
        This class will make the robot follow some points in the space
    '''
    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image
        ############ CONSTANTS ################ 
        rate = 20
        self.height = 300 
        self.width =  533
        ###******* INIT PUBLISHERS *******### 
        self.pub_image = rospy.Publisher('crop_image', Image, queue_size=1)
        ###******* INIT SUSCRIBERS *******### 
        rospy.Subscriber("video_source/raw", Image, self.image_cb) 
        #********** INIT NODE **********### 
        r = rospy.Rate(rate) #1Hz 
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            if self.image_received:  
                image = self.cv_image.copy()
                imagen_chida = image.copy()
                #I resized the image so it can be easier to work with 
                #image_reduce = cv2.resize(image,(self.width,self.height))
                stx = int(self.width*0.5)
                fnx = int(self.width*1)
                sty= int(self.height*0.0)
                fny = int(self.height*0.5)
                imCorte = image[sty:fny, stx:fnx]
                imGray = cv2.cvtColor(imCorte, cv2.COLOR_BGR2GRAY)
                imBlur = cv2.medianBlur(imGray, 1)
                imCanny = cv2.Canny(imBlur, 20, 20)
                kernel1 = np.ones((1,1),np.uint8)
                imClose1 = cv2.morphologyEx(imCanny,cv2.MORPH_CLOSE,kernel1)
                kernel2 = np.ones((1,1),np.uint8)
                imOpen = cv2.morphologyEx(imClose1,cv2.MORPH_OPEN,kernel2)
                # kernel3 = np.ones((3,3),np.uint8)
                # imClose2 = cv2.morphologyEx(imOpen,cv2.MORPH_CLOSE,kernel3)
                # kernel4 = np.ones((1,1),np.uint8)
                # imDilate = cv2.dilate(imClose2, kernel4, iterations=1)

                # Simple blob detector
                params = cv2.SimpleBlobDetector_Params()
                # Filter by area
                params.filterByArea = True
                params.minArea = 900
                params.maxArea = 18000
                # Filter by Cicularity
                params.filterByCircularity = True
                params.minCircularity = 0.01
                params.maxCircularity = 1
                # #Filter by Convexity
                params.filterByConvexity = False
                # params.minConvexity = 0.130
                # params.maxConvexity = 0.900
                # # Filter by Inertia
                params.filterByInertia = False
                # params.minInertiaRatio = 0.24
                # params.maxInertiaRatio = 1

                params.minDistBetweenBlobs = 44
                detector = cv2.SimpleBlobDetector_create(params)

                keypoints = detector.detect(imOpen)
                if keypoints:
                    for point in keypoints:
                        paramy = 28
                        paramx = int(paramy*1.2)
                        y_start = int(point.pt[1] - paramy)
                        y_end = int(point.pt[1] + paramy)
                        x_start = int(point.pt[0]-paramx)
                        x_end = int(point.pt[0]+paramx)
                        if y_start >= 0 and x_end <= fnx:
                            imagen_chida = imCorte[y_start:y_end, x_start:x_end]
                            self.pub_image.publish(self.bridge_object.cv2_to_imgmsg(imagen_chida, "bgr8"))
                        else:
                            imagen_chafa = imagen_chida[0:89,0:89]
                            self.pub_image.publish(self.bridge_object.cv2_to_imgmsg(imagen_chafa, "bgr8"))        
                        break

                else:
                    imagen_chafa = imagen_chida[0:89,0:89]
                    self.pub_image.publish(self.bridge_object.cv2_to_imgmsg(imagen_chafa, "bgr8"))
                self.image_received = 0
            r.sleep()
    def image_cb(self, ros_image):  
        ## This function receives a ROS image and transforms it into opencv format
        if not self.image_received:   
            try:
                self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8") 
                
                self.image_received = 1 #Turn the flag on 
            except CvBridgeError as e: 
                print(e) 
    def cleanup(self):
        #Save the image "img" in the current path  
        #cv2.imwrite('Sign1.jpg', imagen_chida) 
        print("\n-----FINALIZADO-----\n")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("crop_node", anonymous=True) 
    try:
        crop_filter()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')