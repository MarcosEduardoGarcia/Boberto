#!/usr/bin/env python  
import rospy  
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2
#This class will receive a ROS image and transform it to opencv format  

class ShowImage():  

    def __init__(self): 
        rospy.on_shutdown(self.cleanup)  
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image
        rate = 10 
        #Read the image file 
        # Nota se puede utilizar un path absoluto, para abrir cualquier imagen
        # img = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_2/Course_images/test_image_1.jpg') 
        # Nota2: Tambien se puede utilizar una ruta relativa que funcionara dependiendo de donde se ejecute el programa 
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("camera/image_raw", Image, self.image_cb) 
        #********** INIT NODE **********###  
        r = rospy.Rate(rate) #10Hz 
        print("Node initialized " + str(rate)+ " hz") 
        while not rospy.is_shutdown():  
            if self.image_received: 
                print("Do something with the image")
                #Display the image in a window 
                cv2.imshow('image', self.cv_image) 
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
        cv2.imwrite('Final_image.jpg', self.cv_image) 
        cv2.destroyAllWindows()
        print("\n-----FINALIZADO-----\n")

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("cv_bridge_example", anonymous=True)  
    try:
        ShowImage()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY') 