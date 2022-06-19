#!/usr/bin/env python3
import cv2
import rospy 
import numpy as np
from sensor_msgs.msg import Image 
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
import pandas as pd

class neural_net():
    '''
        This class will make the robot follow some points in the space
    '''
    threshold = 0.95
    model = tf.keras.models.load_model('Model/traffic_ultimate.h5')

    def grayscale(self,img):  
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        return img

    def equalize(self,img):
        img = cv2.equalizeHist(img)
        return img

    def preprocessing(self,img):
        img = self.grayscale(img)
        img = self.equalize(img)
        img = img/255
        return img

    def getClassname(self,classNo):
        if   classNo == 14: return 'Stop'
        elif classNo == 32: return 'End of all speed and passing limits'
        elif classNo == 33: return 'Turn right ahead'
        elif classNo == 35: return 'Ahead only'
        elif classNo == 11: return 'Stop'
        elif classNo == 40: return 'Stop'
        else: return "None important sign"

    def getClassNum(self,classNo):
        if classNo == 14: return 14
        elif classNo == 32: return 32
        elif classNo == 33: return 33
        elif classNo == 35: return 35
        elif classNo == 11: return 14
        elif classNo == 40: return 14
        else: return 0

    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        self.bridge_object = CvBridge() # create the cv_bridge object 
        self.image_received = 0 #Flag to indicate that we have already received an image
        ############ CONSTANTS ################ 
        rate = 15
        signal = Int16()
        model = tf.keras.models.load_model('Model/traffic_ultimate.h5')
        threshold = 0.8
        #############################################
        y_final = pd.read_csv('Data/labels.csv')
        Class = y_final["ClassId"].values
        ###******* INIT PUBLISHERS *******### 
        self.pub_signal = rospy.Publisher('signal_inst', Int16, queue_size=1)
        ###******* INIT SUSCRIBERS *******##
        rospy.Subscriber("crop_image", Image, self.image_cb) 
        #********** INIT NODE **********### 
        r = rospy.Rate(rate) #1Hz 
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            if self.image_received: 
                if self.cv_image is not None:
                    image = self.cv_image.copy()
                    if image.shape[0] ==  image.shape[1] :
                        print("No significant image Assertion: Size")
                        signal.data = 0
                        # print("+++++++++++++++++++++++")
                    else:
                        imagen_red = np.asarray(image)
                        imagen_red = cv2.resize(imagen_red,(32,32))
                        imagen_red = self.preprocessing(imagen_red)
                        imagen_red = imagen_red.reshape(1,32,32,1)
                        # Predictions
                        #Inicia Bobertos Brain
                        predictions = model.predict(imagen_red)
                        probability = np.amax(predictions)
                        print(probability)
                        if probability > threshold:
                            probability_pos = np.argmax(predictions)
                            result = Class[probability_pos]
                            text = self.getClassname(result)
                            print("The sign is: ", text, result)
                            signal.data = self.getClassNum(result)
                            
                        else:
                            signal.data = 0
                            print("No sign detected, Assertion: Low posibility")
                    print("==================================")
                    self.pub_signal.publish(signal.data)
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
    rospy.init_node("Neural_Network", anonymous=True) 
    try:
        neural_net()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')