#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2  as cv
from cv_bridge import CvBridge, CvBridgeError

class objectFollower():
    '''
        This class will suscribe to a topic with an Image and
        process the same image to find the colour Red and Green
    '''
    def __init__(self):
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup)
        ############ CONSTANTS ################
        rate = 10
        vel_msg = Twist()
        self.image_received = 0
        ###******* INIT PUBLISHERS *******###
        self.pub_image = rospy.Publisher('object_finder', Image, queue_size=1)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        ###******* INIT SUSCRIBERS *******###
        self.bridge = CvBridge()
        rospy.Subscriber('video_source/raw',Image,self.image_cb)
        #********** INIT NODE **********###
        r = rospy.Rate(rate) #1Hz
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            if self.image_received:
                main_width = self.cv_width
                center_width = main_width//2
                main_image = self.cv_image.copy()
                hue_image = cv.cvtColor(main_image, cv.COLOR_BGR2HSV)
                result = main_image.copy()

                # lower boundary YELLOW color mask
                lower_yellow = np.array([24, 97, 77])
                upper_yellow = np.array([38, 255, 255])
                main_mask_yellow = cv.inRange(hue_image, lower_yellow, upper_yellow)

                result_yellow = cv.bitwise_and(result,result, mask=main_mask_yellow)
                ret,thresh_yellow = cv.threshold(main_mask_yellow, 200, 255, cv.THRESH_BINARY_INV)
                # Simple blob detector
                params = cv.SimpleBlobDetector_Params()
                # Filter by area
                params.filterByArea = False
                # Filter by Cicularity
                params.filterByCircularity = True
                params.minCircularity = 0.6
                # #Filter by Convexity
                params.filterByConvexity = False
                # # Filter by Inertia
                params.filterByInertia = False
                detector = cv.SimpleBlobDetector_create(params)
                keypoints_yellow = detector.detect(thresh_yellow)
                main_size = 0
                main_xcord = 0
                for point in keypoints_yellow:
                    if point.size > 15:
                        main_size = point.size
                        main_xcord = point.pt[0]
                        print("Blob Size: " + str(main_size))
                        print("Blob X: " + str(main_xcord))
                if main_size != 0:
                    main_size = float(24.0/main_size)
                theta = center_width - main_xcord
                vel = main_size
                ang = theta/center_width
                if vel > 0.2: vel = 0.2
                if abs(ang) > 0.1:
                    if ang > 0 :
                        ang = 0.1
                    else:
                        ang = -0.1

                if main_size == 0:
                    print("No Object Detected")
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = 0.1
                else:
                    if abs(theta) < 100:
                        vel_msg.angular.z = 0
                        if main_size > 0.25:
                            #print("Velocidad Lineal: " + str(vel))
                            #print("Velocidad Angular: " + str(ang))
                            vel_msg.linear.x = vel
                        else:
                            print("Object Reached")
                            print("===============================")
                            vel_msg.linear.x = 0
                            vel_msg.angular.z = 0
                    else:
                        vel_msg.angular.z = ang
                        vel_msg.linear.x = 0
                # 300 valor del blob
                self.pub_vel.publish(vel_msg)
                blank = np.zeros((1,1))
                im_with_keypoints = cv.drawKeypoints(thresh_yellow, keypoints_yellow, blank, (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                self.pub_image.publish(self.bridge.cv2_to_imgmsg(im_with_keypoints, "bgr8"))
                self.image_received = 0
            r.sleep()
    def image_cb(self, data):
        if not self.image_received:
            try:
                #print("received ROS image, I will convert it to opencv")
                # We select bgr8 because it is the OpenCV encoding by default
                self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.cv_width = data.width
                self.image_received = 1 #Turn the flag on
            except CvBridgeError as e:
                print(e)
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
        objectFollower()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')
