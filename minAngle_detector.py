#!/usr/bin/env python 
import rospy 
import numpy as np
from sensor_msgs.msg import LaserScan

class minAngleClass():
    '''
        This class receives a LaserScan and find the closest object
    '''
    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################ 
        rate = 10
        self.range_angle = 0
        ###******* INIT PUBLISHERS *******### 
        ###******* INIT SUSCRIBERS *******### 
        rospy.Subscriber('scan',LaserScan,self.laser_cb)
        #********** INIT NODE **********### 
        r = rospy.Rate(rate) #1Hz 
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            r.sleep()
    def laser_cb(self, laser_msg): 
        #cosa = LaserScan()
        angle_increment = laser_msg.angle_increment
        min_angle = laser_msg.angle_min
        min_range = self.min_exclude(laser_msg.ranges)
        range_index = laser_msg.ranges.index(min_range)
        self.range_angle = (angle_increment*range_index) + min_angle
        print("Distancia a objeto mas cercano: " + str(np.round((min_range*100),2)) + " cm.")
        print("Angulo del objecto mas cercano: " + str(np.round(np.degrees(self.range_angle),2)) + " grados")
        print("________________________________________________")
    def min_exclude(self, array):
        min_number = 1000
        for num in array:
            if num != 0 and num < min_number:
                min_number = num
        return min_number
    def cleanup(self):
        print("\n-----FINALIZANDO-----\n")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("minDist_node", anonymous=True) 
    try:
        minAngleClass()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')