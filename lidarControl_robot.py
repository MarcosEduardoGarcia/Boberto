#!/usr/bin/env python 
import rospy 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class lidarControlClass():
    '''
        This class will make a puzzlebot follow a 1 m square
    '''
    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS Aparte ################ 
        wRad = 0.05 # Wheel radius in Meters
        L = 0.19 # Wheel separation in Meters
        max_rads = 17.802358
        v_max = (wRad*(max_rads + max_rads))/2
        print("Velocidad Maxima del Robot: " + str(v_max) + " m/s")
        w_max = (wRad*(max_rads + max_rads))/L # All in radians
        print("Velocidad Angular Absoluta Maxima del Robot: " + str(w_max) + " rad/s")
        ############ CONSTANTS ################ 
        msg = Twist() # Mesage to be send
        rate = 10
        self.clos_theta = 0
        self.clos_range = 0 
        alpha = 0.1 #Me va a decir que tan rapido llega la exponencial
        kmax = v_max # Valor Maximo en la variable de proporcionalidad velocidad maxima de robot
        kw = 1.2
        ###******* INIT PUBLISHERS *******### 
        self.pub_move = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        ###******* INIT SUSCRIBERS *******### 
        rospy.Subscriber('base_scan', LaserScan, self.laser_cb)
        #********** INIT NODE **********### 
        r = rospy.Rate(rate) #1Hz 
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            # Creamos copias de seguridad
            ranges = self.clos_range
            theta = self.clos_theta
            
            kv_aux = 1 - np.exp((ranges**2)*(alpha*-1))
            kv = kmax*(kv_aux/ranges)
            theta = np.arctan2(np.sin(theta), np.cos(theta))
            if np.isposinf(ranges):
                print("No Object Detected")
                msg.linear.x = 0
                msg.angular.z = 0
            else:
                msg.angular.z = kw*theta
                if ranges > 1:
                    msg.linear.x = kv*ranges
                else:
                    print("Object Reached")
                    print("===============================")
                    msg.linear.x = 0
                # Imprimiendo Datos
                print("Error Angulo: "+ str(np.round(theta,2)))
                print("Error Distancia: "+ str(np.round(ranges,2)))
                print("===============================")
            self.pub_move.publish(msg)
            r.sleep()
    def laser_cb(self, laser_msg): 
        #cosa = LaserScan()
        angle_increment = laser_msg.angle_increment
        min_angle = laser_msg.angle_min
        self.clos_range = self.min_exclude(laser_msg.ranges)
        range_index = laser_msg.ranges.index(self.clos_range)
        self.clos_theta = (angle_increment*range_index) + min_angle
        #print("Distancia a objeto mas cercano: " + str(np.round((min_range*100),2)) + " cm.")
        #print("Angulo del objecto mas cercano: " + str(np.round(np.degrees(self.range_angle),2)) + " grados")
        #print("________________________________________________")
    def min_exclude(self, array):
        min_number = np.inf
        for num in array:
            if num != 0 and num < min_number:
                min_number = num
        return min_number
    def go_stop(self):
        # Detener
        vel_msg = Twist() # Generar un Twist en 0
        self.pub_move.publish(vel_msg)
    def cleanup(self):
        self.go_stop()
        print("\n-----FINALIZADO-----\n")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("Byakugan", anonymous=True) 
    try:
        lidarControlClass()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')
# Velocidad Maxima lineal
# Velocidad Maxima Angular