#!/usr/bin/env python 
import rospy 
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float32

class SquareClass():
    '''
        This class will make a puzzlebot follow a 1 m square
    '''
    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################ 
        self.wr = 0
        self.wl = 0
        rate = 100
        wRad = 0.05 # Wheel radius in Meters
        L = 0.19 # Wheel separation in Meters
        dx = 0 # Distance in X
        dy = 0 # Distance in Y
        theta = 0 # Angular Distance
        Dt = 1/float(rate) # Dt is the time between one calculation and the next one
        goal_x = 1
        goal_y = -1
        ###******* INIT PUBLISHERS *******### 
        ###******* INIT SUSCRIBERS *******### 
        rospy.Subscriber('wl',Float32,self.wl_cb)
        rospy.Subscriber('wr',Float32,self.wr_cb)
        #********** INIT NODE **********### 
        r = rospy.Rate(rate) #1Hz 
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            v = (wRad*(self.wr + self.wl))/2
            w = (wRad*(self.wr - self.wl))/L # All in radians
            theta = (w*Dt)+theta
            dx = dx + (v*(np.cos(theta))*Dt)
            dy = dy + (v*(np.sin(theta))*Dt)
            
            e_theta = (np.arctan2(goal_y - dy, goal_x - dx)) - theta
            e_d = np.sqrt(((goal_x - dx)**2)+((goal_y - dy)**2))
            print("Distancia en X y Y: [" + str(np.round(dx,2)) + "," + str(np.round(dy,2)) + "]")
            print("Posicion Angular: " + str(np.round(theta,2)))
            print("Error Angulo: "+ str(np.round(np.degrees(e_theta),2)))
            print("Error Distancia: "+ str(np.round(e_d,2)))
            r.sleep()
    def wr_cb(self, num): self.wr = num.data
    def wl_cb(self, num): self.wl = num.data
    def cleanup(self): 
        print("\n-----FINALIZADO-----\n")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("square_node", anonymous=True) 
    try:
        SquareClass()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')