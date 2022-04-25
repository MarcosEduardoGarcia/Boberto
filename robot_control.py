#!/usr/bin/env python 
from ast import increment_lineno
from sre_constants import SUCCESS
import rospy 
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class SquareClass():
    '''
        This class will make a puzzlebot follow a 1 m square
    '''
    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################ 
        self.msg = Twist() # Mesage to be send
        self.wr = 0
        self.wl = 0
        rate = 100
        wRad = 0.05 # Wheel radius in Meters
        L = 0.19 # Wheel separation in Meters
        dx = 0 # Distance in X
        dy = 0 # Distance in Y
        theta = 0 # Angular Distance
        Dt = 1/float(rate) # Dt is the time between one calculation and the next one
        goal_x = [1.2,1,0,0,0.8,-0.8]
        goal_y = [0,1,0.8,-1.2,0.8,-0.8]
        index = 1
        success = False
        ###******* INIT PUBLISHERS *******### 
        self.pub_move = rospy.Publisher('cmd_vel', Twist, queue_size=1)
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
            if theta > np.pi : theta = theta - (2*np.pi)
            elif theta < (-1*np.pi) : theta = theta + (2*np.pi)
            dx = dx + (v*(np.cos(theta))*Dt)
            dy = dy + (v*(np.sin(theta))*Dt)

            e_theta = (np.arctan2(goal_y[index] - dy, goal_x[index] - dx)) - theta
            e_d = np.sqrt(((goal_x[index] - dx)**2)+((goal_y[index] - dy)**2))

            # Control Proporcional
            vel = 1*e_d
            giro = 1*e_theta
            # Limites de Velocidad
            if vel > 0.2: vel = 0.2
            if giro > 0.2: giro = 0.2
            
            # Control de Errores
            if abs(e_theta) < 0.1:
                e_theta = 0
                self.msg.linear.x = vel
                if abs(e_d) < 0.1:
                    e_d = 0
                    e_theta = 0
                    self.msg.linear.x = 0
                    self.msg.angular.z = 0

            # Publicando las velocidades y giros ald robot
            self.msg.angular.z = giro
            self.pub_move.publish(self.msg)

            #print("Distancia en X y Y: [" + str(np.round(dx,2)) + "," + str(np.round(dy,2)) + "]")
            #print("Posicion Angular: " + str(np.round(theta,2)))
            print("Error Angulo: "+ str(np.round(e_theta,2)))
            print("Error Distancia: "+ str(np.round(e_d,2)))
            r.sleep()
    def wr_cb(self, num): self.wr = num.data
    def wl_cb(self, num): self.wl = num.data
    def go_stop(self):
        # Detener
        self.msg.angular.z = 0
        self.msg.linear.x = 0
        self.pub_move.publish(self.msg)
    def cleanup(self):
        self.go_stop()
        print("\n-----FINALIZADO-----\n")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("square_node", anonymous=True) 
    try:
        SquareClass()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')