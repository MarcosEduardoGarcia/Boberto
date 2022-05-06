#!/usr/bin/env python 
import rospy 
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool

class p2p_navi():
    '''
        This class will make the robot follow some points in the space
    '''
    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################ 
        msg = Twist() # Mesage to be send
        self.wr = 0
        self.wl = 0
        rate = 100
        wRad = 0.05 # Wheel radius in Meters
        L = 0.19 # Wheel separation in Meters
        dx = 0 # Distance in X
        dy = 0 # Distance in Y
        theta = 0 # Angular Distance
        Dt = 1/float(rate) # Dt is the time between one calculation and the next one
        #goal_x = [1.5,2.4,2.4,3.3,3.3]
        #goal_y = [0,-1,-5.4,-6.3,-10.3]
        goal_x = [1,1.5,2,1.5,1,0.2]
        goal_y = [0,-0.5,0,-0.5,0,0.2]
        index = 0
        self.status = False
        ###******* INIT PUBLISHERS *******### 
        self.pub_move = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        ###******* INIT SUSCRIBERS *******### 
        rospy.Subscriber('wl',Float32,self.wl_cb)
        rospy.Subscriber('wr',Float32,self.wr_cb)
        rospy.Subscriber('traffic_light',Bool,self.tl_cb)
        #********** INIT NODE **********### 
        r = rospy.Rate(rate) #1Hz 
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            if self.status:
                # Sacamos los objetivos de un arreglo ya definido
                if index >= len(goal_x):
                    main_goal_x = 0
                    main_goal_y = 0
                else:
                    main_goal_x = goal_x[index]
                    main_goal_y = goal_y[index]
                v = (wRad*(self.wr + self.wl))/2
                w = (wRad*(self.wr - self.wl))/L # All in radians
                theta = (w*Dt)+theta
                if theta > np.pi : theta = theta - (2*np.pi)
                elif theta < (-1*np.pi) : theta = theta + (2*np.pi)
                #print(theta)
                dx = dx + (v*(np.cos(theta))*Dt)
                dy = dy + (v*(np.sin(theta))*Dt)
                
                e_theta = (np.arctan2(main_goal_y - dy, main_goal_x - dx)) - theta
                e_d = np.sqrt(((main_goal_x - dx)**2)+((main_goal_y - dy)**2))
                # Control Proporcional
                vel = e_d
                giro = 0.5*e_theta
                # Limites de Velocidad
                if vel > 0.35: vel = 0.35
                if abs(giro) > 0.2: 
                    if giro > 0:
                        giro = 0.2
                    else:
                        giro = -0.2
                
                if index >= len(goal_x):
                    print("Done with the program")
                    msg.linear.x = 0
                    msg.angular.z = 0.5
                else:
                    if abs(e_theta) < 0.1:
                        msg.angular.z = 0
                        if abs(e_d) < 0.1:
                            msg.linear.x = 0
                        else:
                            msg.linear.x = vel
                    else:
                        msg.angular.z = giro
                    if abs(e_theta) < 0.1 and e_d < 0.1:
                        index += 1
            else:
                print("Trafic Light Stop")
                msg.linear.x = 0
                msg.angular.z = 0
            self.pub_move.publish(msg)
            r.sleep()
    def wr_cb(self, num): self.wr = num.data
    def wl_cb(self, num): self.wl = num.data
    def tl_cb(self, state): self.status = state.data
    def go_stop(self):
        # Detener
        stop = Twist()
        self.pub_move.publish(stop)
    def cleanup(self):
        self.go_stop()
        print("\n-----FINALIZADO-----\n")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("point2point_node", anonymous=True) 
    try:
        p2p_navi()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')
