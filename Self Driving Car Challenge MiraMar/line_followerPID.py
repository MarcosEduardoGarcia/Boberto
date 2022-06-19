#!/usr/bin/env python 
from shutil import move
import rospy
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Int16


class line_follower():
    '''
        This class will suscribe to a topic with an Image and
        process the same image to find the colour Red and Green
    '''
    def __init__(self): 
        # Callback fuction when the node is turned off
        rospy.on_shutdown(self.cleanup) 
        ############ CONSTANTS ################ 
        rate = 30
        vel_msg = Twist()
        self.main_pos = 0
        self.main_state = 0
        self.main_signal = 0
        memory_text = 'No_sign'
        state = 0
        width = 640
        vel = 0.0
        giro = 0.0
        move_flag = False
        #############CONSTANTES CONTROL #################
        error_ant = 0
        acum = 0
        ref = width//2
        kp = 0.002 #0.0055
        # ki = 0.00015
        # ki = kp*0.05
        #ki = kp*0.000015
        # ki = kp*0.025
        ki = 0.0
        # kd = 0.000001
        # kd = kp/300
        kd = 0.0 # kp/10
        Dt = 1/float(rate) # Dt is the time between one calculation and the next one
        ###******* INIT PUBLISHERS *******### 
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        ###******* INIT SUSCRIBERS *******### 
        rospy.Subscriber('line_pos',Int16,self.pos_cb)
        rospy.Subscriber('traffic_light',Bool,self.state_cb)
        rospy.Subscriber('signal_inst', Int16, self.signal_cb)
        #********** INIT NODE **********### 
        r = rospy.Rate(rate) #1Hz 
        print("Node initialized " + str(rate)+ " hz")
        while not rospy.is_shutdown():
            
            signal = self.main_signal
            text = self.translate(signal)
            if text != "No_sign":
                memory_text = text
            print(memory_text)
            if memory_text != "Stop":
                state = self.main_state
                print(acum)
                print("Status " + str(state))
                # Revisa si ve la linea
                pos = self.main_pos
                if pos != 1000 and not move_flag:
                    print("Control Line Follower")
                    error = ref - pos
                    P = float(error) * kp
                    I = float((error + error_ant)/2) * ki
                    D = float((error - error_ant)/Dt) * kd
                    control = P + I + D
                    error_ant = error
                    giro = control
                    if memory_text == "No_limit_Vel" and acum < 400:
                        print("No Limit Vel")
                        vel = 0.13
                        acum += 1
                    elif acum == 400:
                        acum = 0
                        memory_text = "No_sign"
                    else:
                        vel = 0.1
                    vel, giro = self.limit_vel(vel,giro)
                    vel_msg.linear.x = vel
                    vel_msg.angular.z = giro
                # Si no ve la linea
                else:
                    print("No line")
                    if state and memory_text == "Go_forward" and acum < 210:
                        print("Go Forward")
                        move_flag = True
                        vel_msg.linear.x = 0.1
                        vel_msg.angular.z = 0
                        acum += 1
                    elif state and memory_text == "Go_right" and acum < 150:
                        print("Go Right")
                        move_flag = True
                        if acum < 90:
                            vel_msg.linear.x = 0.1
                            vel_msg.angular.z = 0
                        elif acum < 120:
                            vel_msg.linear.x = 0
                            vel_msg.angular.z = -0.3
                        elif acum < 150:
                            vel_msg.linear.x = 0.1
                            vel_msg.angular.z = 0
                        acum += 1
                    elif not state:
                        print("Light Stop")
                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                    else:
                        print("Change Instruction")
                        vel_msg.linear.x = 0
                        vel_msg.angular.z = 0
                        acum = 0
                        memory_text = "No_sign"
                        move_flag = False
            elif memory_text == "Stop" and acum < 90:
                print("Sign Stop")
                vel_msg.linear.x = 0.1
                vel_msg.angular.z = 0.0
                acum += 1
            else:
                print("Finish Program")
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
            self.pub_vel.publish(vel_msg)
            print("===============================")
            r.sleep()
    def pos_cb(self, num): self.main_pos = num.data 
    def state_cb(self, num): self.main_state = num.data
    def signal_cb(self, num): self.main_signal = num.data 
    def limit_vel(self,vel,giro):
        '''
        Fuction that limits the velocities the robot can achieve
        '''
        if vel > 0.2: vel = 0.2
        if abs(giro) > 0.35:
            if giro > 0: giro = 0.35
            else: giro = -0.35
        return vel, giro
    def translate(self,num):
        if num == 14: return 'Stop'
        elif num == 32: return 'No_limit_Vel'
        elif num == 33: return 'Go_right'
        elif num == 35: return 'Go_forward'
        else: return 'No_sign'
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
        line_follower()
    except rospy.ROSInterruptException:
        print('\nEXECUTION COMPLETED SUCCESFULLY')
