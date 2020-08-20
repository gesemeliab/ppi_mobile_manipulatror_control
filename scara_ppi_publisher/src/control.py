#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 13 20:23:23 2020

@author: gesem
"""

import rospy
import math
from geometry_msgs.msg import Vector3Stamped



class ControlPub(object):
    
    def __init__(self):
        
        self._kp = rospy.get_param('/dparams/kp')
        self._kpi = rospy.get_param('/dparams/kpi')
        self._dt = rospy.get_param('/dparams/dt')
        
        self.theta = rospy.get_param('/dparams/init_mobile_pose/theta')
        
        self.control_p = [0.0]*3
        self.control_pi = [0.0]*5
        
        self.control_p = [0.0]*3
        self.err_pi = [0.0]*5
        
        self.control_pi_ant = [0.0]*5 
        self.err_pi_ant = [0.0]*5
               
        self.pos = [0.0]*5 #mobile
        self.vel = [0.0]*5 #mobile
        
        self.w_list = [0.0]*2
        self.goal = [0.0]*5
        
        self.time = 0.0 
                
        rospy.Subscriber('set_point', Vector3Stamped, self.goal_callback)
        
        #mobile
        rospy.Subscriber('mobile_goal', Vector3Stamped, self.mobile_goal_callback)
        
        rospy.Subscriber('m1_response', Vector3Stamped, self.m1_response_callback)
        rospy.Subscriber('m2_response', Vector3Stamped, self.m2_response_callback)
        rospy.Subscriber('m3_response', Vector3Stamped, self.m3_response_callback)
        
        #mobile
        rospy.Subscriber('m4_response', Vector3Stamped, self.m4_response_callback)
        rospy.Subscriber('m5_response', Vector3Stamped, self.m5_response_callback)
        
        self.pub = rospy.Publisher('p_control', Vector3Stamped, queue_size=10)
        self.pub2 = rospy.Publisher('pi_control', Vector3Stamped, queue_size=10)
        self.pub3 = rospy.Publisher('mob_pi_control', Vector3Stamped, queue_size=10)

        
        self.rate = rospy.Rate(10) # 10hz
        
        ready = False
        while ready == False:
            c = self.pub2.get_num_connections()
            if c > 0:
                ready = True
            else:
                self.rate.sleep()

    
    def goal_callback(self,goalv):
        
        self.time += 0.1
        # This callback is the boss, this one dictates the publish rate
        rospy.logwarn("GOT SCARA goal 1 = "+str(goalv.vector.x))
        goal1 = goalv.vector.x
        rospy.logwarn("GOT SCARA goal 2 = "+str(goalv.vector.y))
        goal2 = goalv.vector.y
        rospy.logwarn("GOT SCARA goal 3 = "+str(goalv.vector.z))
        goal3 = goalv.vector.z
        
        self.goal = [goal1, goal2, goal3]
        
        self.control(self.goal, self.w_list)
        
        self.rate.sleep()
    
        
    #mobile    
    def mobile_goal_callback(self, msg):
        
        #self.time += 0.1
        radius = 2.0
        d = 8.0
        
        v = msg.vector.x
        w = msg.vector.y
        self.theta = msg.vector.z
        

        self.w_l = (((2*v) - (w*d))/2*radius)/radius
        self.w_r = (((2*v) + (w*d))/2*radius)/radius

        self.w_list = [self.w_l, self.w_r]
        

                
    def m1_response_callback(self,response1):

        #rospy.logwarn("GOT Pos Response for M1="+str(response1.vector.x))
        self.pos[0] = math.radians(response1.vector.x)
    
        #rospy.logwarn("GOT Vel Response for M1="+str(response1.vector.y))
        self.vel[0] = response1.vector.y*(60/(2*math.pi))
    
    def m2_response_callback(self,response2):

        #rospy.logwarn("GOT Pos Response for M2="+str(response2.vector.x))
        self.pos[1] = math.radians(response2.vector.x)
    
        #rospy.logwarn("GOT Vel Response for M2="+str(response2.vector.y))
        self.vel[1] = response2.vector.y*(60/(2*math.pi))
        
    def m3_response_callback(self,response3):

        #rospy.logwarn("GOT Pos Response for M3 = "+str(response3.vector.x))
        self.pos[2] = (response3.vector.x*2*math.pi)/0.2
    
        #rospy.logwarn("GOT Vel Response for M3 = "+str(response3.vector.y))
        self.vel[2] = response3.vector.y*(60/(2*math.pi))
    
    #mobile    
    def m4_response_callback(self,response4):

        #rospy.logwarn("GOT Pos Response for M3 = "+str(response3.vector.x))
        self.pos[3] = response4.vector.x
    
        #rospy.logwarn("GOT Vel Response for M3 = "+str(response3.vector.y))
        self.vel[3] = response4.vector.y*(60/(2*math.pi))
    #mobile    
    def m5_response_callback(self,response5):

        #rospy.logwarn("GOT Pos Response for M2="+str(response2.vector.x))
        self.pos[4] = response5.vector.x
    
        #rospy.logwarn("GOT Vel Response for M2="+str(response2.vector.y))
        self.vel[4] = response5.vector.y*(60/(2*math.pi))
        
    def publisher(self, p, pi):        
        proportional = Vector3Stamped()
        proportional.header.stamp = rospy.Time.now()
        proportional.vector.x = p[0]
        proportional.vector.y = p[1]
        proportional.vector.z = p[2]
        self.pub.publish(proportional)
        
        p_integral = Vector3Stamped()
        p_integral.header.stamp = rospy.Time.now()
        p_integral.vector.x = pi[0]
        p_integral.vector.y = pi[1]
        p_integral.vector.z = pi[2]
        self.pub2.publish(p_integral)
        
        mob_p_integral = Vector3Stamped()
        mob_p_integral.header.stamp = rospy.Time.now()
        mob_p_integral.vector.x = pi[3] #l
        mob_p_integral.vector.y = pi[4] #r
        mob_p_integral.vector.z = self.theta
        self.pub3.publish(mob_p_integral)

    
    def control(self, goal, w):
        
        self._kp = rospy.get_param('/dparams/kp')
        self._kpi = rospy.get_param('/dparams/kpi')
                        
        for n in range(0,3):
            #Control Proporcional para posicion
            self.control_p[n] = self._kp*(goal[n] - self.pos[n])
        	#Control Proporcional-Integral para velocidad
            self.err_pi[n] = self.control_p[n] - self.vel[n]
            self.control_pi[n] = self.control_pi_ant[n] + (self._kpi*((self.err_pi[n]-self.err_pi_ant[n])+(self.time/self._dt)*((self.err_pi[n]-self.err_pi_ant[n])/2)))
                                
            self.err_pi_ant[n] = self.err_pi[n]
            self.control_pi_ant[n] = self.control_pi[n]
            
        for m in range(3,5):
        	#Control Proporcional-Integral para velocidad
            self.err_pi[m] = w[m - 3] - self.vel[m]
            self.control_pi[m] = self.control_pi_ant[m] + (self._kpi*((self.err_pi[m]-self.err_pi_ant[m])+(self.time/self._dt)*((self.err_pi[m]-self.err_pi_ant[m])/2)))
                        
            self.err_pi_ant[m] = self.err_pi[m]
            self.control_pi_ant[m] = self.control_pi[m]
                        
        self.publisher(self.control_p, self.control_pi)

        
        
    def loop(self):
        rospy.logwarn("Starting Control Loop...")
        rospy.spin()
              
if __name__ == '__main__':
    rospy.init_node('ppi_node', anonymous=True, log_level=rospy.INFO)
    contr_obj = ControlPub()
    contr_obj.loop()

        

