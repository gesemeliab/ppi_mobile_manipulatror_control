#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 13 22:17:03 2020

@author: gesem
"""
import rospy
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import math


class Mysystem(object):
    
    def __init__(self):

        J = rospy.get_param('/dparams/motor_params/J_1')
        Ki = rospy.get_param('/dparams/motor_params/Ki_1')
        Km = rospy.get_param('/dparams/motor_params/Km_1')
        b = rospy.get_param('/dparams/motor_params/b_1')
        L = rospy.get_param('/dparams/motor_params/L_1')
        R = rospy.get_param('/dparams/motor_params/R_1')
        
        self.theta_mob = rospy.get_param('/dparams/init_mobile_pose/theta')
        
        self.pi = [0.0, 0.0, 0.0, 0.0, 0.0]
        #self.pi = [0.0,0.0,0.0]
        
        self.pi4 = 0.0
        self.pi5 = 0.0
        
        self.A = np.array([[0,1,0],[0,-b/J, Ki/J],[0,-Km/L,-R/L]])
        self.B = np.array([[0,0],[0,1/J],[1/L,0]])
        #C = np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.x = np.zeros((5,3,1))
        self.dx = np.zeros((5,3,1))
        
        #mobile
        self.POS = np.zeros((3,1))
        self.POS_ant = np.zeros((3,1))
        self.ANG = np.zeros((3,2))
        self.DELTA = np.zeros((2,1))

        
        
        rospy.Subscriber('pi_control', Vector3Stamped, self.system_callback)
        rospy.Subscriber('mob_pi_control', Vector3Stamped, self.mob_system_callback)

        self.pub1 = rospy.Publisher('m1_response', Vector3Stamped, queue_size=10)
        self.pub2 = rospy.Publisher('m2_response', Vector3Stamped, queue_size=10)
        self.pub3 = rospy.Publisher('m3_response', Vector3Stamped, queue_size=10)
        
        #mobile
        self.pub4 = rospy.Publisher('m4_response', Vector3Stamped, queue_size=10)
        self.pub5 = rospy.Publisher('m5_response', Vector3Stamped, queue_size=10)
        self.pub6 = rospy.Publisher('pose', Vector3Stamped, queue_size=10)

        self.rate = rospy.Rate(10) # 10hz

        ready = False
        while ready == False:
            #mobile
            c = self.pub3.get_num_connections()
            if c > 0:
                ready = True
            else:
                self.rate.sleep()
        
        
        
    def system_callback(self, controlv):
        
        self.pi1 = controlv.vector.x
        self.pi2 = controlv.vector.y
        self.pi3 = controlv.vector.z
        
        self.pi = [self.pi1,self.pi2,self.pi3, self.pi4, self.pi5]
        #self.pi = [self.pi1,self.pi2,self.pi3]
        #mobile
        self.ANG = [[math.cos(self.theta_mob),0.0],[math.sin(self.theta_mob),0.0],[0.0,1.0]]
        #mobile cambio a 2
        for n in range(0,5):
            
            u = [[1],[self.pi[n]]]
            self.dx[n] = np.dot(self.A,self.x[n]) + np.dot(self.B,u)
            self.x[n] += self.dx[n]*0.1
                    
        print("Motor 1 Position: ",math.degrees(self.x[0][0]),"Velocity: ",self.x[0][1]*((math.pi*2)/60))
        print("Motor 2 Position: ",math.degrees(self.x[1][0]),"Velocity: ",self.x[1][1]*((math.pi*2)/60))
        print("Motor 3 Position: ",(self.x[2][0]/(2*math.pi))*0.2,"Velocity: ",self.x[2][1]*((math.pi*2)/60))
                #mobile  
        
        self.DELTA = self.sensor(self.x[3][1],self.x[4][1])
        #mobile
        self.POS = self.POS_ant + np.dot(self.ANG,self.DELTA)
        print(self.POS)
        
        self.publisher(self.x)
          
        #mobile
        pose = Vector3Stamped()
        pose.header.stamp = rospy.Time.now()
        pose.vector.x = self.POS[0]
        pose.vector.y = self.POS[1]
        pose.vector.z = self.POS[2]
        self.pub6.publish(pose)
        #mobile
        self.POS_ant = self.POS
        
        self.rate.sleep()
      
    def mob_system_callback(self, msg):
        self.pi4 = msg.vector.x
        self.pi5 = msg.vector.y
        self.theta_mob = msg.vector.z
                

    #mobile
    def sensor(self,wl,wr):
        
        radius= 2.0
        Ts = 0.01
        d = 8.0
        d_s = (radius/2)*((wr*Ts) + (wl*Ts))
        d_theta = (radius/d)*((wr*Ts) - (wl*Ts))
        delta = np.array([d_s,d_theta])
        return delta
    
        
    def publisher(self, m):
        
        #rospy.loginfo(m[0])
        m1_response = Vector3Stamped()
        m1_response.header.stamp = rospy.Time.now()
        m1_response.vector.x = math.degrees(m[0][0])
        #m1_response.vector.xm[0][0]
        m1_response.vector.y = m[0][1]*((math.pi*2)/60)
        m1_response.vector.z = m[0][2]
        self.pub1.publish(m1_response)

        #rospy.loginfo(m[1])
        m2_response = Vector3Stamped()
        m2_response.header.stamp = rospy.Time.now()
        m2_response.vector.x = math.degrees(m[1][0])
        m2_response.vector.y = m[1][1]*((math.pi*2)/60)
        m2_response.vector.z = m[1][2]
        self.pub2.publish(m2_response)

        #rospy.loginfo(m[2])
        m3_response = Vector3Stamped()
        m3_response.header.stamp = rospy.Time.now()
        m3_response.vector.x = (m[2][0]/(2*math.pi))*0.2
        m3_response.vector.y = m[2][1]*((math.pi*2)/60)
        m3_response.vector.z = m[2][2]
        self.pub3.publish(m3_response)
        
        #rospy.loginfo(m[0])
        m4_response = Vector3Stamped()
        m4_response.header.stamp = rospy.Time.now()
        m4_response.vector.x = m[3][0]
        m4_response.vector.y = m[3][1]*((math.pi*2)/60)
        m4_response.vector.z = m[3][2]
        self.pub4.publish(m4_response)

        #rospy.loginfo(m[1])
        m5_response = Vector3Stamped()
        m5_response.header.stamp = rospy.Time.now()
        m5_response.vector.x = m[4][0]
        m5_response.vector.y = m[4][1]*((math.pi*2)/60)
        m5_response.vector.z = m[4][2]
        self.pub5.publish(m5_response)
        
        
    def loop(self):
        rospy.logwarn("Starting System Loop...")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('system_node', anonymous=True, log_level=rospy.WARN)
    sys_obj = Mysystem()
    sys_obj.loop()
