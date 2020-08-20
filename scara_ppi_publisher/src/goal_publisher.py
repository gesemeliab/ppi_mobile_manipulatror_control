#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 13 20:11:57 2020

@author: gesem
"""

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import math

class GoalPub(object):
    
    def __init__(self):
        
        rospy.Subscriber('coordinate', PointStamped, self.XYZ_callback)
                
        self.pub = rospy.Publisher('set_point', Vector3Stamped, queue_size=10)
                
        self.rate = rospy.Rate(10) # 10hz
        
        ready = False
        while ready == False:
            c = self.pub.get_num_connections()
            if c > 0:
                ready = True
            else:
                self.rate.sleep()
            

    
    def XYZ_callback(self,msg):
        
        self.x_coordinate = msg.point.x
        self.y_coordinate = msg.point.y
        self.z_coordinate = msg.point.z
        
        goal1, goal2, goal3 = self.scara_ik(self.x_coordinate, self.y_coordinate, self.z_coordinate)
        
        sp = Vector3Stamped()
        sp.header.stamp = rospy.Time.now()
        sp.vector.x = goal1
        sp.vector.y = goal2
        sp.vector.z = goal3
        self.pub.publish(sp)
        
        self.rate.sleep()
        
    def scara_ik(self,x,y,z):

        #B1 = 6.2  
        L1 = 5.2  
        #E1 = 0  
        L2 = 7  
                
        r1 = np.sqrt(x**2+y**2)
        
        maxR = L1 + L2
        minR = abs(L1-L2)
        
        if r1 > maxR or r1 < minR:
            print("Coordenada fuera del área de trabajo")
            return 12.2, 0.0, 0.0
            
        
        else:
            phi_1 = np.arccos((L2**2-L1**2-r1**2)/(-2*L1*r1))  
            phi_2 = np.arctan2(y, x)  
            theta_1 = phi_2 - phi_1
            
            phi_3 = np.arccos((r1**2-L1**2-L2**2)/(-2*L1*L2))
            theta_2 = math.pi - phi_3
            
            #tornillo acme de 4 hilos con pitch de 0.2mm
            theta_3 = -(z*2*math.pi)/0.2
            
            return theta_1, theta_2, theta_3


        
    def loop(self):
        rospy.logwarn("Starting Goal Loop...")
        rospy.spin()
              
if __name__ == '__main__':
    rospy.init_node('ppi_node', anonymous=True, log_level=rospy.INFO)
    goal_obj = GoalPub()
    goal_obj.loop()

        

