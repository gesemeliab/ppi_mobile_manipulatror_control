#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 17 13:05:19 2020

@author: gesem
"""

#import numpy as np
import math
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped


class MobileRobot(object):
    
    def __init__(self):
        
        self.x_r = rospy.get_param('/dparams/init_mobile_pose/x_r')
        self.y_r = rospy.get_param('/dparams/init_mobile_pose/y_r')
        self.theta = rospy.get_param('/dparams/init_mobile_pose/theta')
                
        rospy.Subscriber('pose', Vector3Stamped, self.mobile_goal_calllback)
        rospy.Subscriber('coordinate2', PointStamped, self.XYZ_callback)

            
        self.pub = rospy.Publisher('mobile_goal', Vector3Stamped, queue_size=10)
        
        self.rate = rospy.Rate(10) # 10hz
    
        ready = False
        while ready == False and not rospy.is_shutdown():
            c = self.pub.get_num_connections()
            if c > 0:
                ready = True
            else:
                self.rate.sleep()

    def XYZ_callback(self, msg): 
        
        x_g = msg.point.x
        y_g = msg.point.y   
        #self.beta = msg.point.z
        
        y_dif = y_g - self.y_r
        x_dif = x_g - self.x_r
        
        beta = math.atan(y_dif/x_dif)
        
        if x_dif < 0 :
            beta = math.radians(180) + beta
        elif x_dif < 0 and y_dif < 0:
            beta = math.radians(180) - beta
        
        rho = math.sqrt( ((x_g - self.x_r)**2)+((y_g - self.y_r)**2))
        alpha = beta - self.theta
        
        v, w = self.compute_v_w(rho, alpha)
                
        pos = Vector3Stamped()
        pos.header.stamp = rospy.Time.now()
        pos.vector.x = v
        pos.vector.y = w    
        pos.vector.z = self.theta
        self.pub.publish(pos)
    
           
        self.rate.sleep()
        
        
    def mobile_goal_calllback(self,r_pos):
        
        self.x_r = r_pos.vector.x
        self.y_r = r_pos.vector.y
        self.theta = r_pos.vector.z
        

    def compute_v_w(self, rho, alpha):
        
        k_rho = 6#rad/s
        k_alpha = 10 #1/s
                
        v = k_rho * rho * math.cos(alpha) #v=cm/s
        w = ( k_rho * math.sin(alpha) * math.cos(alpha) ) + k_alpha * alpha #w=rad/s
        return v, w
    
    def loop(self):
        rospy.logwarn("Starting Mobile Loop...")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('mobile_node', anonymous=True, log_level=rospy.WARN)
    mobile_obj = MobileRobot()
    mobile_obj.loop()


