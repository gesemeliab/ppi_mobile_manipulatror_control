#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 16 21:46:20 2020

@author: gesem
"""

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3Stamped
import numpy as np


class Automatic(object):
    
    def __init__(self):
        
        self.scara_arr = np.zeros((2,3))
        self.mobile_arr = np.zeros((4,2))
        
        self.scara_pos = np.ones((3))
        self.mobile_pos = np.ones((2))
        
        self.scara_arr[0][0] = rospy.get_param('/dparams/pick_coordinate/x_s')
        self.scara_arr[0][1] = rospy.get_param('/dparams/pick_coordinate/y_s')
        self.scara_arr[0][2] = rospy.get_param('/dparams/pick_coordinate/z_s')

        self.scara_arr[1][0] = rospy.get_param('/dparams/place_coordinate/x_s')
        self.scara_arr[1][1] = rospy.get_param('/dparams/place_coordinate/y_s')
        self.scara_arr[1][2] = rospy.get_param('/dparams/place_coordinate/z_s')
        
        self.mobile_arr[0][0] = rospy.get_param('/dparams/wp1/x_m')
        self.mobile_arr[0][1] = rospy.get_param('/dparams/wp1/y_m')
        
        self.mobile_arr[1][0] = rospy.get_param('/dparams/wp2/x_m')
        self.mobile_arr[1][1] = rospy.get_param('/dparams/wp2/y_m')
        
        self.mobile_arr[2][0] = rospy.get_param('/dparams/wp3/x_m')
        self.mobile_arr[2][1] = rospy.get_param('/dparams/wp3/y_m')
        
        self.mobile_arr[3][0] = rospy.get_param('/dparams/wp4/x_m')
        self.mobile_arr[3][1] = rospy.get_param('/dparams/wp4/y_m')
        
        
        rospy.Subscriber('pose', Vector3Stamped, self.pos_callback)
        rospy.Subscriber('m1_response', Vector3Stamped, self.m1_response_callback)
        rospy.Subscriber('m2_response', Vector3Stamped, self.m2_response_callback)
        rospy.Subscriber('m3_response', Vector3Stamped, self.m3_response_callback)

        self.pub1 = rospy.Publisher('coordinate', PointStamped, queue_size=10)
        self.pub2 = rospy.Publisher('coordinate2', PointStamped, queue_size=10)
    
        self.rate = rospy.Rate(10) # 10hz
        
        ready = False
        while ready == False and not rospy.is_shutdown():
            c = self.pub2.get_num_connections()
            if c > 0:
                print('ready to publish')
                ready = True
            else:
                self.rate.sleep()
                        
    
    def main(self):
                
        rospy.logwarn("Maquina de estados...")
         
        for m in range(0, len(self.mobile_arr)):
            self.compute_error_mobile(self.mobile_arr[m][0],self.mobile_arr[m][1])
            for s in range(0,len(self.scara_arr)):
                self.compute_error_scara(self.scara_arr[s][0],self.scara_arr[s][1], self.scara_arr[s][2])
        
            
    def pos_callback(self, msg):
        
        self.mobile_pos[0] = msg.vector.x
        self.mobile_pos[1] = msg.vector.y
        
                
    def compute_error_mobile(self, x_goal, y_goal):  
                
        # | P* - P| / P 

        
        mob_goal = PointStamped()
        mob_goal.header.stamp = rospy.Time.now()
        mob_goal.point.x = x_goal
        mob_goal.point.y = y_goal
        
        err_abs_x = abs(self.mobile_pos[0] - x_goal)
        err_abs_y = abs(self.mobile_pos[1] - y_goal)
        
        
        while err_abs_x/x_goal > 0.05 and err_abs_y/y_goal > 0.05 and not rospy.is_shutdown():
            
            print('MOVIL error en x: ', err_abs_x, 'error en y: ', err_abs_y)
            
            self.pub2.publish(mob_goal)
            
            err_abs_x = abs(self.mobile_pos[0] - x_goal)
            err_abs_y = abs(self.mobile_pos[1] - y_goal)
            
            self.rate.sleep()
            
    def compute_error_scara(self, x_goal, y_goal, z_goal):  
                
        # | P* - P| / P 

        scara_goal = PointStamped()
        scara_goal.header.stamp = rospy.Time.now()
        scara_goal.point.x = x_goal
        scara_goal.point.y = y_goal
        scara_goal.point.z = z_goal
        
        
        err_abs_x = abs(self.scara_pos[0] - x_goal)
        err_abs_y = abs(self.scara_pos[1] - y_goal)
        err_abs_y = abs(self.scara_pos[2] - z_goal)
                
        while err_abs_x/x_goal > 0.5 and err_abs_y/y_goal > 0.5 and not rospy.is_shutdown():
            print('SCARA error en x: ', err_abs_x, 'error en y: ', err_abs_y)
            self.pub1.publish(scara_goal)
            err_abs_x = abs(self.scara_pos[0] - x_goal)
            err_abs_y = abs(self.scara_pos[1] - y_goal)
            err_abs_y = abs(self.scara_pos[2] - z_goal)
            
            self.rate.sleep()
            
            
    def m1_response_callback(self,response1):

        #rospy.logwarn("GOT Pos Response for M1="+str(response1.vector.x))
        self.scara_pos[0] = response1.vector.x
    
    def m2_response_callback(self,response2):

        #rospy.logwarn("GOT Pos Response for M2="+str(response2.vector.x))
        self.scara_pos[1] = response2.vector.x
    
    def m3_response_callback(self,response3):

        #rospy.logwarn("GOT Pos Response for M3 = "+str(response3.vector.x))
        self.scara_pos[2] = response3.vector.x
        
                

if __name__ == '__main__':
    rospy.init_node('coordinates_node', anonymous=True, log_level=rospy.WARN)
    auto_obj = Automatic()
    auto_obj.main()