#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 15 20:48:52 2020

@author: gesem
"""

import rospy

from dynamic_reconfigure.server import Server
from scara_ppi_publisher.cfg import DParamsConfig

def callback(config, level):
    rospy.loginfo("Reconfigure Request: {kp}, {kpi}".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dparams", anonymous = False)
    srv = Server(DParamsConfig, callback)
    rospy.spin()
