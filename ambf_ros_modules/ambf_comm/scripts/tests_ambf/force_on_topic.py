#!/usr/bin/env python2.7
# Import the Client from ambf_client package
from ambf_client import Client
import time
import math
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from ambf_msgs.msg import ObjectState, ObjectCmd, WorldState, WorldCmd
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
fz = 0
force = fz
rospy.init_node('pub_f', anonymous=True)
pub = rospy.Publisher('pub_f', Float64, queue_size=10) 

def fun_callback(ObjectState):
    global fz 
    fz = ObjectState.wrench.force.z
    #set_force()
    #print(fz)
    
    
def get_force():
    #global fz
    #print('wellaaaaa1111')
    rospy.Subscriber("/ambf/env/psm/maininsertionlink/State", ObjectState, fun_callback)
    #print('wellaaaaa2222')
    pub.publish(fz)
    #print('lurido')
    print(fz)
    rospy.spin()

def set_force():
    global fz
    #pub = rospy.Publisher('pub_f', Float64, queue_size=10)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        force = fz
        print('lurido')
        print(force)
        pub.publish(force)
        rate.sleep()
     

if __name__ == '__main__':
    get_force()
    
   
     


