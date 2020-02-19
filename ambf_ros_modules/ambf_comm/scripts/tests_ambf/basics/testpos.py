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
import numpy as np
from scipy.signal import butter,filtfilt
from geometry_msgs.msg import Pose

# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bi-directional communication
_client.connect()

print('\n\n----')
raw_input("We can see what objects the client has found. Press Enter to continue...")
# You can print the names of objects found. We should see all the links found
print(_client.get_obj_names())

# Lets get a handle to PSM and ECM, as we can see in the printed
# object names, 'ecm/baselink' and 'psm/baselink' should exist
psm_handle_base = _client.get_obj_handle('psm/baselink')
psm_handle_yaw = _client.get_obj_handle('psm/yawlink')
psm_handle_mi = _client.get_obj_handle('psm/maininsertionlink')
psm_handle_pfl = _client.get_obj_handle('psm/pitchfrontlink')
psm_handle_pbl = _client.get_obj_handle('psm/pitchbottomlink')
psm_handle_pel = _client.get_obj_handle('psm/pitchendlink')
psm_handle_tgl1 = _client.get_obj_handle('psm/toolgripper1link')
psm_handle_tpl = _client.get_obj_handle('psm/toolpitchlink')
psm_handle_tyl = _client.get_obj_handle('psm/toolyawlink')
psm_handle_trl = _client.get_obj_handle('psm/toolrolllink')






# Let's sleep for a very brief moment to give the internal callbacks
# to sync up new data from the running simulator
time.sleep(0.2)

print('\n\n----')

raw_input("Number of joints of mainInsertionLink")
num_joints_mi = psm_handle_mi.get_num_joints()
print(num_joints_mi)

raw_input("Name of joints of mainInsertionLink")
name_joints_mi = psm_handle_mi.get_joint_names()
print(name_joints_mi)

raw_input("Number of joints of pitchEndLink")
num_joints_pel = psm_handle_pel.get_num_joints()
print(num_joints_pel)

raw_input("Name of joints of pitchEndLink")
name_joints_pel = psm_handle_pel.get_joint_names()
print(name_joints_pel)

raw_input("Name of joints of base")
name_joints_base = psm_handle_base.get_joint_names()
print(name_joints_base)

raw_input("Display movement...")
#psm_handle_pfl.set_joint_pos(0, math.radians(-60))
psm_handle_pfl.set_joint_pos(0, 0)
psm_handle_base.set_joint_pos(0, math.radians(0))
time.sleep(2)


#psm_handle_mi.set_joint_pos(0, 0)
psm_handle_pel.set_joint_pos(0, 0)
time.sleep(1)
#psm_handle_pel.set_joint_pos(0, 0.1)
#time.sleep(1)
psm_handle_pel.set_joint_pos(0, 0)
time.sleep(1)
m = 0.18
psm_handle_pel.set_joint_pos(0, m)
time.sleep(1)
print(psm_handle_trl.get_pos())

psm_handle_pel.set_joint_pos(0, math.radians(0))
#psm_handle_tyl.set_joint_pos(1, math.radians(-20))
#time.sleep(1)
print("11111111111111\n")
time.sleep(2)
pose = Pose()
pose.position.x = -0.05067549144346678
pose.position.y = 0.00035648299651597215
pose.position.z = 0.0209319057460326352
psm_handle_mi.set_pose(pose)
print("22222222222222")
time.sleep(5)
print("33333333333333")
raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()

