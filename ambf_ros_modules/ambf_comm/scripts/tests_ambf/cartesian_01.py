#!/usr/bin/env python2.7
# Import the Client from ambf_client package
from ambf_client import Client
import time
import math
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from tf import transformations

def compute_T_matrix(obj_handle):
    rpy = obj_handle.get_rpy()
    R = transformations.euler_matrix(rpy[0], rpy[1], rpy[2], axes='sxyz')
    t = np.zeros((4, 4))
    pos = obj_handle.get_pos()
    tx = pos.x
    ty = pos.y
    tz = pos.z
    for i in range(0, 4):
        for j in range(0, 4):
            if j<3 and i<3:
                t[i,j] = 0
            if j==3 and i==0:
                t[i,j] = pos.x
            if j==3 and i==1:
                t[i,j] = pos.y
            if j==3 and i==2:
                t[i,j] = pos.z        
            if i==3:
                t[i,j] = 0
    T = R + t 
    return(T)

def hom_vec(pos):
    t = np.zeros(4)
    t[0] = pos.x
    t[1] = pos.y
    t[2] = pos.z
    t[3] = 1
    return(t)

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
psm_handle_pbackl = _client.get_obj_handle('psm/pitchbacklink')
psm_handle_pfl = _client.get_obj_handle('psm/pitchfrontlink')
psm_handle_ptl = _client.get_obj_handle('psm/pitchtoplink')
psm_handle_pbotl = _client.get_obj_handle('psm/pitchbottomlink')
psm_handle_pel = _client.get_obj_handle('psm/pitchendlink')
psm_handle_mi = _client.get_obj_handle('psm/maininsertionlink')
psm_handle_trl = _client.get_obj_handle('psm/toolrolllink')

psm_handle_tgl1 = _client.get_obj_handle('psm/toolgripper1link')
psm_handle_tpl = _client.get_obj_handle('psm/toolpitchlink')
psm_handle_tyl = _client.get_obj_handle('psm/toolyawlink')

link_handles = []
link_handles.append(psm_handle_trl)
link_handles.append(psm_handle_mi)
link_handles.append(psm_handle_pel)
link_handles.append(psm_handle_pbotl)
link_handles.append(psm_handle_pfl)
link_handles.append(psm_handle_yaw)
link_handles.append(psm_handle_base)

T_mat = []
for i in range(0,len(link_handles)):
    T = compute_T_matrix(link_handles[i])
    T_mat.append(T)

for i in range(0, len(T_mat)):
    if i == 0:
        T_tot = T_mat[i]
    else:
        T_tot = np.dot(T_tot, T_mat[i])
#print(T_tot)

pos_trl_raw = psm_handle_trl.get_pos()
pos_trl_local = hom_vec(pos_trl_raw)
print(pos_trl_local)
pos_trl_glob = np.dot(T_tot, pos_trl_local)
print('\n') 
print("POS BASE")
print(psm_handle_base.get_pose())
print("\n")
pos_yaw_raw = psm_handle_yaw.get_pose()
print("POS YAW")
print(psm_handle_yaw.get_pose())
print("\n")
print("POS PBackL")
print(psm_handle_pbackl.get_pose())
print("\n")
print("POS PFrontL")
print(psm_handle_pfl.get_pose())
print("\n")
print("POS MIl")
print(psm_handle_mi.get_pose())
print("\n")
print("POS TRL")
print(psm_handle_trl.get_pose())
print("\n")
#pos_yaw_local = hom_vec(pos_yaw_raw)
pos_local = [0, 0, 0, 1]
T1 = compute_T_matrix(psm_handle_yaw)
T2 = compute_T_matrix(psm_handle_pbackl)
T3 = compute_T_matrix(psm_handle_pfl)
T12 = np.dot(T1, T2)
T13 = np.dot(T12, T3)
#T1_inv = np.linalg.inv(T1_inv) 
#pos_yaw_global = np.dot(T1_inv, pos_yaw_local)
#pos_yaw_global = np.dot(T1, pos_yaw_local)
#print(pos_yaw_global)
#print(pos_trl_glob) 
#pos_bbackl_global = np.dot(T12, pos_local)
#print(pos_bbackl_global)
pos_pfl_global = np.dot(T13, pos_local)
print(pos_pfl_global)
print("MOVE")
time.sleep(2)
psm_handle_mi.set_pos(1.3858062843482137, -0.4012031646000947, 0.2727832775073985)
#psm_handle_trl.set_pos(1.6829794699541343, -0.27864292698008664, 0.289808949290927)
time.sleep(2)
print("MOVED")
print("POS MIl")
print(psm_handle_mi.get_pose())
print("\n")

raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()

