#!/usr/bin/env python2.7
# Import the Client from ambf_client package
from ambf_client import Client
import time
import math
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt

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
psm_handle_pel = _client.get_obj_handle('psm/pitchendlink')
psm_handle_tgl1 = _client.get_obj_handle('psm/toolgripper1link')
psm_handle_tpl = _client.get_obj_handle('psm/toolpitchlink')
psm_handle_tyl = _client.get_obj_handle('psm/toolyawlink')


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
psm_handle_pel.set_joint_pos(0, 0.15)
time.sleep(15)
#psm_handle_pel.set_joint_pos(0, math.radians(40))
#time.sleep(1)
#psm_handle_tyl.set_joint_pos(1, math.radians(-20))
#time.sleep(1)

start_pos = psm_handle_pel.get_all_joint_pos()

graph = []
N = 10000
counter = 0
window = 10
sum0 = 0
sum = 0
m = 0.10

'''
while m < 0.25:
	psm_handle_pel.set_joint_pos(0, m)
	print(psm_handle_pel.get_all_joint_pos())
	m = m +0.0005
	force = psm_handle_mi.get_force()
	print(force)
	print("\n")
	"""if counter < window:
		forceVect[counter] = psm_handle_tgl1.get_force()
		sum0 = sum0 + forceVect[counter]
		if counter == 0:
			counter = 1
		maF = sum0/counter
	else:
		for i in range(window-1,0,-1):
			forceVect[i] = forceVect[i-1]
			forceVect[0] = psm_handle_tgl1.get_force()
        for i in range(0,window):
            sum = sum + forceVect[i]
	maF = sum/window
	sum = 0
	sum0 = 0
	counter = counter + 1

    """
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[10])
	graph = np.append(graph, force)
	#print(maF)
	print('\n\n')
	time.sleep(0.2)

plt.plot(graph)
plt.show()
'''
#for cm in range(0, 10):
        #print("movement")
	#psm_handle_pel.set_joint_pos(0, cm/100)
	#time.sleep(0.1)

raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()

