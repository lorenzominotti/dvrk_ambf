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

# Let's sleep for a very brief moment to give the internal callbacks
# to sync up new data from the running simulator
time.sleep(0.2)

print('\n\n----')

raw_input("Number of joints of baseLink")
num_joints_base = psm_handle_base.get_num_joints()
print(num_joints_base)

raw_input("Name of joints of baseLink")
name_joints_base = psm_handle_base.get_joint_names()
print(name_joints_base)

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

m = 0.22
psm_handle_pel.set_joint_pos(0, m)
time.sleep(1)
#psm_handle
'''
psm_handle_pel.set_joint_pos(0, 0.16)
time.sleep(1)
'''
N = 1000
graph = []
degree = 0

#limit_mi = 0.20 cheese
limit_mi = 0.28

delta = 0.4
#delta = 0.6 cheese
delta_m = 0.00002
delta_degree = 0.20
force_old2 = 0
force_old1 = 0

while m < limit_mi:
	psm_handle_pel.set_joint_pos(0, m)
	m = m + delta_m
	force = psm_handle_mi.get_force()
	average = (force_old2 + force_old1)/2
	if(force > (average + delta)) or (force < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')
	force_old2 = force_old1
	force_old1 = force
	graph = np.append(graph, force)
	print(force)
	print("\n")
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])
'''
print('\n')
print('\n')
print('NOW CHECK THE FORCE IN A FIXED POSITION')
print('\n')
print('\n')
count = 0
while count < 1000:
	psm_handle_pel.set_joint_pos(0, m)
	force = psm_handle_mi.get_force()
	print(force)
	count = count + 1
	graph = np.append(graph, force)
'''
while degree < 30:
	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	force = psm_handle_mi.get_force()
	average = (force_old2 + force_old1)/2
	if(force > (average + delta)) or (force < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')
	force_old2 = force_old1
	force_old1 = force
	graph = np.append(graph, force)
	print(force)
	print("\n")
	degree = degree + delta_degree
	time.sleep(0.1)	
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])

while degree > (-30):
	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	force = psm_handle_mi.get_force()
	average = (force_old2 + force_old1)/2
	if(force > (average + delta)) or (force < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')
	force_old2 = force_old1
	force_old1 = force
	graph = np.append(graph, force)
	print(force)
	print("\n")
	degree = degree - delta_degree
	time.sleep(0.1)	
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])

count = 0
while count < 150:
	psm_handle_pel.set_joint_pos(0, m)
	force = psm_handle_mi.get_force()
	average = (force_old2 + force_old1)/2
	if(force > (average + delta)) or (force < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')
	force_old2 = force_old1
	force_old1 = force
	print(force)
	count = count + 1
	graph = np.append(graph, force)
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])
	

#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
#graph = np.append(graph,maF[5])

plt.plot(graph)
plt.show()

print('\n\n----')
raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()

