#!/usr/bin/env python2.7
# Import the Client from ambf_client package
from ambf_client import Client
import time
import math
import rospy
from tf import transformations
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
psm_handle_trl = _client.get_obj_handle('psm/toolrolllink')

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

#m = 0.22 #cloth
m = 0.18 #cheese
psm_handle_pel.set_joint_pos(0, m)
time.sleep(1)
#psm_handle

#count = 0
#while count < 500:
	#print(psm_handle_base.get_rpy())

quat_base = psm_handle_base.get_rot()
quat_trl = psm_handle_trl.get_rot()

#q' = quat_trl-1 * quat_base
q_base_inv = transformations.quaternion_inverse([quat_base.x, quat_base.y, quat_base.z, quat_base.w])
q_rot = transformations.quaternion_multiply(q_base_inv, [quat_trl.x, quat_trl.y, quat_trl.z, quat_trl.w])
R_base_trl = transformations.quaternion_matrix(q_rot)

N = 1000
graph_f = []
graph_d = []
degree = 0

#limit_mi = 0.195 #cheese
limit_mi = 0.60 #cloth

delta = 0.4 #cloth
#delta = 0.25 #cheese
delta_m = 0.00001
delta_m1 = 0.0001
delta_degree = 0.250
force_old3 = 0
force_old2 = 0
force_old1 = 0
band = 0.05
#band2 = 0.4 #cloth
band2 = 0.08 #cheese

k_corr = 0


count_mi_loop = 0

force_const = 2
force_const_vect_to_transpose = np.array([[0, 0, force_const, 1]])
force_const_vect = force_const_vect_to_transpose.transpose()

#force_const_vect = np.array([0, 0, force_const, 1])
#print(force_const_vect)

while m < limit_mi:

	force = psm_handle_mi.get_force()
	print(force)
	average = (force_old2 + force_old1)/2
	if(force > (average + delta)) or (force < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')
	
	if (force < (force_const + band)) and (force > (force_const - band)):
		count_mi_loop = count_mi_loop + 1
		m = m - delta_m
	if count_mi_loop == 250:
		break
	
	m = m + delta_m
	psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)

print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')

while degree < 45:

	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	force = psm_handle_mi.get_force()
	#print(force)
	
	'''
	if abs(force-force_const) > 1.5:
		k_corr = 4
	if abs(force-force_const) > 1 and abs(force-force_const) < 1.5:
		k_corr = 3
	if abs(force-force_const) > 0.8 and abs(force-force_const) < 1:
		k_corr = 2
	if abs(force-force_const) > 0.6 and abs(force-force_const) < 0.8:
		k_corr = 1

	if force > (force_const + band2):
		m = m - k_corr*delta_m
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
		m = m + k_corr*delta_m
		psm_handle_pel.set_joint_pos(0, m)
	'''	

	quat_base = psm_handle_base.get_rot()
	quat_trl = psm_handle_trl.get_rot()
	q_base_inv = transformations.quaternion_inverse([quat_base.x, quat_base.y, quat_base.z, quat_base.w])
	q_rot = transformations.quaternion_multiply(q_base_inv, [quat_trl.x, quat_trl.y, quat_trl.z, quat_trl.w])
	R_base_trl = transformations.quaternion_matrix(q_rot)

	force_to_set_to_convert = R_base_trl * force_const_vect
	force_to_set = np.array([force_to_set_to_convert[2,0], force_to_set_to_convert[2,1], force_to_set_to_convert[2,2]])

	if (force < (force_const + band2)) and (force > (force_const - band2)):
		degree = degree + delta_degree
		print(force)

	if force > (force_const + band2):
		m = m - delta_m1
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
		m = m + delta_m1
		psm_handle_pel.set_joint_pos(0, m)

	psm_handle_trl.set_force(force_to_set[0], force_to_set[1], force_to_set[2])
	#degree = degree + 1


	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	#graph_d = np.append(graph_d, degree)
	#print(force)
	#print("\n")
	#degree = degree + delta_degree
	time.sleep(0.05)	
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])

plt.plot(graph_f)
#plt.plot(graph_d, color = 'r')
plt.grid()
plt.show()







raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()