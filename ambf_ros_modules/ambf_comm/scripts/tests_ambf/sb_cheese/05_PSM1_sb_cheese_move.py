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
m = 0.17
psm_handle_pel.set_joint_pos(0, m)
time.sleep(1)
#psm_handle_pel.set_joint_pos(0, math.radians(40))
#time.sleep(1)
#psm_handle_tyl.set_joint_pos(1, math.radians(-20))
#time.sleep(1)
graph_f = []
graph_d = []

degree = 0
#delta = 0.6 #cloth
delta = 0.25 #cheese
delta_m = 0.00005
delta_degree = 0.250
force_old3 = 0
force_old2 = 0
force_old1 = 0
band = 0.05
#band2 = 0.6 #cloth
band2 = 0.5 #cheese
limit_mi = 0.30

k_corr = 0


count_mi_loop = 0

force_const = 2.0

while m < limit_mi:

	force = psm_handle_mi.get_force()
	print(force)
	average = (force_old2 + force_old1)/2
	if(force > (average + delta)) or (force < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')

	if force > (force_const + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const + band)) and (force > (force_const - band)):
		count_mi_loop = count_mi_loop + 1
	if count_mi_loop == 250:
		break
	#m = m + delta_m
	psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)
	#print(force)
	#print("\n")
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])


while degree < 20:

	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	force = psm_handle_mi.get_force()
	print(force)
	force_read = psm_handle_mi.get_force()
	average = (force_old3 + force_old2 + force_old1)/3
	if force_old2 != force_old1 and force_old2 != force_read and force_old2 != force_old3:
		if(force_read > (average + delta)) or (force_read < (average - delta)):
			force = force_old1
			print('\n')
			print('UNEXPECTED_PEAK..........COMPENSATION')
			print('\n')
	else:
		force = force_read

	if abs(force-force_const) > 1.5:
		k_corr = 8
	if abs(force-force_const) > 1 and abs(force-force_const) < 1.5:
		k_corr = 6
	if abs(force-force_const) > 0.8 and abs(force-force_const) < 1:
		k_corr = 4
	if abs(force-force_const) > 0.5 and abs(force-force_const) < 0.8:
		k_corr = 2

	if force > (force_const + band2):
		m = m - k_corr*delta_m
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
		m = m + k_corr*delta_m
		psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const + band2)) and (force > (force_const - band2)):
		degree = degree + delta_degree

	force_old3 = force_old2
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)
	#print(force)
	#print("\n")
	#degree = degree + delta_degree
	time.sleep(0.1)	
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])

while degree > (-20):

	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	force_read = psm_handle_mi.get_force()
	average = (force_old3 + force_old2 + force_old1)/3
	if force_old2 != force_old1 and force_old2 != force_read and force_old2 != force_old3:
		if(force_read > (average + delta)) or (force_read < (average - delta)):
			force = force_old1
			print('\n')
			print('UNEXPECTED_PEAK..........COMPENSATION')
			print('\n')
	else:
		force = force_read


	if abs(force-force_const) > 1.5:
		k_corr = 8
	if abs(force-force_const) > 1 and abs(force-force_const) < 1.5:
		k_corr = 6
	if abs(force-force_const) > 0.8 and abs(force-force_const) < 1:
		k_corr = 4
	if abs(force-force_const) > 0.5 and abs(force-force_const) < 0.8:
		k_corr = 2

	if force > (force_const + band2):
		m = m - k_corr*delta_m
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
		m = m + k_corr*delta_m
		psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const + band2)) and (force > (force_const - band2)):
		degree = degree - delta_degree

	force_old3 = force_old2
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)

	print(force)
	#print(degree)
	#print(m)
	#print("\n")
	time.sleep(0.1)	
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])
'''
while degree < 20:

	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	force = psm_handle_mi.get_force()
	print(force)
	force_read = psm_handle_mi.get_force()
	average = (force_old2 + force_old1)/2
	#if force_old2 != force_old1 and force_old2 != force_read:
	if(force_read > (average + delta)) or (force_read < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')
	#else:
		#force = force_read

	if abs(force-force_const) > 1.5:
		k_corr = 4
	if abs(force-force_const) > 1 and abs(force-force_const) < 1.5:
		k_corr = 3
	if abs(force-force_const) > 0.8 and abs(force-force_const) < 1:
		k_corr = 2
	if abs(force-force_const) > 0.5 and abs(force-force_const) < 0.8:
		k_corr = 1

	if force > (force_const + band2):
		m = m - k_corr*delta_m
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
		m = m + k_corr*delta_m
		psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const + band2)) and (force > (force_const - band2)):
		degree = degree + delta_degree

	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)
	#print(force)
	#print("\n")
	#degree = degree + delta_degree
	time.sleep(0.1)	
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])

'''

plt.plot(graph_f)
plt.plot(graph_d, color = 'r')
plt.grid()
plt.show()



raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()

