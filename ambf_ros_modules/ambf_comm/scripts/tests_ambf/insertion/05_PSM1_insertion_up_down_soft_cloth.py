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
time.sleep(5)
#m = 0.15 #cheese
m = 0.08
psm_handle_pel.set_joint_pos(0, m)
time.sleep(1)
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
#psm_handle_pel.set_joint_pos(0, math.radians(40))
#time.sleep(1)
#psm_handle_tyl.set_joint_pos(1, math.radians(-20))
#time.sleep(1)
graph_f = []
graph_d = []
graph_PID = []
graph_m = []

degree = 0
delta = 0.6 #cloth
#delta = 0.25 #cheese
delta_m = 0.00005
delta_degree = 0.05
force_old3 = 0
force_old2 = 0
force_old1 = 0
band = 0.05
band2 = 0.3 #cloth
#band2 = 0.08 #cheese
limit_mi = 0.30
limit_mi_2 = 0.09

k_corr = 0


count_mi_loop = 0
P_value = 0
I_value = 0
D_value = 0
graph_Pval = 0
graph_Ival = 0
graph_Dval = 0

#force_const_max = 5
force_const_max = 2
force_const_min = 0.5
Kp = 0.005
Ki = 0.0005
#Kd = 0.00005
Integrator = 0
Derivator = 0
time.sleep(2)
while m < limit_mi:

	force = psm_handle_mi.get_force()
	print(force)
	average = (force_old2 + force_old1)/2
	if(force > (average + delta)) or (force < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')

	if force > (force_const_max + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const_max - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const_max + band)) and (force > (force_const_max - band)):
		count_mi_loop = count_mi_loop + 1
	if count_mi_loop == 300:
		count_mi_loop = 0
		break
	#m = m + delta_m
	psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)
	PID = 1
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
	#print(force)
	#print("\n")
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])

time.sleep(1)
while m > limit_mi_2:
	force = psm_handle_mi.get_force()
	print(force)
	average = (force_old2 + force_old1)/2
	if(force > (average + delta)) or (force < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')

	if force > (force_const_min + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const_min - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const_min + band)) and (force > (force_const_min - band)):
		count_mi_loop = count_mi_loop + 1
	if count_mi_loop == 200:
		count_mi_loop = 0
		break
	#m = m + delta_m
	#psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)
	PID = 1
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)

time.sleep(1)
while m < limit_mi:

	force = psm_handle_mi.get_force()
	print(force)
	average = (force_old2 + force_old1)/2
	if(force > (average + delta)) or (force < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')

	if force > (force_const_max + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const_max - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const_max + band)) and (force > (force_const_max - band)):
		count_mi_loop = count_mi_loop + 1
	if count_mi_loop == 200:
		count_mi_loop = 0
		break
	#m = m + delta_m
	psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)
	PID = 1
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
	#print(force)
	#print("\n")
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])
print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')

time.sleep(1)
limit_mi_2 = 0.09
while m > limit_mi_2:
	force = psm_handle_mi.get_force()
	print(force)
	#average = (force_old2 + force_old1)/2
	#if(force > (average + delta)) or (force < (average - delta)):
		#force = force_old1
		#print('\n')
		#print('UNEXPECTED_PEAK..........COMPENSATION')
		#print('\n')

	if force > (force_const_min + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const_min - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const_min + band)) and (force > (force_const_min - band)):
		count_mi_loop = count_mi_loop + 1
	if count_mi_loop == 200:
		count_mi_loop = 0
		break
	#m = m + delta_m
	#psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)
	PID = 1
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
time.sleep(1)

while m < limit_mi:

	force = psm_handle_mi.get_force()
	print(force)
	average = (force_old2 + force_old1)/2
	if(force > (average + delta)) or (force < (average - delta)):
		force = force_old1
		print('\n')
		print('UNEXPECTED_PEAK..........COMPENSATION')
		print('\n')

	if force > (force_const_max + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const_max - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const_max + band)) and (force > (force_const_max - band)):
		count_mi_loop = count_mi_loop + 1
	if count_mi_loop == 200:
		count_mi_loop = 0
		break
	#m = m + delta_m
	psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)
	PID = 1
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
	#print(force)
	#print("\n")
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])
print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
time.sleep(1)

limit_mi_2 = 0.09
while m > limit_mi_2:
	force = psm_handle_mi.get_force()
	print(force)
	#average = (force_old2 + force_old1)/2
	#if(force > (average + delta)) or (force < (average - delta)):
		#force = force_old1
		#print('\n')
		#print('UNEXPECTED_PEAK..........COMPENSATION')
		#print('\n')

	if force > (force_const_min + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const_min - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const_min + band)) and (force > (force_const_min - band)):
		count_mi_loop = count_mi_loop + 1
	if count_mi_loop == 200:
		count_mi_loop = 0
		break
	#m = m + delta_m
	#psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)
	PID = 1
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
	


'''
plt.plot(graph_m)
plt.grid()
plt.show()
'''
time.sleep(5)
plt.plot(graph_f)
#plt.plot(graph_d, color = 'r')
#plt.plot(graph_PID, color = 'g')
plt.grid()
plt.show()
'''
plt.plot(graph_PID, color = 'g')
plt.plot(graph_Pval, color = 'b')
plt.plot(graph_Ival, color = 'r')
plt.plot(graph_Dval, color = 'y')
plt.grid()
plt.show()
'''

raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()
