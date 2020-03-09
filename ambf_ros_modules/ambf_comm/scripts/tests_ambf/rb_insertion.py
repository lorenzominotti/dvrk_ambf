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
graph_fd = []
graph_d = []
graph_PID = []
graph_m = []
graph_freq = []
graph_frn = []

degree = 0
delta = 0.6 #cloth
#delta = 0.25 #cheese
delta_m = 0.0005
delta_degree = 0.025
force_old3 = 0
force_old2 = 0
force_old1 = 0
band = 0.05
band2 = 0.3 #cloth
#band2 = 0.08 #cheese
limit_mi = 0.30

k_corr = 0
time_now = 0
f_inv = 0.04


count_mi_loop = 0
P_value = 0
I_value = 0
D_value = 0
graph_Pval = 0
graph_Ival = 0
graph_Dval = 0

force_const = 4
Kd = 0
Integrator = 0
Derivator = 0
count = 0
window = []
window_size = 10
sum = 0
force_raw = []
graph_f = []
graph_m = []
vec_step = []
force1 = []
force_vect = []
error_force = []
error_pos = []



deltat_a = 0
time1 = []
deltat_a_ef = 0
time_ef = []

Integrator = 0
Derivator = 0
time_now = 0

flag_first_pos = True

'''
Kp = 0.01 #stiffer
Ki = 0.00005
#Kd = 0.00008
'''

Kp = 0.002
Ki = 0.00005

Integrator = 0
Derivator = 0
time_now = 0

force_const = 6
while m < limit_mi:

	time_start_a = time.time()
	force_raw_now = psm_handle_mi.get_force()
	#force = force_raw_now
	print(force_raw_now)
	average = (force_old2 + force_old1)/2
	#if(force > (average + delta)) or (force < (average - delta)):
		#force = force_old1
		#print('\n')
		#print('UNEXPECTED_PEAK..........COMPENSATION')
		#print('\n')

	count = count + 1
	if count < window_size + 1:
		window = np.append(window, force_raw_now)
		force = force_raw_now
	else:
		for i in range(1, window_size):
			window[i-1] = window[i]
			if i == (window_size - 1):
				window[i] = force_raw_now
			sum = sum + window[i-1]
		force = sum / window_size
		sum = 0

	if force > (force_const + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const + band)) and (force > (force_const - band)):
		count_mi_loop = count_mi_loop + 1
		print(count_mi_loop)
	if count_mi_loop == 3:
		break
	#m = m + delta_m
	psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_d = np.append(graph_d, degree)
	PID = 1
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
	graph_frn = np.append(graph_frn, force_raw_now)
	graph_f = np.append(graph_f, force)
	graph_fd = np.append(graph_fd, force_const)
	error_force = np.append(error_force, 0)
	time_end_a_ef = time.time()
	deltat_a_ef = (time_end_a_ef-time_start_a) + deltat_a_ef 
	time_ef = np.append(time_ef, deltat_a_ef)
	time_end_a = time.time()
	deltat_a = (time_end_a-time_start_a) + deltat_a 
	time1 = np.append(time1, deltat_a)
	time.sleep(f_inv)
print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

time.sleep(1)
force_const = 2
while m < limit_mi:

	time_start_a = time.time()
	force_raw_now = psm_handle_mi.get_force()
	#force = force_raw_now
	print(force_raw_now)
	average = (force_old2 + force_old1)/2
	#if(force > (average + delta)) or (force < (average - delta)):
		#force = force_old1
		#print('\n')
		#print('UNEXPECTED_PEAK..........COMPENSATION')
		#print('\n')

	count = count + 1
	if count < window_size + 1:
		window = np.append(window, force_raw_now)
		force = force_raw_now
	else:
		for i in range(1, window_size):
			window[i-1] = window[i]
			if i == (window_size - 1):
				window[i] = force_raw_now
			sum = sum + window[i-1]
		force = sum / window_size
		sum = 0

	if force > (force_const + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const + band)) and (force > (force_const - band)):
		count_mi_loop = count_mi_loop + 1
	if count_mi_loop == 5:
		break
	#m = m + delta_m
	psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_d = np.append(graph_d, degree)
	PID = 1
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
	graph_frn = np.append(graph_frn, force_raw_now)
	graph_f = np.append(graph_f, force)
	graph_fd = np.append(graph_fd, force_const)
	error_force = np.append(error_force, 0)
	time_end_a_ef = time.time()
	deltat_a_ef = (time_end_a_ef-time_start_a) + deltat_a_ef 
	time_ef = np.append(time_ef, deltat_a_ef)
	time_end_a = time.time()
	deltat_a = (time_end_a-time_start_a) + deltat_a 
	time1 = np.append(time1, deltat_a)
	time.sleep(f_inv)

time.sleep(1)
force_const = 6
while m < limit_mi:

	time_start_a = time.time()
	force_raw_now = psm_handle_mi.get_force()
	#force = force_raw_now
	print(force_raw_now)
	average = (force_old2 + force_old1)/2
	#if(force > (average + delta)) or (force < (average - delta)):
		#force = force_old1
		#print('\n')
		#print('UNEXPECTED_PEAK..........COMPENSATION')
		#print('\n')

	count = count + 1
	if count < window_size + 1:
		window = np.append(window, force_raw_now)
		force = force_raw_now
	else:
		for i in range(1, window_size):
			window[i-1] = window[i]
			if i == (window_size - 1):
				window[i] = force_raw_now
			sum = sum + window[i-1]
		force = sum / window_size
		sum = 0

	if force > (force_const + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const + band)) and (force > (force_const - band)):
		count_mi_loop = count_mi_loop + 1
	if count_mi_loop == 5:
		break
	#m = m + delta_m
	psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_d = np.append(graph_d, degree)
	PID = 1
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
	graph_frn = np.append(graph_frn, force_raw_now)
	graph_f = np.append(graph_f, force)
	graph_fd = np.append(graph_fd, force_const)
	error_force = np.append(error_force, 0)
	time_end_a_ef = time.time()
	deltat_a_ef = (time_end_a_ef-time_start_a) + deltat_a_ef 
	time_ef = np.append(time_ef, deltat_a_ef)
	time_end_a = time.time()
	deltat_a = (time_end_a-time_start_a) + deltat_a 
	time1 = np.append(time1, deltat_a)
	time.sleep(f_inv)

time.sleep(1)
force_const = 2
while m < limit_mi:

	time_start_a = time.time()
	force_raw_now = psm_handle_mi.get_force()
	#force = force_raw_now
	print(force_raw_now)
	average = (force_old2 + force_old1)/2
	#if(force > (average + delta)) or (force < (average - delta)):
		#force = force_old1
		#print('\n')
		#print('UNEXPECTED_PEAK..........COMPENSATION')
		#print('\n')

	count = count + 1
	if count < window_size + 1:
		window = np.append(window, force_raw_now)
		force = force_raw_now
	else:
		for i in range(1, window_size):
			window[i-1] = window[i]
			if i == (window_size - 1):
				window[i] = force_raw_now
			sum = sum + window[i-1]
		force = sum / window_size
		sum = 0

	if force > (force_const + band):
		m = m - delta_m/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
	    m = m + delta_m/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const + band)) and (force > (force_const - band)):
		count_mi_loop = count_mi_loop + 1
		print(count_mi_loop)
	if count_mi_loop == 5:
		break
	#m = m + delta_m
	psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_d = np.append(graph_d, degree)
	PID = 1
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
	graph_frn = np.append(graph_frn, force_raw_now)
	graph_f = np.append(graph_f, force)
	graph_fd = np.append(graph_fd, force_const)
	error_force = np.append(error_force, 0)
	time_end_a_ef = time.time()
	deltat_a_ef = (time_end_a_ef-time_start_a) + deltat_a_ef 
	time_ef = np.append(time_ef, deltat_a_ef)
	time_end_a = time.time()
	deltat_a = (time_end_a-time_start_a) + deltat_a 
	time1 = np.append(time1, deltat_a)
	time.sleep(f_inv)


time = []
time = time1
time_ef = []
time_ef = time_ef

fig, axs = plt.subplots(nrows = 2)
axs[0].plot(time, graph_f, color = 'r', label = "actual force")
axs[0].plot(time, graph_fd, color = 'b', label = "target force")
#axs[0].set(xlabel = 'Time [s]', ylabel = 'Force [N]')	
axs[0].set(ylabel = 'Force [N]')	
axs[0].legend(loc='best')
axs[0].grid()

axs[1].plot(time, error_force, color = 'r', label = "error")
axs[1].set(xlabel = 'Time [s]', ylabel = 'Force_error %')
axs[1].legend(loc='best')
axs[1].grid()
plt.show()



raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()

