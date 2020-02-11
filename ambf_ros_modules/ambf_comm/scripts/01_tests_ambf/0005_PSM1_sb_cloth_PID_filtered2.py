#!/usr/bin/env python2.7
# Import the Client from ambf_client package
from ambf_client import Client
import time
import math
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
#from lib import fnlib
#import scipy.signal
from scipy import zeros, signal

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
time.sleep(2)
#psm_handle_pel.set_joint_pos(0, 0.1)
#time.sleep(1)
psm_handle_pel.set_joint_pos(0, 0)
time.sleep(1)
m = 0.10
psm_handle_pel.set_joint_pos(0, m)
time.sleep(1)
#psm_handle_pel.set_joint_pos(0, math.radians(40))
#time.sleep(1)
#psm_handle_tyl.set_joint_pos(1, math.radians(-20))
#time.sleep(1)
graph_f = []
graph_d = []
graph_PID = []
graph_m = []
graph_frn = []
force_raw = []

degree = 0
delta = 0.6 #cloth
#delta = 0.25 #cheese
delta_m = 0.00005
delta_degree = 0.1
force_old3 = 0
force_old2 = 0
force_old1 = 0
band = 0.05
band2 = 0.3 #cloth
#band2 = 0.08 #cheese
limit_mi = 0.30

k_corr = 0
f_inv = 0.01

count_mi_loop = 0
P_value = 0
I_value = 0
D_value = 0
graph_Pval = 0
graph_Ival = 0
graph_Dval = 0

force_const = 3
Kp = 0.003
Ki = 0.00001
#Kd = 0.00005
Integrator = 0
Derivator = 0

while m < limit_mi:

	force_raw_now = psm_handle_mi.get_force()
	force = force_raw_now
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
	if count_mi_loop == 100:
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
	graph_frn = np.append(graph_frn, force_raw_now)
	#print(force)
	#print("\n")
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])
print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')


count = 0
window = []
window_size = 10
sum = 0
while degree < 20:

	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	force_raw_now = psm_handle_mi.get_force()
	#print(force)
	#force_read = psm_handle_mi.get_force()
	#average = (force_old2 + force_old1)/2
	#if force_old2 != force_old1 and force_old2 != force_read:
	#if(force_read > (average + delta)) or (force_read < (average - delta)):
		#force = force_old1
		#print('\n')
		#print('UNEXPECTED_PEAK..........COMPENSATION')
		#print('\n')
	#else:
		#force = force_read

	force_raw = np.append(force_raw, force_raw_now)
	b = signal.firwin(5, 0.000001)
	z = signal.lfilter_zi(b, 1)
	force = zeros(force_raw.size)
	for i, x in enumerate(force_raw):
		force[i], z = signal.lfilter(b, 1, [x], zi=z)
    	#force[i], z = signal.lfilter(b, 1, [x], zi=z)
    #print('LERCIOOOOOOO')
	#print('LERCIOOOOO')
	print(force[-1])

	error = force_const - force[-1]
	P_value = (Kp * error)
	
	#D_value = Kd * (error - Derivator)
	#Derivator = error
	
	Integrator = Integrator + error

	#if Integrator > Integrator_max:
		#Integrator = Integrator_max
	#elif Integrator < Integrator_min:
		#Integrator = Integrator_min
	I_value = Integrator * Ki
	
	
	PID = P_value + I_value #+ D_value
	#if force > (force_const + band):
	m = m + PID*m
    #psm_handle_pel.set_joint_pos(0, m)
	#if force < (force_const - band):
	    #m = m - PID * m
	psm_handle_pel.set_joint_pos(0, m)
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
	#if (force < (force_const + band2)) and (force > (force_const - band2)):
		#degree = degree + delta_degree
	degree = degree + delta_degree

	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force[-1])
	graph_d = np.append(graph_d, degree)
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
	graph_frn = np.append(graph_frn, force_raw_now)
	#print(force)
	#print("\n")
	#degree = degree + delta_degree
	time.sleep(f_inv)	
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])

'''
count = 0
window_size = 10
sum = 0
while degree > (0):

	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	#force_read = psm_handle_mi.get_force()
	#average = (force_old3 + force_old2 + force_old1)/3
	#if force_old2 != force_old1 and force_old2 != force_read:
	#if(force_read > (average + delta)) or (force_read < (average - delta)):
		#force = force_old1
		#print('\n')
		#print('UNEXPECTED_PEAK..........COMPENSATION')
		#print('\n')
	#else:
		#force = force_read

	force_raw_now = psm_handle_mi.get_force()

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

	error = force_const - force
	P_value = (Kp * error)

	#D_value = Kd * (error - Derivator)
	#Derivator = error

	Integrator = Integrator + error

	I_value = Integrator * Ki

	PID = P_value + I_value + D_value

	m = m + PID*m

	psm_handle_pel.set_joint_pos(0, m)

	#if (force < (force_const + band2)) and (force > (force_const - band2)):
    #if (force < (force_const + band2)) and (force > (force_const - band2)):
		#degree = degree - delta_degree
	degree = degree - delta_degree

	force_old3 = force_old2
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	graph_d = np.append(graph_d, degree)
	graph_PID = np.append(graph_PID, PID)
	graph_Pval = np.append(graph_Pval, P_value)
	graph_Ival = np.append(graph_Ival, I_value)
	graph_Dval = np.append(graph_Dval, D_value)
	graph_m = np.append(graph_m, m)
	graph_frn = np.append(graph_frn, force_raw_now)

	print(force)
	#print(degree)
	#print(m)
	#print("\n")
	time.sleep(f_inv)	
	#maF = np.convolve(force, np.ones((N,))/N, mode='valid')*10000
	#graph = np.append(graph,maF[5])
'''
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
	if abs(force-force_const) > 0.6 and abs(force-force_const) < 0.8:
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


while degree > (-20):

	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	force_read = psm_handle_mi.get_force()
	average = (force_old3 + force_old2 + force_old1)/3
	if force_old2 != force_old1 and force_old2 != force_read:
		if(force_read > (average + delta)) or (force_read < (average - delta)):
			force = force_old1
			print('\n')
			print('UNEXPECTED_PEAK..........COMPENSATION')
			print('\n')
	else:
		force = force_read


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
'''
plt.plot(graph_m)
plt.grid()
plt.show()
'''
plt.plot(graph_frn)
plt.plot(graph_f, color = 'r')
#plt.plot(graph_d, color = 'r')
#plt.plot(graph_PID, color = 'g')
plt.grid()
plt.show()

plt.plot(graph_d, color = 'r')
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

