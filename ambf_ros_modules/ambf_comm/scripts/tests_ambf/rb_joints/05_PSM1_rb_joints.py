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



def reach_pos_XY(goal_x, goal_y, start):
	global Kp, Ki
	global m
	global f_inv
	global Integrator
	global graph_f
	global graph_posX
	global graph_posY
	global posx_start0
	global posy_start0
	global posz_start0
	global degree_base
	global degree_pfl
	global vec_step
	global count_step
	global force
	global force_vec
	global prevSignal
	

	if start == True:
		pos_start = psm_handle_trl.get_pos()
		posx_start =  pos_start.x
		posy_start =  pos_start.y
		posz_start =  pos_start.z
		posx_start0 = posx_start
		posy_start0 = posy_start
		posz_start0 = posz_start
		target_x = posx_start + goal_x
		target_y = posy_start + goal_y
		degree_base = 0
		degree_pfl = 0
		count_step = 0
	if start == False:
		pos_start = psm_handle_trl.get_pos()
		posx_start =  pos_start.x
		posy_start =  pos_start.y
		target_x = posx_start0 + goal_x
		target_y = posy_start0 + goal_y

	path_x = abs(target_x - posx_start)
	path_y = abs(target_y - posy_start)

	if path_x >= path_y:
		path_long = path_x
		path_short = path_y
	if path_x < path_y:
		path_long = path_y
		path_short = path_x

	d_degree_long = 0.1
	d_degree_short = (path_short / path_long) * d_degree_long
	if path_x >= path_y:
		dx = d_degree_long
		dy = d_degree_short
	if path_x < path_y:
		dx = d_degree_short
		dy = d_degree_long

	#n_steps = 100*path_long / d_degree_long 

	count = 0
	window = []
	window_size = 10
	sum = 0
	count1 = 0
	

	stop_x = False
	stop_y = False
	
	while (stop_x == False) or (stop_y == False):

		#print(step)
		pos_tool = psm_handle_trl.get_pos()
		px = pos_tool.x
		py = pos_tool.y
		pz = pos_tool.z
		force_old1 = force
		force_raw_now = psm_handle_mi.get_force()
		#print(force_raw_now)

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
		
		

		'''
		force1 = np.append(force1, force2)
		nsamps = len(force1)
		samp_rate = 100
		#x = np.interp(vec_step, len(graph_f), graph_f)
		x = force1
		#xfreq = np.fft.fft(x)
		#fft_freqs = np.fft.fftfreq(nsamps, d=1./samp_rate)
		#plt.loglog(fft_freqs[0:nsamps/2], np.abs(xfreq)[0:nsamps/2])
		#plt.title('Filter Input - Frequency Domain')
		#plt.grid(True)
		#plt.show()

		cuttoff_freq = 1
		norm_pass = cuttoff_freq/(samp_rate/2)
		norm_stop = 1.5*norm_pass
		#(N, Wn) = signal.buttord(wp=norm_pass, ws=norm_stop, gpass=2, gstop=30, analog=0)
		(b, a) = signal.butter(4, 0.1, btype='low', analog=0, output='ba')

		#zi = signal.lfiltic(b, a, x[0:5], x[0:5])
		#(y, zi) = signal.lfilter(b, a, x, zi=zi)
		force_vec = signal.lfilter(b, a, x)
		force = force_vec[-1]
		
		###########################################################################################
		
		count1 = count1 + 1
		if count1 < window_size + 1:
			window = np.append(window, force2)
			force = force2
		else:
			for i in range(1, window_size):
				window[i-1] = window[i]
				if i == (window_size - 1):
					window[i] = force2
				sum = sum + window[i-1]
			force = sum / window_size
			sum = 0
		'''
		
		###########################################################################################

		
		error = force_const - force
		P_value = (Kp * error)
	
		#D_value = Kd * (error - Derivator)
		#Derivator = error
	
		Integrator = Integrator + error
		I_value = Integrator * Ki
	
	
		PID = P_value + I_value #+ D_value

		m = m + PID*m
    	
		psm_handle_pel.set_joint_pos(0, m)
	
		print(px-posx_start0, py-posy_start0, pz-posz_start0)
		if target_x >= posx_start:
			if target_x > px:
				degree_base = degree_base - dx
				psm_handle_base.set_joint_pos(0, math.radians(degree_base))
			else:
				stop_x = True
		if target_x < posx_start:
			if target_x < px:
				degree_base = degree_base + dx
				psm_handle_base.set_joint_pos(0, math.radians(degree_base))
			else:
				stop_x = True

		if target_y >= posy_start:
			if target_y > py:
				degree_pfl = degree_pfl + dy
				psm_handle_pfl.set_joint_pos(0, math.radians(degree_pfl))
			else:
				stop_y = True
		if target_y < posy_start:
			if target_y < py:
				degree_pfl = degree_pfl - dy
				psm_handle_pfl.set_joint_pos(0, math.radians(degree_pfl))
			else:
				stop_y = True

		graph_f = np.append(graph_f, force)
		'''
		graph_d = np.append(graph_d, degree)
		graph_PID = np.append(graph_PID, PID)
		graph_Pval = np.append(graph_Pval, P_value)
		graph_Ival = np.append(graph_Ival, I_value)
		graph_frn = np.append(graph_frn, force_raw_now)
		graph_m = np.append(graph_m, m)
		'''
		pos = psm_handle_trl.get_pos()
		#posZ =  pos.z
		#graph_posZ = np.append(graph_posZ, posZ)
		posX =  pos.x
		graph_posX = np.append(graph_posX, posX)
		posY =  pos.y
		graph_posY = np.append(graph_posY, posY)
		time.sleep(f_inv)
		count_step = count_step + 1
		vec_step = np.append(vec_step, count_step)	
		


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
m = 0.16
psm_handle_pel.set_joint_pos(0, m)
time.sleep(1)
print(psm_handle_trl.get_pos())

psm_handle_pel.set_joint_pos(0, math.radians(0))
#psm_handle_tyl.set_joint_pos(1, math.radians(-20))
#time.sleep(1)
graph_f = []
graph_d = []
graph_PID = []
graph_m = []
force_raw = []
force_old2 = 0
force_old1 = 0
vec_step = []
force1 = []
force_vec = []

degree = 0
delta = 0.6 
delta_m = 0.00005
delta_m_start = 0.0001
band = 0.03
limit_mi = 0.30

f_inv = 0.01

count_mi_loop = 0
P_value = 0
I_value = 0
D_value = 0
graph_Pval = 0
graph_Ival = 0
graph_Dval = 0
graph_frn = []
graph_posZ = []
graph_posX = []
graph_posY = []
posx_start0 = 0
posy_start0 = 0

force_const = 6


#circle:
Kp = 0.002
Ki = 0.0001

'''
Kp = 0.0005
#Ki = 0.0001
Ki = 0.00001
Kd = 0.00008
#Ki = 0.0001
#Kd = 0.00005
'''
Integrator = 0
Derivator = 0
time_now = 0

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
		m = m - delta_m_start/2
		psm_handle_pel.set_joint_pos(0, m)
	if force < (force_const - band):
	    m = m + delta_m_start/2
	    psm_handle_pel.set_joint_pos(0, m)

	if (force < (force_const + band)) and (force > (force_const - band)):
		count_mi_loop = count_mi_loop + 1
	if count_mi_loop == 10:
		break
	
	psm_handle_pel.set_joint_pos(0, m)
	force_old2 = force_old1
	force_old1 = force
	graph_f = np.append(graph_f, force)
	PID = 1
	graph_frn = np.append(graph_frn, force_raw_now)
	graph_m = np.append(graph_m, m)
	pos = psm_handle_trl.get_pos()
	posZ =  pos.z
	graph_posZ = np.append(graph_posZ, posZ)
	posX =  pos.x
	posY =  pos.y

print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')

#reach_pos_XY(-0.07, 0.05, True)
reach_pos_XY(-0.05, 0.03, True)
print('STEP1')
time.sleep(2)
reach_pos_XY(0.05, 0.06, False)
#print('STEP2')
#time.sleep(2)
reach_pos_XY(0.00, -0.02, False)
print('STEP3')
time.sleep(5)
'''
reach_pos_XY(-0.05, 0.0, True)
print('STEP1')
time.sleep(0.5)

reach_pos_XY(-0.03, 0.04, False)
print('STEP2')
time.sleep(0.5)
reach_pos_XY(-0.02, 0.0458, False)
print('STEP3')
time.sleep(0.5)

reach_pos_XY(0.0, 0.05, False)
print('STEP4')
time.sleep(0.5)

reach_pos_XY(0.02, 0.0458, False)
print('STEP5')
time.sleep(0.5)
reach_pos_XY(0.03, 0.04, False)
print('STEP6')
time.sleep(0.5)

reach_pos_XY(0.05, 0.0, False)
print('STEP7')
time.sleep(0.5)

reach_pos_XY(0.03, -0.04, False)
print('STEP8')
time.sleep(0.5)
reach_pos_XY(0.02, -0.0458, False)
print('STEP9')
time.sleep(0.5)

reach_pos_XY(0.0, -0.05, False)
print('STEP10')
time.sleep(0.5)

reach_pos_XY(-0.02, -0.0458, False)
print('STEP11')
time.sleep(0.5)
reach_pos_XY(-0.03, -0.04, False)
print('STEP12')
time.sleep(0.5)

reach_pos_XY(-0.05, 0.0, False)
print('STEP13')
time.sleep(5)
'''
print('BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB')

#plt.plot(graph_frn)
plt.plot(graph_f, color = 'r')
#plt.plot(graph_d, color = 'g')
plt.grid()
plt.show()

plt.plot(graph_posX)
plt.plot(graph_posY)
#plt.plot(graph_d, color = 'g')
plt.grid()
plt.show()

raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()

