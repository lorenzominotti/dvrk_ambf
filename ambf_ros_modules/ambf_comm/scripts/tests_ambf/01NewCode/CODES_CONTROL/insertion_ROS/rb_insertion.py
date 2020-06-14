#!/usr/bin/env python2.7
from __future__ import division
# Import the Client from ambf_client package
from ambf_client import Client
import time
import math
import rospy
from std_msgs.msg import Float64
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from scipy import signal
from numpy import asarray
from numpy import savetxt


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


class Insertion:
	
	error_force2 = []
	graph_f2 = []
	er = []

	force_raw = []
	graph_f = []
	graph_m = []
	vec_step = []
	force1 = []
	force_vect = []
	error_force = []
	error_pos = []
	fr_x = []
	fr_y = []
	fr_z = []
	er_x = []
	er_y = []
	er_z = []
	er1 = []
	er2 = []

	degree = 0
	delta = 0.6 
	delta_m = 0.00005
	delta_m_start = 0.00003
	band = 0.03
	band2 = 0.5
	limit_mi = 0.30
	update_pos = False

	f_inv = 0.01

	count_mi_loop = 0
	P_value = 0
	I_value = 0
	D_value = 0
	graph_frn = []
	graph_fd = []
	q1_plot = []
	q2_plot = []
	error_abs = []
	posX = 0
	posY = 0
	posZ = 0
	pi = math.pi
	l_RCC = 0.4318
	l_tool_original = 0.4162
	l_tool = 0.05 + l_tool_original

	deltat_a = 0
	time = []
	deltat_a_ef = 0
	time_ef = []

	window = []
	window_size = 20


	#reach a certain target force. In particular, reach a contact force higher than the current contact force
	def reachFhigh(self, m_start, force_goal, flag):
			
		if flag == True:
			self.m = m_start
		if flag == False:
			self.m = self.m

		count = 0
		
		sum = 0
		count1 = 0
		force_old2 = 0
		force_old1 = 0
		while self.m < self.limit_mi:
			self.time_start_a = time.time()
			force_raw_now = psm_handle_mi.get_force()
			self.force = force_raw_now
			count = count + 1
			if count < self.window_size + 1:
				self.window = np.append(self.window, force_raw_now)
				self.force = force_raw_now
			else:
				for i in range(1, self.window_size):
					self.window[i-1] = self.window[i]
					if i == (self.window_size - 1):
						self.window[i] = force_raw_now
					sum = sum + self.window[i-1]
				self.force = sum / self.window_size
				sum = 0

			if self.force > (force_goal + self.band):
				self.m = self.m - self.delta_m_start/2
				psm_handle_pel.set_joint_pos(0, self.m)
			if self.force < (force_goal - self.band):
				self.m = self.m + self.delta_m_start/2
				psm_handle_pel.set_joint_pos(0, self.m)

			
			if self.force > force_goal:
				#keep = self.force
				self.count_stop(force_goal)
				break

			print(self.force)
			
			
			psm_handle_pel.set_joint_pos(0, self.m)
			force_old2 = force_old1
			force_old1 = self.force
			self.graph_f = np.append(self.graph_f, self.force)
			
			self.count_time()

	#reach a certain target force. In particular, reach a contact force lower than the current contact force
	def reachFlow(self, m_start, force_goal, flag):
			
		if flag == True:
			self.m = m_start
		if flag == False:
			self.m = self.m

		count = 0
		
		sum = 0
		count1 = 0
		force_old2 = 0
		force_old1 = 0
		while self.m < self.limit_mi:
			self.time_start_a = time.time()
			force_raw_now = psm_handle_mi.get_force()
			self.force = force_raw_now
			count = count + 1
			if count < self.window_size + 1:
				self.window = np.append(self.window, force_raw_now)
				self.force = force_raw_now
			else:
				for i in range(1, self.window_size):
					self.window[i-1] = self.window[i]
					if i == (self.window_size - 1):
						self.window[i] = force_raw_now
					sum = sum + self.window[i-1]
				self.force = sum / self.window_size
				sum = 0

			if self.force > (force_goal + self.band):
				self.m = self.m - self.delta_m_start/2
				psm_handle_pel.set_joint_pos(0, self.m)
			if self.force < (force_goal - self.band):
				self.m = self.m + self.delta_m_start/2
				psm_handle_pel.set_joint_pos(0, self.m)

			print(self.force)

			
			if self.force < force_goal:
				self.count_stop(force_goal)
				break
		
			psm_handle_pel.set_joint_pos(0, self.m)
			force_old2 = force_old1
			force_old1 = self.force
			self.graph_f = np.append(self.graph_f, self.force)
			
			self.count_time()


	#count the time
	def count_time(self):
		time_end_a = time.time()
		self.deltat_a = (time_end_a-self.time_start_a) + self.deltat_a 
		self.time = np.append(self.time, self.deltat_a)


	#maintain the current value of contact force for a certain amount of time (here 2 seconds)
	def count_stop(self, force_goal):
		time_end_a = time.time()
		t = 0
		count = 0
		
		sum = 0
		count1 = 0
		force_old2 = 0
		force_old1 = 0
		delta = 0.00001
		while t < 2:
			self.time_start_a = time.time()
			time_now = time.time()
			t = time_now-time_end_a
			#print(t)
			force_raw_now = psm_handle_mi.get_force()
			self.force = force_raw_now
			count = count + 1
			if count < self.window_size + 1:
				self.window = np.append(self.window, force_raw_now)
				self.force = force_raw_now
			else:
				for i in range(1, self.window_size):
					self.window[i-1] = self.window[i]
					if i == (self.window_size - 1):
						self.window[i] = force_raw_now
					sum = sum + self.window[i-1]
				self.force = sum / self.window_size
				sum = 0
			#print(self.force)

			if self.force > (force_goal + self.band):
				self.m = self.m - delta/2
				psm_handle_pel.set_joint_pos(0, self.m)
			if self.force < (force_goal - self.band):
				self.m = self.m + delta/2
				psm_handle_pel.set_joint_pos(0, self.m)

			print(self.force)

			self.graph_f = np.append(self.graph_f, self.force)
			self.count_time()


	def plot(self):
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/rb_ins_f.csv', self.graph_f, delimiter=",")
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/rb_ins_t.csv', self.time, delimiter=",")

		plt.figure()
		plt.plot(self.time, self.graph_f, color = 'r', label = "Force")
		plt.xlabel('Time [s]')
		plt.ylabel('Force [N]')	
		plt.legend(loc='best')
		plt.grid()

		plt.show()
		

def main():

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
	psm_handle_pfl.set_joint_pos(0, 0)
	psm_handle_base.set_joint_pos(0, math.radians(0))
	time.sleep(2)


	psm_handle_pel.set_joint_pos(0, 0)
	time.sleep(1)
	psm_handle_pel.set_joint_pos(0, 0)
	time.sleep(1)
	m_start = 0.18
	psm_handle_pel.set_joint_pos(0, m_start)
	time.sleep(1)
	print(psm_handle_trl.get_pos())


	ins = Insertion()
	ins.reachFhigh(m_start, 5, True)
	ins.reachFlow(m_start, 0.5, False)
	ins.reachFhigh(m_start, 5, False)
	ins.reachFlow(m_start, 0.5, False)
	ins.reachFhigh(m_start, 5, False)
	ins.reachFlow(m_start, 0.5, False)
	ins.reachFhigh(m_start, 5, False)
	ins.reachFlow(m_start, 0.5, False)
	#ins.reachFhigh(m_start, 5, False)
	ins.plot()

	
	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()



