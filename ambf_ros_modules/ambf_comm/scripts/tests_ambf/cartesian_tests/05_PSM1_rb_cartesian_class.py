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
import adaptfilt

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


class Cartesian_control:
	
	force_raw = []
	graph_f = []
	graph_m = []
	vec_step = []
	force1 = []
	force_vect = []

	degree = 0
	delta = 0.6
	m = 0 
	delta_m = 0.000005
	delta_m_start = 0.0005
	band = 0.05
	limit_mi = 0.30

	f_inv = 0.01

	count_mi_loop = 0
	P_value = 0
	I_value = 0
	D_value = 0
	graph_frn = []
	graph_posZ = []
	graph_posX = []
	graph_posY = []
	posx_start0 = 0
	posy_start0 = 0
	posX = 0
	posY = 0
	posZ = 0

	force_const = 6

	Kp = 0.0005
	#Ki = 0.0001
	Ki = 0.00001
	Kd = 0.00008

	Integrator = 0
	Derivator = 0
	time_now = 0
	
	def __init__(self):
		pass

	#movement only along z to get in contact with the body and reach a specific force value (f_const)
	def approach_goal_Z(self, m_start):
		self.m = m_start
		force_old2 = 0
		force_old1 = 0
		while self.m < self.limit_mi:
			
			force_raw_now = psm_handle_mi.get_force()
			self.force = force_raw_now
			print(self.force)
			average = (force_old2 + force_old1)/2
			'''
			if(self.force > (average + self.delta)) or (self.force < (average - self.delta)):
				self.force = force_old1
				print('\n')
				print('UNEXPECTED_PEAK..........COMPENSATION')
				print('\n')
			'''
			if self.force > (self.force_const + self.band):
				self.m = self.m - self.delta_m_start/2
				psm_handle_pel.set_joint_pos(0, self.m)
			if self.force < (self.force_const - self.band):
				self.m = self.m + self.delta_m_start/2
				psm_handle_pel.set_joint_pos(0, self.m)

			if (self.force < (self.force_const + self.band)) and (self.force > (self.force_const - self.band)):
				self.count_mi_loop = self.count_mi_loop + 1
			if self.count_mi_loop == 50:
				break
			
			psm_handle_pel.set_joint_pos(0, self.m)
			force_old2 = force_old1
			force_old1 = self.force
			self.graph_f = np.append(self.graph_f, self.force)
			PID = 1
			self.graph_frn = np.append(self.graph_frn, force_raw_now)
			self.graph_m = np.append(self.graph_m, self.m)
			pos = psm_handle_trl.get_pos()
			self.posZ =  pos.z
			self.graph_posZ = np.append(self.graph_posZ, self.posZ)
			self.posX =  pos.x
			self.posY =  pos.y

	#reach a certain position defined by X and Y coordinates trying to mantain constant the force along Z 
	def reach_pos_XY(self, goal_x, goal_y, start):

		#if it is the first time that the function is called, the flag needs to be true, so that positions are expressed with 
		#respect to the ball position reached just after the approach function was called. If it is not the first time, the flag
		#needs to be false, so that position specified then remain referred to the first one specified the first time
		if start == True:
			pos_start = psm_handle_trl.get_pos()
			posx_start =  pos_start.x
			posy_start =  pos_start.y
			posz_start =  pos_start.z
			self.posx_start0 = posx_start
			self.posy_start0 = posy_start
			self.posz_start0 = posz_start
			target_x = posx_start + goal_x
			target_y = posy_start + goal_y
			self.X_desired = 0
			self.Y_desired = 0
			count_step = 0

			self.q1 = 0
			self.q2 = 0
			self.q3 = self.m

		if start == False:
			pos_start = psm_handle_trl.get_pos()
			posx_start =  pos_start.x
			posy_start =  pos_start.y
			target_x = self.posx_start0 + goal_x
			target_y = self.posy_start0 + goal_y

		#do a path to reach the specified point. Find 2 different steps so to reach the goal almost in the same time with a straight line
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

		count = 0
		window = []
		window_size = 10
		sum = 0
		count1 = 0


		stop_x = False
		stop_y = False

		#remain in this cycle until the goal position is reached
		while (stop_x == False) or (stop_y == False):
			
			#read the force 
			force_raw_now = psm_handle_mi.get_force()
			#get the cartesian position of the ball
			pos_tool = psm_handle_trl.get_pos()
			px = pos_tool.x
			py = pos_tool.y
			pz = pos_tool.z
			self.Z_desired = pz
			
			#filter force with moving average
			count = count + 1
			if count < window_size + 1:
				window = np.append(window, force_raw_now)
				self.force = force_raw_now
			else:
				for i in range(1, window_size):
					window[i-1] = window[i]
					if i == (window_size - 1):
						window[i] = force_raw_now
					sum = sum + window[i-1]
				self.force = sum / window_size
				sum = 0
			
			#PI control on Z direction. I want the Z desired
			error = self.force_const - self.force
			P_value = (self.Kp * error)

			self.Integrator = self.Integrator + error
			I_value = self.Integrator * self.Ki


			PID = P_value + I_value 

			self.Z_desired = self.Z_desired + PID*self.Z_desired

			#Update the position desired
			print(px-self.posx_start0, py-self.posy_start0, pz-self.posz_start0)
			if target_x >= posx_start:
				if target_x > px:
					self.X_desired = self.X_desired - dx
				else:
					stop_x = True
			if target_x < posx_start:
				if target_x < px:
					self.X_desired = self.X_desired + dx
				else:
					stop_x = True
			if target_y >= posy_start:
				if target_y > py:
					self.Y_desired = self.Y_desired + dy
				else:
					stop_y = True
			if target_y < posy_start:
				if target_y < py:
					self.Y_desired = self.Y_desired - dy
				else:
					stop_y = True			

			self.internal_block(self.X_desired, self.Y_desired, self.Z_desired, px, py, pz)


			self.graph_f = np.append(self.graph_f, self.force)

			pos = psm_handle_trl.get_pos()
			#posZ =  pos.z
			#graph_posZ = np.append(graph_posZ, posZ)
			self.posX =  pos.x
			self.graph_posX = np.append(self.graph_posX, self.posX)
			self.posY =  pos.y
			self.graph_posY = np.append(self.graph_posY, self.posY)
			time.sleep(self.f_inv)
			#self.count_step = self.count_step + 1
			#self.vec_step = np.append(self.vec_step, self.count_step)	
	


	def internal_block(self, X_desired, Y_desired, Z_desired, X_real, Y_real, Z_real):

		#compute positions variations in cartesian space
		deltaX = X_desired - X_real
		deltaY = Y_desired - Y_real
		deltaZ = Z_desired - Z_real

		#inverse kinematics
		pi = math.pi
		l_RCC = 0.4318
		l_tool = 0.4162
		r = math.sqrt(math.pow(deltaX,2)+math.pow(deltaY,2)+math.pow(deltaZ,2))

		delta_q1 = pi/2 - (math.acos((math.pow(deltaX,2)-math.pow(deltaY,2))/(math.pow(deltaX,2)+math.pow(deltaZ,2))))/2
		delta_q2 = math.asin(deltaZ/(math.sqrt(abs(deltaY/r))))
		delta_q3 = l_RCC - l_tool + r

		self.q1 = self.q1 + delta_q1
		self.q2 = self.q2 + delta_q2
		self.q3 = self.q3 + delta_q3
		
		#set joint values
		psm_handle_base.set_joint_pos(0, self.q1)
		psm_handle_pfl.set_joint_pos(0, self.q2)
		psm_handle_pel.set_joint_pos(0, self.q3)
		


	def plots(self):
		plt.plot(self.graph_f, color = 'r')
		#plt.plot(graph_d, color = 'g')
		plt.grid()
		plt.show()

		plt.plot(self.graph_posX)
		plt.plot(self.graph_posY)
		#plt.plot(graph_d, color = 'g')
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
	m_start = 0.16
	psm_handle_pel.set_joint_pos(0, m_start)
	time.sleep(1)
	print(psm_handle_trl.get_pos())

	psm_handle_pel.set_joint_pos(0, math.radians(0))
	#psm_handle_tyl.set_joint_pos(1, math.radians(-20))
	#time.sleep(1)
	
	cart_c = Cartesian_control()
	cart_c.approach_goal_Z(m_start)

	print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')

	cart_c.reach_pos_XY(0.05, 0.03, True)
	print('STEP1')
	time.sleep(2)
	cart_c.reach_pos_XY(0.05, -0.10, False)
	print('STEP2')
	time.sleep(2)
	#cart_c.reach_pos_XY(0.00, -0.02, False)
	#print('STEP3')
	#time.sleep(5)	
	cart_c.plots()


	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()

