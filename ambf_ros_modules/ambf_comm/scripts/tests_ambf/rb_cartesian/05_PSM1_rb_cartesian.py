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
	
	graph_q2_set0 = []
	graph_q2_set1 = []
	graph_q2_set3 = []
	graph_q2_set4 = []
	graph_q2_set2 = []
	j2_read = []
	j2_read1 = []
	j2_read2 = []
	j2_read3 = []
	j2_read4 = []
	x_plot = []
	y_plot = []
	z_plot = []
	pos_actualx = []
	pos_actualy = []
	pos_actualz = []

	force_const = 2
	force_raw = []
	graph_f = []
	graph_m = []
	vec_step = []
	force1 = []
	force_vect = []

	degree = 0
	delta = 0.6 
	delta_m = 0.00005
	delta_m_start = 0.0001
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
	graph_posZ = []
	graph_posX = []
	graph_posY = []
	posX = 0
	posY = 0
	posZ = 0
	pi = math.pi
	l_RCC = 0.4318
	l_tool_original = 0.4162
	l_tool = 0.05 + l_tool_original

	Kp_start = 0.000001
	Ki_start = 0.000001

	force_const = 6


	Integrator = 0
	Derivator = 0
	time_now = 0

	flag_first_pos = True

	
	'''
	Kp = 0.002
	Ki = 0.00005
	#Kd = 0.00008
    '''
	Kp = 0.002
	Ki = 0.00005
	#Kd = 0.00008

	
	
	def __init__(self):
		pass
	
	def inverse_kinematics(self, X_des, Y_des, Z_des):

		r = math.sqrt(math.pow(X_des,2)+math.pow(Y_des,2)+math.pow(Z_des,2))

		self.q1 = math.asin((X_des)/(math.sqrt(math.pow(X_des,2)+math.pow(Z_des,2))))
		self.q2 = math.asin(-Y_des/r)
		self.q3 = self.l_RCC - self.l_tool + r


	def set_position_robot(self, q1_set, q2_set, q3_set):
				
		psm_handle_base.set_joint_pos(0, q1_set)
		psm_handle_pfl.set_joint_pos(0, q2_set)
		psm_handle_pel.set_joint_pos(0, q3_set)


	def get_position_joints_PSM(self):

		self.q1_read = psm_handle_base.get_joint_pos(0)
		self.q2_read = psm_handle_base.get_joint_pos(3)
		self.q3_read = psm_handle_base.get_joint_pos(4)

	def forward_kinematics(self, q1, q2, q3):

		self.x_fk = math.cos(q2)*math.sin(q1)*(self.l_tool-self.l_RCC+q3)
		self.y_fk = -math.sin(q2)*(self.l_tool-self.l_RCC+q3)
		self.z_fk = -math.cos(q1)*math.cos(q2)*(self.l_tool-self.l_RCC+q3)

	def jacobian(self, qj1, qj2, qj3):

		self.jac = np.zeros((3,3))
		self.jac[0,0] = math.cos(qj1)*math.cos(qj2)*(self.l_tool-self.l_RCC+qj3)
		self.jac[0,1] = -math.sin(qj1)*math.sin(qj2)*(self.l_tool-self.l_RCC+qj3)
		self.jac[0,2] = math.cos(qj2)*math.sin(qj1)
		self.jac[1,0] = 0
		self.jac[1,1] = -math.cos(qj2)*(self.l_tool-self.l_RCC+qj3)
		self.jac[1,2] = -math.sin(qj2)
		self.jac[2,0] = math.cos(qj2)*math.sin(qj1)*(self.l_tool-self.l_RCC+qj3)
		self.jac[2,1] = math.cos(qj1)*math.sin(qj2)*(self.l_tool-self.l_RCC+qj3) 
		self.jac[2,2] = -math.cos(qj1)*math.cos(qj2)
		#print("jacobian")
		#print(self.jac)

	def plots(self):
		plt.plot(self.graph_f, color = 'r')
		#plt.plot(graph_d, color = 'g')
		plt.grid()
		plt.show()


	#reach a certain position defined by X and Y coordinates trying to mantain constant the force along Z 
	
	
	def approach_goal_Z(self, m_start):
		self.m = m_start
		force_old2 = 0
		force_old1 = 0
		while self.m < self.limit_mi:
			
			force_raw_now = psm_handle_mi.get_force()
			self.force = force_raw_now
			print(self.force)
			average = (force_old2 + force_old1)/2
			if(self.force > (average + self.delta)) or (self.force < (average - self.delta)):
				self.force = force_old1
				print('\n')
				print('UNEXPECTED_PEAK..........COMPENSATION')
				print('\n')

			if self.force > (self.force_const + self.band):
				self.m = self.m - self.delta_m_start/2
				psm_handle_pel.set_joint_pos(0, self.m)
			if self.force < (self.force_const - self.band):
				self.m = self.m + self.delta_m_start/2
				psm_handle_pel.set_joint_pos(0, self.m)

			if (self.force < (self.force_const + self.band)) and (self.force > (self.force_const - self.band)):
				self.count_mi_loop = self.count_mi_loop + 1
			if self.count_mi_loop == 50:
				self.count_mi_loop = 0
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
	
	
	def reach_pos_XY(self, goal_x, goal_y, start):

		if start == True:
			posx_start =  0.000000001
			posy_start =  0.000000001
			posz_start =  0.000000001
			X_desired = 0
			Y_desired = 0
			count_step = 0
			self.posx_end = goal_x
			self.posy_end = goal_y

		if start == False:
			
			posx_start =  self.posx_end
			posy_start =  self.posy_end
			self.posx_end = goal_x
			self.posy_end = goal_y
			X_desired = posx_start
			Y_desired = posy_start

		path_x = abs(goal_x - posx_start)
		path_y = abs(goal_y - posy_start)

		if path_x >= path_y:
			path_long = path_x
			path_short = path_y
		if path_x < path_y:
			path_long = path_y
			path_short = path_x

		d_step_long = 0.0001
		d_step_short = (path_short / path_long) * d_step_long
		if path_x >= path_y:
			dx = d_step_long
			dy = d_step_short
		if path_x < path_y:
			dx = d_step_short
			dy = d_step_long 

		count = 0
		window = []
		window_size = 10
		sum = 0
		count1 = 0
		
		stop_x = False
		stop_y = False
		
		while (stop_x == False) or (stop_y == False):
			
			force_raw_now = psm_handle_mi.get_force()

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
			
			error = self.force_const - self.force
			self.P_value = (self.Kp * error)
		
			self.Integrator = self.Integrator + error
			self.I_value = self.Integrator * self.Ki
		
		
			PID = self.P_value + self.I_value 

			self.get_position_joints_PSM()
			self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
			#X_desired_2 = self.x_fk
			#Y_desired_2 = self.y_fk
			Z_desired_2 = self.z_fk
			Z_desired_2 = Z_desired_2 + PID*Z_desired_2

			#INTERNAL BLOCK
			
			self.inverse_kinematics(X_desired, Y_desired, Z_desired_2)
			self.jacobian(self.q1, self.q2, self.q3)
			
			if self.flag_first_pos == True:
				self.set_position_robot(self.q1, self.q2, self.q3)
				time.sleep(0.1)
				self.get_position_joints_PSM()
				self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
				self.flag_first_pos = False
				
			if self.flag_first_pos == False:
				delta_cart = [0, 0, 0]
				delta_q = [0, 0, 0]
				delta_cart[0] = X_desired-self.x_fk
				delta_cart[1] = Y_desired-self.y_fk
				delta_cart[2] = Z_desired_2-self.z_fk
				#print(delta_cart)
				invJ = np.linalg.inv(self.jac) 
				delta_q = invJ.dot(delta_cart)

				
				#put a PI control here
				#kp_q = 0.01
				kp_q = 10
				ki_q = 5
				Integ_q = [0,0,0]
				P_value_q = [0,0,0]
				I_value_q = [0,0,0]
				PI_q = [0,0,0]
				for i in range(0,3):
					P_value_q[i] = kp_q*delta_q[i]
					Integ_q[i] = Integ_q[i] + delta_q[i]
					I_value_q[i] = Integ_q[i] * ki_q
					PI_q[i] = P_value_q[i] + I_value_q[i]
					delta_q[i] = delta_q[i]*PI_q[i]
				

				self.q1 = self.q1 + delta_q[0]
				self.q2 = self.q2 + delta_q[1]
				self.q3 = self.q3 + delta_q[2]
				self.set_position_robot(self.q1, self.q2, self.q3)
				time.sleep(0.1)
				self.get_position_joints_PSM()
				self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
				
				if self.update_pos == False:
					print(self.force)
					if (self.force < (self.force_const + self.band)) and (self.force > (self.force_const - self.band)):
						self.count_mi_loop = self.count_mi_loop + 1
						print("waiting...", self.count_mi_loop)
					if self.count_mi_loop == 10:
						count_mi_loop = 0
						self.update_pos = True
				
			if self.update_pos == True:
				print(self.force)

				if goal_x >= posx_start:
					if goal_x > X_desired:
						X_desired = X_desired + dx
					else:
						stop_x = True
				if goal_x < posx_start:
					if goal_x < X_desired:
						X_desired = X_desired - dx
					else:
						stop_x = True

				if goal_y >= posy_start:
					if goal_y > Y_desired:
						Y_desired = Y_desired + dy
					else:
						stop_y = True
				if goal_y < posy_start:
					if goal_y < Y_desired:
						Y_desired = Y_desired - dy
					else:
						stop_y = True

				self.force1 = self.force

			print("\n")
			print(X_desired)
			print(Y_desired)
			print("\n")
			
			

			self.graph_f = np.append(self.graph_f, self.force1)
			'''
			posX =  pos.x
			graph_posX = np.append(graph_posX, posX)
			posY =  pos.y
			graph_posY = np.append(graph_posY, posY)
			'''
			time.sleep(self.f_inv)
			#self.count_step = self.count_step + 1
			#vec_step = np.append(vec_step, count_step)

		self.flag_first_pos = True	
		self.update_pos == False
		

	
def main():

	# Let's sleep for a very brief moment to give the internal callbacks
	# to sync up new data from the running simulator
	time.sleep(0.2)

	print('\n\n----')

	raw_input("Number of joints of pitchfrontLink")
	num_joints_pfl = psm_handle_pfl.get_num_joints()
	print(num_joints_pfl)

	raw_input("Name of joints of pitchfrontLink")
	name_joints_pfl = psm_handle_pfl.get_joint_names()
	print(name_joints_pfl)

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

	psm_handle_pel.set_joint_pos(0, 0)
	psm_handle_pfl.set_joint_pos(0, 0)
	psm_handle_base.set_joint_pos(0, math.radians(0))
	time.sleep(2)
	#psm_handle_mi.set_joint_pos(0, 0)
	psm_handle_pel.set_joint_pos(0, 0)
	time.sleep(1)
	#psm_handle_pel.set_joint_pos(0, 0.1)
	#time.sleep(1)
	psm_handle_pel.set_joint_pos(0, 0)
	m_start = 0.16
	psm_handle_pel.set_joint_pos(0, m_start)
	

	cart_c = Cartesian_control()
	
	print("go in 2 seconds!!!")
	time.sleep(2)
	#cart_c.temp(math.radians(-20), math.radians(-30), 0.08)
	#cart_c.temp(math.radians(30), -0.0, 0.09)
	
	#cart_c.temp(0.0-0.08, 4.608079977738305e-05-0.04, -0.00042080074000368033-0.08)
	
	
	#cart_c.temp(-0.06, 0.04, 0.0)
	cart_c.approach_goal_Z(m_start)
	cart_c.reach_pos_XY(-0.05, 0.03, True)
	cart_c.reach_pos_XY(0.05, 0.06, False)
	cart_c.reach_pos_XY(0.00, -0.02, False)
	
	'''
	cart_c.approach_goal_Z(m_start)
	cart_c.reach_pos_XY(0.01, 0.02, True)
	cart_c.reach_pos_XY(-0.01, 0.04, False)
	cart_c.reach_pos_XY(0.03, 0.02, False)
	
	cart_c.reach_pos_XY(0.01, -0.01, False)
	cart_c.reach_pos_XY(-0.03, -0.05, False)
	'''
	cart_c.plots()
	'''
	x=0
	y=0
	z=-0.17
	step = 0.0001
	for i in np.arange(0, 0.1, step):
		x = x+step
		y = y-step
		z = z#-step/2
		print(x,y)
		cart_c.test_pos(x, y, z)
	'''


	
	#cart_c.temp01
 
	#cart_c.plots()
	#cart_c.temp(-0.15, 0.16, -0.16)
	
	print('STEP1')



	#time.sleep(5)
	
	
	
	#cart_c.plots()


	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()

