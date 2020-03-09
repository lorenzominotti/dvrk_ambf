#!/usr/bin/env python2.7
# Import the Client from ambf_client package
from ambf_client import Client
import time
import math
import rospy
import tf
import numpy as np
#import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
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
	error_force = []
	error_pos = []
	fr_x = []
	fr_y = []
	fr_z = []
	er_x = []
	er_y = []
	er_z = []
	graph_px = []
	graph_py = []
	graph_pxr = []
	graph_pyr = []

	degree = 0
	delta = 0.6 
	delta_m = 0.00005
	delta_m_start = 0.00005
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

	amplitude = 0.5

	force_const = 1.5-amplitude

	deltat_a = 0
	time = []
	deltat_a_ef = 0
	time_ef = []

	Integrator = 0
	Derivator = 0
	time_now = 0

	Integratorx = 0
	Integratory = 0
	Integratorz = 0

	flag_first_pos = True

	'''
	Kp = 0.01 #stiffer
	Ki = 0.00005
	#Kd = 0.00008
	'''

	Kp = 0.01 #stiffer
	Ki = 0.000008


	Integrator = 0
	Derivator = 0
	time_now = 0

	flag_first_pos = True
	
	def __init__(self):
		pass
	
	def inverse_kinematics(self, X_des, Y_des, Z_des):

		r = math.sqrt(math.pow(X_des,2)+math.pow(Y_des,2)+math.pow(Z_des,2))
		
		#self.q1 = 0.5*(math.acos((math.pow(Z_des,2)-math.pow(X_des,2))/(math.pow(X_des,2)+math.pow(Z_des,2))))   #original

		self.q1 = math.asin((X_des)/(math.sqrt(math.pow(X_des,2)+math.pow(Z_des,2))))
		self.q2 = math.asin(-Y_des/r)
		self.q3 = self.l_RCC - self.l_tool + r

		#print("acos arg")
		#print(math.pow(X_des,2)-math.pow(Y_des,2))/(math.pow(X_des,2)+math.pow(Z_des,2))
		#print(arcsin_q1)
		#self.q1 = arcsin_q1/2
		#print(self.q1)
		#print("joints angles prescribed")
		#print(self.q1*180/self.pi, self.q2*180/self.pi, self.q3)


	def set_position_robot(self, q1_set, q2_set, q3_set):
		
		#set joint values		
		psm_handle_base.set_joint_pos(0, q1_set)
		psm_handle_pfl.set_joint_pos(0, q2_set)
		psm_handle_pel.set_joint_pos(0, q3_set)


	def get_position_joints_PSM(self):

		self.q1_read = psm_handle_base.get_joint_pos(0)
		self.q2_read = psm_handle_base.get_joint_pos(3)
		self.q3_read = psm_handle_base.get_joint_pos(4)

		

		#print("actual angles")
		#print(self.q1_read*180/self.pi, self.q2_read*180/self.pi, self.q3_read)
		'''
		print("\n")
		print("pos end loop")
		print(self.q1_read, self.q2_read, self.q3_read)
		print("\n")
		print("pos all joints")
		print(psm_handle_base.get_all_joint_pos())
		'''

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

	def count_time(self):
		time_end_a = time.time()
		self.deltat_a = (time_end_a-self.time_start_a) + self.deltat_a 
		self.time = np.append(self.time, self.deltat_a)

	def count_time_ef(self):
		time_end_a_ef = time.time()
		self.deltat_a_ef = (time_end_a_ef-self.time_start_a) + self.deltat_a_ef 
		self.time_ef = np.append(self.time_ef, self.deltat_a_ef)

	def plots(self):
		#last_t = self.time.size
		#last_r = self.graph_f.size
		#last_d = self.graph_fd.size
		#print(self.time.size)
		#print(self.graph_f.size)
		#print(self.graph_fd.size)
		
		time = []
		time = self.time
		time_ef = []
		time_ef = self.time_ef
		'''
		fig, axs = plt.subplots(nrows = 3)
		axs[0].plot(time, self.graph_f, color = 'r', label = "actual force")
		axs[0].plot(time, self.graph_fd, color = 'b', label = "target force")
		#axs[0].set(xlabel = 'Time [s]', ylabel = 'Force [N]')	
		axs[0].set(ylabel = 'Force [N]')	
		axs[0].legend(loc='best')
		axs[0].grid()
	
		axs[1].plot(time, self.error_force, color = 'r', label = "error")
		axs[1].set(xlabel = 'Time [s]', ylabel = 'Force_error_norm')
		axs[1].legend(loc='best')
		axs[1].grid()

		axs[2].plot(time, self.graph_px, color = 'r', label = "position x")
		axs[2].plot(time, self.graph_py, color = 'b', label = "position y")
		axs[2].set(xlabel = 'Time [s]', ylabel = 'Position [m]')
		axs[2].legend(loc='best')
		axs[2].grid()

		plt.show()
		
		
		fig, axs = plt.subplots(nrows = 5)

		axs[0].plot(time, self.graph_f, color = 'r', label = "actual force")
		axs[0].plot(time, self.graph_fd, color = 'b', label = "target force")
		axs[0].set(ylabel = 'Force [N]')	
		axs[0].legend(loc='best')
		axs[0].grid()

		axs[1].plot(time, self.error_force, color = 'r', label = "error")
		axs[1].set(ylabel = 'Force_error %')
		axs[1].legend(loc='best')
		axs[1].grid()

		axs[2].plot(time, self.er_x, label = "err_posx")
		axs[2].set(ylabel = 'pox_error %')
		axs[2].legend(loc='best')
		axs[2].grid()

		axs[3].plot(time, self.er_y, label = "err_posy")
		axs[3].set(ylabel = 'posy_error %')
		axs[3].legend(loc='best')
		axs[3].grid()

		axs[4].plot(time, self.er_z, label = "err_posz")
		axs[4].set(ylabel = 'posz_error %')
		axs[4].set(xlabel = 'Time [s]')	
		axs[4].legend(loc='best')
		axs[4].grid()
	
		plt.show()
		
		fig, axs = plt.subplots(nrows = 2)
		axs[0].plot(time, self.graph_f, color = 'r', label = "actual force")
		axs[0].plot(time, self.graph_fd, color = 'b', label = "target force")
		#axs[0].set(xlabel = 'Time [s]', ylabel = 'Force [N]')	
		axs[0].set(ylabel = 'Force [N]')	
		axs[0].legend(loc='best')
		axs[0].grid()
		
		axs[1].plot(time, self.fr_x, label = "force x")
		axs[1].plot(time, self.fr_y, label = "force y")
		axs[1].plot(time, self.fr_z, label = "force z")
		axs[1].set(xlabel = 'Time [s]')
		axs[1].set(ylabel = 'Force [N]')
		axs[1].legend(loc='best')
		axs[1].grid()
		plt.show()
		'''

		fig, axs = plt.subplots(nrows = 4)
		axs[0].plot(time, self.graph_f, color = 'r', label = "actual force")
		axs[0].plot(time, self.graph_fd, color = 'b', label = "target force")
		#axs[0].set(xlabel = 'Time [s]', ylabel = 'Force [N]')	
		axs[0].set(ylabel = 'Force [N]')	
		axs[0].legend(loc='best')
		axs[0].grid()
	
		axs[1].plot(time, self.error_force, color = 'r', label = "error")
		axs[1].set(xlabel = 'Time [s]', ylabel = 'Force_error_norm')
		axs[1].legend(loc='best')
		axs[1].grid()

		axs[2].plot(time, self.graph_px, color = 'r', label = "pos x")
		axs[2].plot(time, self.graph_pxr, color = 'b', label = "pos x real")
		axs[2].set(xlabel = 'Time [s]', ylabel = 'Position [m]')
		axs[2].legend(loc='best')
		axs[2].grid()

		axs[3].plot(time, self.graph_py, color = 'r', label = "pos y")
		axs[3].plot(time, self.graph_pyr, color = 'b', label = "pos y real")
		axs[3].set(xlabel = 'Time [s]', ylabel = 'Position [m]')
		axs[3].legend(loc='best')
		axs[3].grid()


		plt.show()
		

	def plot_sin(self):
		time = []
		time = self.time
		time_ef = []
		time_ef = self.time_ef
		
		fig, axs = plt.subplots(nrows = 2)
		axs[0].plot(time, self.graph_f, color = 'r', label = "actual force")
		axs[0].plot(time, self.graph_fd, color = 'b', label = "target force")
		#axs[0].set(xlabel = 'Time [s]', ylabel = 'Force [N]')	
		axs[0].set(ylabel = 'Force [N]')	
		axs[0].legend(loc='best')
		axs[0].grid()
	
		axs[1].plot(time, self.error_force, color = 'r', label = "error")
		axs[1].set(xlabel = 'Time [s]', ylabel = 'Force_error_norm')
		axs[1].legend(loc='best')
		axs[1].grid()

		plt.show()
		

	
	def approach_goal_Z(self, m_start):
		self.m = m_start
		force_old2 = 0
		force_old1 = 0
		while self.m < self.limit_mi:
			
			self.time_start_a = time.time()
			_,_,force_raw_now = psm_handle_mi.get_force()
			#force_raw_now = psm_handle_mi.get_force()
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
			self.graph_fd = np.append(self.graph_fd, self.force_const)
			self.error_force = np.append(self.error_force, 0)
			self.count_time_ef()
			self.graph_px = np.append(self.graph_px, 0)
			self.graph_py = np.append(self.graph_py, 0)
			self.graph_pxr = np.append(self.graph_pxr, 0)
			self.graph_pyr = np.append(self.graph_pyr, 0)
			PID = 1
			self.graph_frn = np.append(self.graph_frn, force_raw_now)
			self.graph_m = np.append(self.graph_m, self.m)
			pos = psm_handle_trl.get_pos()
			self.posZ =  pos.z
			self.graph_posZ = np.append(self.graph_posZ, self.posZ)
			self.posX =  pos.x
			self.posY =  pos.y
			self.count_time()
			fr_x,_,_ = psm_handle_mi.get_force()
			#fr_x = psm_handle_mi.get_force()
			self.fr_x = np.append(self.fr_x, fr_x)
			_,fr_y,_ = psm_handle_mi.get_force()
			#fr_y = psm_handle_mi.get_force()
			self.fr_y = np.append(self.fr_y, fr_y)
			_,_,fr_z = psm_handle_mi.get_force()
			#fr_z = psm_handle_mi.get_force()
			self.fr_z = np.append(self.fr_z, fr_z)
			ex = 0
			self.er_x = np.append(self.er_x, ex)
			ey = 0
			self.er_y = np.append(self.er_y, ey)
			ez = 0
			self.er_z = np.append(self.er_z, ez)

	def pos_loop(self, x_desired, y_desired, z_desired):

			self.get_position_joints_PSM()
			self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)


			x_desired_2 = self.x_fk
			y_desired_2 = self.y_fk
			z_desired_2 = self.z_fk
			#print(self.z_fk)
			
			#z_desired = self.Z_desired_2
			#print(Z_desired)
			
			i= 0
			x_loop = self.x_fk
			y_loop = self.y_fk
			z_loop = self.z_fk
			while i<10:
				Kpx = 5
				Kix = 0.005
				errorx = x_desired - x_loop
				self.P_value = (Kpx * errorx)		
				self.Integratorx = self.Integratorx + errorx
				self.I_value = self.Integrator * Kix		
				PID = self.P_value + self.I_value 
				x_desired_2 = x_desired_2 + PID*x_desired_2

				Kpy = 5
				Kiy = 0.005
				errory = y_desired - y_loop
				self.P_value = (Kpy * errory)		
				self.Integratory = self.Integratory + errory
				self.I_value = self.Integrator * Kiy		
				PID = self.P_value + self.I_value 
				y_desired_2 = y_desired_2 + PID*y_desired_2
				
				Kpz = 0.00005 
				Kiz = 0.000005
				errorz = z_desired_2 - z_loop
				self.P_value = (Kpz * errorz)		
				self.Integratorz = self.Integratorz + errorz
				self.I_value = self.Integrator * Kiz		
				PID = self.P_value + self.I_value 
				z_desired_2 = z_desired_2 + PID*z_desired_2
				
				i = i + 1
			
			return x_desired_2, y_desired_2, z_desired_2


	def reach_pos_XY(self, goal_x, goal_y, start):

		if start == True:
			posx_start =  0.000000001
			posy_start =  0.000000001
			posz_start =  0.000000001
			X_desired = 0.0
			Y_desired = 0.0
			count_step = 0
			self.posx_end = goal_x
			self.posy_end = goal_y
			self.first = 1

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

		d_step_long = 0.00001
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

			self.time_start_a = time.time()			
			_,_,force_raw_now = psm_handle_mi.get_force()
			#force_raw_now = psm_handle_mi.get_force()

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
			e_rel = error/self.force_const
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

			#print("\n")
			#print(X_desired, Y_desired, Z_desired_2)
			X_desired1, Y_desired1, Z_desired_1 = self.pos_loop(X_desired, Y_desired, Z_desired_2)
			#print(X_desired1, Y_desired1, Z_desired_1)
			#print(self.x_fk, self.y_fk, self.z_fk)
			#INTERNAL BLOCK
			
			self.inverse_kinematics(X_desired1, Y_desired1, Z_desired_1)
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
				self.q1 = self.q1 + delta_q[0]
				self.q2 = self.q2 + delta_q[1]
				self.q3 = self.q3 + delta_q[2]
				print(self.q1,self.q2,self.q3)
				self.set_position_robot(self.q1, self.q2, self.q3)
				time.sleep(0.1)
				self.get_position_joints_PSM()
				self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
				print(self.x_fk, self.y_fk, self.z_fk)
				#self.update_pos == True
				
			if self.update_pos == False:
				print(self.force)
				if (self.force < (self.force_const + self.band)) and (self.force > (self.force_const - self.band)):
					self.count_mi_loop = self.count_mi_loop + 1
					print("waiting...", self.count_mi_loop)
				if self.count_mi_loop == 10:
					count_mi_loop = 0
					
					self.update_pos = True

				self.graph_f = np.append(self.graph_f, self.force)
				self.graph_fd = np.append(self.graph_fd, self.force_const)
				self.count_time()
				self.error_force = np.append(self.error_force, e_rel)
				self.count_time_ef()
				self.graph_px = np.append(self.graph_px, X_desired)
				self.graph_py = np.append(self.graph_py, Y_desired)
				self.graph_pxr = np.append(self.graph_pxr, self.x_fk)
				self.graph_pyr = np.append(self.graph_pyr, self.y_fk)

				fr_x,_,_ = psm_handle_mi.get_force()
				#fr_x = psm_handle_mi.get_force()
				self.fr_x = np.append(self.fr_x, fr_x)
				_,fr_y,_ = psm_handle_mi.get_force()
				#fr_y = psm_handle_mi.get_force()
				self.fr_y = np.append(self.fr_y, fr_y)
				_,_,fr_z = psm_handle_mi.get_force()
				#fr_z = psm_handle_mi.get_force()
				self.fr_z = np.append(self.fr_z, fr_z)

				#ex = (X_desired-self.x_fk)/X_desired
				ex = 0
				self.er_x = np.append(self.er_x, ex)
				#ey = (Y_desired-self.y_fk)/Y_desired
				ey = 0
				self.er_y = np.append(self.er_y, ey)
				#ez = (Z_desired_2-self.z_fk)/Z_desired_2
				ez = 0
				self.er_z = np.append(self.er_z, ez)
				
			if self.update_pos == True:
				print(self.force, self.force_const)

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

				self.graph_f = np.append(self.graph_f, self.force)
				self.graph_fd = np.append(self.graph_fd, self.force_const)
				self.count_time()
				self.error_force = np.append(self.error_force, e_rel)
				self.count_time_ef()
				self.graph_px = np.append(self.graph_px, X_desired)
				self.graph_py = np.append(self.graph_py, Y_desired)
				self.graph_pxr = np.append(self.graph_pxr, self.x_fk)
				self.graph_pyr = np.append(self.graph_pyr, self.y_fk)

				fr_x,_,_ = psm_handle_mi.get_force()
				#fr_x = psm_handle_mi.get_force()
				self.fr_x = np.append(self.fr_x, fr_x)
				_,fr_y,_ = psm_handle_mi.get_force()
				#fr_y = psm_handle_mi.get_force()
				self.fr_y = np.append(self.fr_y, fr_y)
				_,_,fr_z = psm_handle_mi.get_force()
				#fr_z = psm_handle_mi.get_force()
				self.fr_z = np.append(self.fr_z, fr_z)

				ex = (X_desired-self.x_fk)#/X_desired
				self.er_x = np.append(self.er_x, ex)
				ey = (Y_desired-self.y_fk)/Y_desired
				self.er_y = np.append(self.er_y, ey)
				ez = (Z_desired_2-self.z_fk)/Z_desired_2
				self.er_z = np.append(self.er_z, ez)
				self.update_pos = True

				self.update_pos = True

			self.x_loop = self.x_fk
			self.y_loop = self.y_fk
			self.z_loop = self.z_fk
				
			print("\n")
			print(X_desired)
			print(Y_desired)
			print("\n")
						
			#self.count_step = self.count_step + 1
			#vec_step = np.append(vec_step, count_step)

		self.flag_first_pos = True	
		self.update_pos == False

	def temp(self, g1, g2, g3):
		
		self.inverse_kinematics(g1, g2, g3)
		self.set_position_robot(self.q1, self.q2, self.q3)
		time.sleep(4)
		self.get_position_joints_PSM()
		self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
		print("\n")
		print("FK")
		print(self.x_fk, self.y_fk, self.z_fk)
		print(self.jacobian(self.q1, self.q2, self.q3))

	def temp01(self, qt1, qt2, qt3):
		
		#try this for every joint, once per time
		self.q1 = 0
		self.q2 = 0
		self.set_position_robot(self.q1, self.q2, self.q3)
		self.get_position_joints_PSM()
		self.j1_read = np.append(self.j1_read, self.q1)

	
	def prescribed_sine(self):

		times = 0
		while times < 1:
			
			step = 0.1

			for angle in np.arange(0, 360+step, step):
				q2 = math.radians(30*np.sin(math.radians(angle)))
				self.graph_q2_set0 = np.append(self.graph_q2_set0, q2)
				print(angle)
				self.set_position_robot(0, q2, 0.12)
				self.get_position_joints_PSM()
				self.j2_read = np.append(self.j2_read, self.q2_read)
			times = times + 1
			time.sleep(2)
			self.set_position_robot(0, 0, 0.12)
			
		fig = plt.figure()
		plt.plot(self.graph_q2_set0)
		plt.plot(self.j2_read, color = 'y')
		plt.grid()
		plt.show()
		
	def mainInsertionLink(self):
		step = 0.5
		for angle in np.arange(0.09, 360+step, step):
			q3 = 0.18+0.06*(np.sin(math.radians(angle)))
			self.graph_q3_set0 = np.append(self.graph_q3_set0, q3)
			print(angle)
			self.set_position_robot(0, 0, q3)
			self.get_position_joints_PSM()
			self.j3_read = np.append(self.j3_read, self.q3_read)
		
		time.sleep(2)
		fig = plt.figure()
		
		plt.figure()
		plt.plot(self.graph_q3_set0)
		plt.plot(self.j3_read, color = 'y')
		plt.grid()
		plt.show()
		#self.set_position_robot(0, 0, 0.12)

	def mainInsertionLinkIK(self):
		step = 0.5
		for angle in np.arange(0.09, 360+step, step):
			q3 = -0.18+0.06*(np.sin(math.radians(angle)))
			self.graph_q3_set0 = np.append(self.graph_q3_set0, q3)
			print(angle)
			self.inverse_kinematics(0, 0, q3)
			self.set_position_robot(0, 0, self.q3)
			self.get_position_joints_PSM()
			self.forward_kinematics(0,0,self.q3_read)
			self.j3_read = np.append(self.j3_read, self.z_fk)
		
		time.sleep(2)
		fig = plt.figure()
		
		plt.figure()
		plt.plot(self.graph_q3_set0)
		plt.plot(self.j3_read, color = 'y')
		plt.grid()
		plt.show()
		#self.set_position_robot(0, 0, 0.12)

	def exert_sin_force(self, m_start, X_desired, Y_desired, begin):
		'''
		#kind of sinus
		Kps = 0.0007
		Kis = 0.000001
		'''
		Kps = 0.003
		Kis = 0.00001
		#Kis = 0
		self.approach_goal_Z(m_start)
		x_fixed = 0.00000001
		self.reach_pos_XY(x_fixed,0.01, True)
		dy = 0.01
		dist_y = 0.01
		degree = -psm_handle_pfl.get_joint_pos(0)
		while degree <= math.radians(10):
			self.reach_pos_XY(0,dist_y,False)
			degree = -psm_handle_pfl.get_joint_pos(0)
			print(degree*180/3.14)
			dist_y = dist_y + dy
		X_desired = x_fixed
		Y_desired = dist_y
		print("STARTING SINUSOID IN 2 SECONDS")
		#time.sleep(2)
		force_base = self.force_const
		count = 0
		window = []
		window_size = 10
		sum = 0
		count1 = 0
		
		step = 0.5
		times = 0
		angle = 0
		while times<3:
			
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

			if angle >= 360:
				angle = 0
				times = times + 1
			angle = angle + step
			force_target = force_base + self.amplitude*np.sin(math.radians(angle))
			
			
			error = force_target - self.force
			e_rel = error/force_target
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
				self.q1 = self.q1 + delta_q[0]
				self.q2 = self.q2 + delta_q[1]
				self.q3 = self.q3 + delta_q[2]
				self.set_position_robot(self.q1, self.q2, self.q3)
				time.sleep(0.1)
				self.get_position_joints_PSM()
				self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
				#self.update_pos == True
			print("\n")
			print(angle)
			print(self.force, force_target)
			if self.update_pos == True:
				self.graph_f = np.append(self.graph_f, self.force)
			
			if self.update_pos == False:
				print(self.force)
				if (self.force < (self.force_const + self.band)) and (self.force > (self.force_const - self.band)):
					self.count_mi_loop = self.count_mi_loop + 1
					print("waiting...", self.count_mi_loop)
				if self.count_mi_loop == 10:
					count_mi_loop = 0
					self.update_pos = True

	def exert_sin_force1(self, m_start):
		
		set_angle = 20
		print("SET INCLINATION")
		psm_handle_pfl.set_joint_pos(0,math.radians(-set_angle))
		time.sleep(0.5)
		self.approach_goal_Z(m_start)
		'''
		Kp = 0.0001 #good for step = 0.5
		Ki = 0.0000000005
		

		Kps = 0.025 #good for step = 2.5
		Kis = 0.0000000008
		'''
		Kps = 0.02 #good for step = 5
		Kis = 0.0000000008

		print("STARTING SINUSOID IN 2 SECONDS")
		time.sleep(2)
		force_base = self.force_const
		count = 0
		window = []
		window_size = 10
		sum = 0
		count1 = 0
		
		step = 5
		times = 0
		angle = 0
		while times<1:
			
			self.time_start_a = time.time()
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

			if self.update_pos == True:
				if angle >= 360:
					angle = 0
					times = times + 1
				angle = angle + step
				force_target = force_base + self.amplitude*np.sin(math.radians(angle))
			if self.update_pos == False:
				force_target = force_base
			
			
			error = force_target - self.force
			e_rel = error/force_target
			self.P_value = (Kps * error)
		
			self.Integrator = self.Integrator + error
			self.I_value = self.Integrator * Kis
		
		
			PID = self.P_value + self.I_value 

			self.get_position_joints_PSM()
			self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
			X_desired = self.x_fk
			Y_desired = self.y_fk
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
				self.q1 = self.q1 + delta_q[0]
				self.q2 = self.q2 + delta_q[1]
				self.q3 = self.q3 + delta_q[2]
				self.set_position_robot(self.q1, self.q2, self.q3)
				time.sleep(0.1)
				self.get_position_joints_PSM()
				self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
				#self.update_pos == True
			print("\n")
			print(angle)
			print(self.force, force_target)
			if self.update_pos == True:
				self.graph_f = np.append(self.graph_f, self.force)
				self.graph_fd = np.append(self.graph_fd, force_target)
				self.count_time()
				self.error_force = np.append(self.error_force, e_rel)
				self.count_time_ef()
			
			if self.update_pos == False:
				print(self.force)
				if (self.force < (self.force_const + self.band)) and (self.force > (self.force_const - self.band)):
					self.count_mi_loop = self.count_mi_loop + 1
					print("waiting...", self.count_mi_loop)
				if self.count_mi_loop == 10:
					count_mi_loop = 0
					self.graph_f = np.append(self.graph_f, self.force)
					self.graph_fd = np.append(self.graph_fd, force_target)
					self.count_time()
					self.error_force = np.append(self.error_force, e_rel)
					self.count_time_ef()
					self.update_pos = True



		
			





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
	m_start = 0.17
	psm_handle_pel.set_joint_pos(0, m_start)
	

	cart_c = Cartesian_control()
	
	print("go in 2 seconds!!!")
	time.sleep(2)
	#cart_c.temp(math.radians(-20), math.radians(-30), 0.08)
	#cart_c.temp(math.radians(30), -0.0, 0.09)
	
	#cart_c.temp(0.0-0.08, 4.608079977738305e-05-0.04, -0.00042080074000368033-0.08)
	
	
	#cart_c.temp(-0.06, 0.04, 0.0)
	
	cart_c.approach_goal_Z(m_start)
	cart_c.reach_pos_XY(0.01, 0.02, True)
	cart_c.reach_pos_XY(-0.01, 0.04, False)
	#cart_c.reach_pos_XY(0.02, 0.02, False)
	#cart_c.reach_pos_XY(0.01, -0.01, False)
	#cart_c.reach_pos_XY(-0.03, -0.05, False)
	#begin = True
	#cart_c.exert_sin_force(m_start, 0, 0, begin)
	#cart_c.exert_sin_force1(m_start)


	
	#cart_c.temp01
 
	#cart_c.plots()
	#cart_c.temp(-0.15, 0.16, -0.16)
	
	print('STEP1')



	#time.sleep(5)
	
	
	
	cart_c.plots()


	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()
