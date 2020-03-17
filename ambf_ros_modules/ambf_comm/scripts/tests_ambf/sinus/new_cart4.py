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

	degree = 0
	delta = 0.6 
	delta_m = 0.00005
	delta_m_start = 0.00005
	band = 0.05
	band2 = 0.5
	limit_mi = 0.30
	update_pos = False

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

	#Kp_start = 0.000001
	#Ki_start = 0.000001

	amplitude = 0.5

	force_const = 1.5-amplitude

	deltat_a = 0
	time = []
	deltat_a_ef = 0
	time_ef = []

	Integrator = 0
	Derivator = 0
	time_now = 0

	flag_first_pos = True

	'''
	Kp = 0.008 #stiffer
	Ki = 0.00008
	'''

	Kp = 0.005 #rqt_plot
	Ki = 0.000001

	Integrator = 0
	Integratorx = 0
	Integratory = 0
	Integratorz = 0
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


	def set_position_robot(self, q1_set, q2_set, q3_set):
		
		#set position of the robot in simulation through joint values		
		psm_handle_base.set_joint_pos(0, q1_set)
		psm_handle_pfl.set_joint_pos(0, q2_set)
		psm_handle_pel.set_joint_pos(0, q3_set)


	def get_position_joints_PSM(self):
		
		#get the joint values of the robot from the simulation
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
		

	def count_time(self):
		time_end_a = time.time()
		self.deltat_a = (time_end_a-self.time_start_a) + self.deltat_a 
		self.time = np.append(self.time, self.deltat_a)

	def count_time_ef(self):
		time_end_a_ef = time.time()
		self.deltat_a_ef = (time_end_a_ef-self.time_start_a) + self.deltat_a_ef 
		self.time_ef = np.append(self.time_ef, self.deltat_a_ef)

	def plots(self):
		
		time = []
		time = self.time
		time_ef = []
		time_ef = self.time_ef
	
		fig, axs = plt.subplots(nrows = 6)

		axs[0].plot(time, self.graph_f, color = 'r', label = "actual force")
		axs[0].plot(time, self.graph_fd, color = 'b', label = "target force")
		axs[0].set(ylabel = 'Force [N]')	
		axs[0].legend(loc='best')
		axs[0].grid()

		axs[1].plot(time, self.error_force, color = 'r', label = "error")
		axs[1].set(ylabel = 'Force_error_norm')
		axs[1].legend(loc='best')
		axs[1].grid()

		axs[2].plot(time, self.er_x, label = "err_posx")
		axs[2].set(ylabel = 'pox_error [m]')
		axs[2].legend(loc='best')
		axs[2].grid()

		axs[3].plot(time, self.er_y, label = "err_posy")
		axs[3].set(ylabel = 'posy_error [m]')
		axs[3].legend(loc='best')
		axs[3].grid()

		axs[4].plot(time, self.er_z, label = "err_posz")
		axs[4].set(ylabel = 'posz_error [m]')
		axs[4].set(xlabel = 'Time [s]')	
		axs[4].legend(loc='best')
		axs[4].grid()

		axs[5].plot(time, self.graph_px, label = "posx")
		axs[5].plot(time, self.graph_py, label = "posy")
		axs[5].set(ylabel = 'position xy [m]')
		axs[5].set(xlabel = 'Time [s]')	
		axs[5].legend(loc='best')
		axs[5].grid()

		plt.show()

	'''
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
	'''
	
	#this faction to approach an object. The position of main insertion link only is increased in the joint space, while
	#keping fixed the position of the other joints previously set. The position is increased at every iteration of a certain amount
	#if the force read is lower than the target force, it is decreased of the same amount if the force read is higher than the target
	#force. Once the target force is reached, if the force read remains within a bandwidth for a specified number of iteration, exit the loop.

	def approach_goal_Z(self, m_start):
		self.m = m_start
		force_old2 = 0
		force_old1 = 0
		while self.m < self.limit_mi:
			
			self.time_start_a = time.time()
			#_,_,force_raw_now = psm_handle_mi.get_force()
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
			self.graph_fd = np.append(self.graph_fd, self.force_const)
			self.error_force = np.append(self.error_force, 0)
			self.count_time_ef()

			
			self.count_time()		
			self.graph_px = np.append(self.graph_px, 0)
			self.graph_py = np.append(self.graph_py, 0)
			PID = 1

			self.graph_frn = np.append(self.graph_frn, force_raw_now)


			#lines below to plot even x and y components of the force in the world reference frame
			'''
			fr_x,_,_ = psm_handle_mi.get_force()
			#fr_x = psm_handle_mi.get_force()
			self.fr_x = np.append(self.fr_x, fr_x)
			_,fr_y,_ = psm_handle_mi.get_force()
			#fr_y = psm_handle_mi.get_force()
			self.fr_y = np.append(self.fr_y, fr_y)
			_,_,fr_z = psm_handle_mi.get_force()
			#fr_z = psm_handle_mi.get_force()
			self.fr_z = np.append(self.fr_z, fr_z)
			'''

			ex = 0
			self.er_x = np.append(self.er_x, ex)
			ey = 0
			self.er_y = np.append(self.er_y, ey)
			ez = 0
			self.er_z = np.append(self.er_z, ez)
			
			

	#Cartesian control is applied here, reaching an (x,y) position trying to keep constant the force in z direction. 
	#Initially, a start boolean is specified: if it is True, it means that is the first time this function is called. So, out of the internal 
	#cycle I am updating the desired position. The difference between the goal and the starting point is computed both for x and y, then the 
	#longer path (difference) between the two is found. The desired longer delta space is set, so that is possible to determine also the shortest one.
	#Having now both the increments dx and dy it is possible to increase the values of x_desired and y_desired at every iteration until the goal is reached. 
	
	
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
		#d_step_long = 0.001
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
		

		#Here until the position x y haven't reached the goal position in x and y the following things are done:
		#	-filter the force read from the simulation with a moving average filter
		#	-find the delta movement in z direction sith a PI control of the filtered force
		#	-apply the control


		while (stop_x == False) or (stop_y == False):

			self.time_start_a = time.time()			
			#_,_,force_raw_now = psm_handle_mi.get_force()
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
			e_rel = error/self.force_const
			self.P_value = (self.Kp * error)
		
			self.Integrator = self.Integrator + error
			self.I_value = self.Integrator * self.Ki
		
		
			PID = self.P_value + self.I_value 

			self.get_position_joints_PSM()
			self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
			Z_desired_2 = self.z_fk
			Z_desired_2 = Z_desired_2 + PID*Z_desired_2

			#Now, find the desired joint positions and compute the jacobian. Then, if it is the first iteration, the first desired
			#position is set and then the actual joint position is read. If it is not the first iteration, first of all a PI control
			#is applied to the difference between the desired joint position and the read joint position. Then, the new obtained dq is 
			#added to the desired joint position, this new value is set to the robot, than the joint position is read again.
			
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
				dq = [0,0,0]

				Kpx = 0.9
				Kix = 0.05
				delta_q[0] = self.q1-self.q1_read
				self.P_value = (Kpx * delta_q[0])		
				self.Integratorx = self.Integratorx + delta_q[0]
				self.I_value = self.Integratorx * Kix		
				PID = self.P_value + self.I_value 
				dq[0] = delta_q[0] + PID*delta_q[0]

				Kpy = 0.9
				Kiy = 0.05
				delta_q[1] = self.q2-self.q2_read
				self.P_value = (Kpy * delta_q[1])		
				self.Integratory = self.Integratory + delta_q[1]
				self.I_value = self.Integratory * Kiy		
				PID = self.P_value + self.I_value 
				dq[1] = delta_q[1] + PID*delta_q[1]
				
				Kpz = 0.005
				Kiz = 0.0008	
				delta_q[2] = self.q3-self.q3_read
				self.P_value = (Kpz * delta_q[2])		
				self.Integratorz = self.Integratorz + delta_q[2]
				self.I_value = self.Integratorz * Kiz		
				PID = self.P_value + self.I_value 
				dq[2] = delta_q[2] + PID*delta_q[2]
				
				joint_q1 = self.q1 + dq[0]
				joint_q2 = self.q2 + dq[1]
				joint_q3 = self.q3 + dq[2]
				self.set_position_robot(joint_q1, joint_q2, joint_q3)
				time.sleep(0.1)
				self.get_position_joints_PSM()
				self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
				#self.update_pos == True



			#At the beginning, don't update the desired x and y positions until the force is within a certain bandwidth for
			#a defined number of iterations.

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

				'''
				fr_x,_,_ = psm_handle_mi.get_force()
				#fr_x = psm_handle_mi.get_force()
				self.fr_x = np.append(self.fr_x, fr_x)
				_,fr_y,_ = psm_handle_mi.get_force()
				#fr_y = psm_handle_mi.get_force()
				self.fr_y = np.append(self.fr_y, fr_y)
				_,_,fr_z = psm_handle_mi.get_force()
				#fr_z = psm_handle_mi.get_force()
				self.fr_z = np.append(self.fr_z, fr_z)
				'''


				#ex = (X_desired-self.x_fk)/X_desired
				ex = 0
				self.er_x = np.append(self.er_x, ex)
				#ey = (Y_desired-self.y_fk)/Y_desired
				ey = 0
				self.er_y = np.append(self.er_y, ey)
				#ez = (Z_desired_2-self.z_fk)/Z_desired_2
				ez = 0
				self.er_z = np.append(self.er_z, ez)
				


			#update desired cartesian positions

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

				'''
				fr_x,_,_ = psm_handle_mi.get_force()
				#fr_x = psm_handle_mi.get_force()
				self.fr_x = np.append(self.fr_x, fr_x)
				_,fr_y,_ = psm_handle_mi.get_force()
				#fr_y = psm_handle_mi.get_force()
				self.fr_y = np.append(self.fr_y, fr_y)
				_,_,fr_z = psm_handle_mi.get_force()
				#fr_z = psm_handle_mi.get_force()
				self.fr_z = np.append(self.fr_z, fr_z)
				'''

				ex = (X_desired-self.x_fk)#/X_desired
				self.er_x = np.append(self.er_x, ex)
				ey = (Y_desired-self.y_fk)#/Y_desired
				self.er_y = np.append(self.er_y, ey)
				ez = (Z_desired_2-self.z_fk)#/Z_desired_2
				self.er_z = np.append(self.er_z, ez)
				self.update_pos = True

				self.update_pos = True
				
			print("\n")
			print(X_desired)
			print(Y_desired)
			print("\n")
						
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
	psm_handle_pel.set_joint_pos(0, 0)
	time.sleep(1)
	psm_handle_pel.set_joint_pos(0, 0)
	m_start = 0.155
	psm_handle_pel.set_joint_pos(0, m_start)
	

	cart_c = Cartesian_control()
	
	print("go in 2 seconds!!!")
	time.sleep(2)
	'''
	cart_c.approach_goal_Z(m_start)
	cart_c.reach_pos_XY(0.01, 0.02, True)
	cart_c.reach_pos_XY(-0.01, 0.04, False)
	cart_c.reach_pos_XY(0.02, 0.02, False)
	'''
	#cart_c.reach_pos_XY(0.01, -0.01, False)
	#cart_c.reach_pos_XY(-0.03, -0.05, False)
	
	
	cart_c.approach_goal_Z(m_start)
	cart_c.reach_pos_XY(0.01, 0.06, True)
	cart_c.reach_pos_XY(-0.04, 0.06, False)
	cart_c.reach_pos_XY(-0.04, 0.01, False)
	

	
	print('STEP1')	
	
	
	cart_c.plots()


	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()



'''FREQUENZA_CICLO=100Hz
DURATA_ESPERIMENTO=10s
vettore_tempi=0:0.01:10 vettore di durata*frequenza elementi
vttore posizioni x= qualcosa che va calcolato, lungo ESATTAMENTE COME VETTORE TEMPI  
vttore posizioni y= qualcosa che va calcolato, lungo ESATTAMENTE COME VETTORE TEMPI
vttore posizioni z= qualcosa che va calcolato, lungo ESATTAMENTE COME VETTORE TEMPI (solo pler verificare controllo posizione)

p=1
i=1

def calcolo_cin_inversa():
	for i=0:length(vettore_tempi)
		q(i)=cin_inversa(vttore posizioni x(i),vttore posizioni y(i),vttore posizioni z(i))
	end

def move_to_joint_pos(self):
	j=0
	while(j<FREQUENZA_CICLO*DURATA_ESPERIMENTO)
		robot.goto(q(j))
		sleep(1/FREQUENZA_CICLO)
	end
stessa cosa  con controllo forza
def move_to_joint_pos(self):
	j=0
	integrale_errore_forza =0
	while(j<FREQUENZA_CICLO*DURATA_ESPERIMENTO)
		
		forza=leggi_forza()
		errore_forza=foza_desiderata-forza(z)
		
		integrale errore forza= integrale_errore_forza + (1/FREQUENZA_CICLO)*errore forza

		variazione_z=p*errore_forza+i*integrale_errore_forza
		z_desiderata=z_attuale+variazione_z
		q_new=cin_inversa(vttore posizioni x(j),vttore posizioni y(ij,z_desiderata)

		robot.goto(q_new)

		sleep(1/FREQUENZA_CICLO)
	end
	'''