#!/usr/bin/env python2.7
from __future__ import division
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

# fino a qua tutto uguale poi vedi funzione sotto
class Cartesian_control:
	
	xd_plot = []
	yd_plot = []
	zd_plot = []
	xr_plot = []
	yr_plot = []
	zr_plot = []
	time_plot = []
	px = []
	py = []
	pz = []
	q1_r = []
	q2_r = []
	q3_r = []

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
	graph_pz=[]
	error_abs = []

	graph_f2 = []
	error_force2 = []

	degree = 0
	delta = 0.6 
	delta_m = 0.00005
	delta_m_start = 0.0003
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

	posX = 0
	posY = 0
	posZ = 0
	pi = math.pi
	l_RCC = 0.4318
	l_tool_original = 0.4162
	l_tool = l_tool_original + 0.02

	T = np.zeros((4,4))

	graph_pxd =[]
	graph_pyd = []
	graph_pzd = []

	f = []
	fd = []
	z = []
	er = []



	#Kp_start = 0.000001
	#Ki_start = 0.000001

	amplitude = 0.5

	force_const = 2.5-amplitude

	deltat_a = 0
	time = []
	deltat_a_ef = 0
	time_ef = []

	Integrator = 0
	Derivator = 0
	time_now = 0

	flag_first_pos = True

	'''
	Kp = 0.0015 #good2  <--------
	Ki = 0.000008
	'''
	'''
	Kp = 0.0006 #test  <--------
	Ki = 0.00008
	'''
	'''
	Kp = 0.0015#0.0015 default #test  <--------
	Ki = 0.0004#0.0004 default
	'''
	
	Kp = 0.0006 #sponge soft  <--------
	Ki = 0.0004
	
	'''
	Kp = 0.0001
	Ki = 0.00000008  #bad
	'''
	

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

		q1 = math.asin((X_des)/(math.sqrt(math.pow(X_des,2)+math.pow(Z_des,2))))
		q2 = -math.asin(Y_des/r)
		q3 = self.l_RCC - self.l_tool + r

		return q1, q2, q3


	def set_position_robot(self, q1_set, q2_set, q3_set):
		
		#set position of the robot in simulation through joint values		
		psm_handle_base.set_joint_pos(0, q1_set)
		psm_handle_pfl.set_joint_pos(0, q2_set)
		psm_handle_pel.set_joint_pos(0, q3_set)


	def get_position_joints_PSM(self):
		
		#get the joint values of the robot from the simulation
		q1_read = psm_handle_base.get_joint_pos(0)
		q2_read = psm_handle_base.get_joint_pos(3)
		q3_read = psm_handle_base.get_joint_pos(4)

		return q1_read, q2_read, q3_read


	def forward_kinematics(self, q1, q2, q3):

		x_fk = math.cos(q2)*math.sin(q1)*(self.l_tool-self.l_RCC+q3)
		y_fk = -math.sin(q2)*(self.l_tool-self.l_RCC+q3)
		z_fk = -math.cos(q1)*math.cos(q2)*(self.l_tool-self.l_RCC+q3) 

		return x_fk, y_fk, z_fk


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

		return jac
		

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
		time2 = self.time_ef

		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_cart_time.csv', time2, delimiter=",")
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_cart_force.csv', self.graph_f2, delimiter=",") 
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_cart_error.csv', self.error_force2, delimiter=",")
	
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
		axs[2].set(ylabel = 'posx_error [m]')
		axs[2].legend(loc='best')
		axs[2].grid()

		axs[3].plot(time, self.er_y, label = "err_posy")
		axs[3].set(ylabel = 'posy_error [m]')
		axs[3].legend(loc='best')
		axs[3].grid()
		'''
		axs[4].plot(time, self.er_z, label = "err_posz")
		axs[4].set(ylabel = 'posz_error [m]')
		axs[4].set(xlabel = 'Time [s]')	
		axs[4].legend(loc='best')
		axs[4].grid()
		'''
		axs[4].plot(time, self.graph_pz, label = "posz")
		axs[4].set(ylabel = 'position z [m]')
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


	def plot_force(self):

		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_cart_time.csv', time2, delimiter=",")
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_cart_force.csv', self.graph_f2, delimiter=",") 
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_sp_cart_error.csv', self.er, delimiter=",")

		time = []
		time = self.time
		time_ef = []
		time2 = self.time_ef

		#fdim = 12
		font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 18}

		matplotlib.rc('font', **font)
		matplotlib.rc('legend',fontsize=15)
			
		fig, axs = plt.subplots(nrows = 4, sharex=True)
		fig.subplots_adjust(hspace=0.25)
		

		axs[0].plot(time, self.graph_f, color = 'r', label = "actual force")
		axs[0].plot(time, self.graph_fd, color = 'b', label = "target force")
		axs[0].set(ylabel = 'Force [N]')
		#axs[0].set_title('SPONGE', fontsize=24)
		#axs[0].set_xticklabels([],rotation=0, fontsize=1)
		#axs[0].tick_params(labelsize=fdim)	
		axs[0].legend(loc='best')
		axs[0].grid()

		axs[1].plot(time, self.error_abs, color = 'r', label = "abs_error")
		axs[1].set(ylabel = 'Abs_F_err [N]')
		#axs[1].set_xticklabels([],rotation=0, fontsize=1)
		#axs[1].tick_params(labelsize=fdim)
		axs[1].legend(loc='best')
		axs[1].grid()

		axs[2].plot(time, self.graph_px, label = "posX")
		axs[2].plot(time, self.graph_py, label = "posY")
		axs[2].set(ylabel = 'pos_xy [m]')
		#axs[3].set(xlabel = 'Time [s]')	
		#axs[3].set_xticklabels([],rotation=0, fontsize=1)
		#axs[3].tick_params(labelsize=fdim)
		
		axs[2].legend(loc='best')
		axs[2].grid()

		#axs[3].plot(time, self.graph_pz, color = 'r', label = "posz" )
		axs[3].plot(time, self.pz, color = 'g', label = "posZ")
		axs[3].set(ylabel = 'pos_z [m]')
		#axs[1].set(xlabel = 'Time [s]')	
		#axs[1].set_xticklabels([],rotation=0, fontsize=1)
		#axs[1].tick_params(labelsize=fdim)	
		axs[3].legend(loc='best', fontsize = 'small')
		axs[3].set(xlabel = 'Time [s]')	
		axs[3].grid()
		'''
		axs[4].plot(time, self.er_z, color = 'g', label = "err_posz")
		axs[4].set(ylabel = 'Z_er [mm]')
		axs[4].set(xlabel = 'Time [s]')	
		#axs[4].set_xticklabels([],rotation=0, fontsize=1)	
		#axs[4].tick_params(labelsize=fdim)
		axs[4].set(xlabel = 'Time [s]')
		axs[4].legend(loc='3', fontsize = 'small')
		axs[4].grid()
		'''
	
		plt.show()


	def plot_complete1(self):

		time = []
		time = self.time
		time_ef = []
		time2 = self.time_ef

		fig, axs = plt.subplots(nrows = 4, sharex=True)
		fig.subplots_adjust(hspace=0.25)

		axs[0].plot(time, self.graph_px, color = 'b', label = "posx")
		axs[0].plot(time, self.graph_py, color = 'r', label = "posy")
		axs[0].set(ylabel = 'pos_X_Y [cm]')
		#axs[0].set(xlabel = 'Time [s]')	
		#axs[0].set_xticklabels([],rotation=0, fontsize=1)
		#axs[0].tick_params(labelsize=fdim)	
		axs[0].legend(loc='best', fontsize = 'small')
		axs[0].grid()

		axs[1].plot(time, self.graph_pz, color = 'g', label = "posz" )
		axs[1].set(ylabel = 'pos_z [cm]')
		#axs[1].set(xlabel = 'Time [s]')	
		#axs[1].set_xticklabels([],rotation=0, fontsize=1)
		#axs[1].tick_params(labelsize=fdim)	
		axs[1].legend(loc='best', fontsize = 'small')
		axs[1].grid()	
	
		axs[2].plot(time, self.er_x, color = 'b', label = "err_posx")
		axs[2].plot(time, self.er_y, color = 'r', label = "err_posy")
		axs[2].set(ylabel = 'X_Y_er [mm]')
		#axs[2].set_xticklabels([],rotation=0, fontsize=1)
		#axs[2].tick_params(labelsize=fdim)	
		axs[2].legend(loc='3', fontsize = 'small')
		axs[2].grid()
	
		axs[3].plot(time, self.er_z, color = 'g', label = "err_posz")
		axs[3].set(ylabel = 'Z_er [mm]')
		axs[3].set(xlabel = 'Time [s]')	
		#axs[4].set_xticklabels([],rotation=0, fontsize=1)	
		#axs[4].tick_params(labelsize=fdim)
		axs[3].set(xlabel = 'Time [s]')
		axs[3].legend(loc='3', fontsize = 'small')
		axs[3].grid()

	
		plt.show()

	






	
	#this faction to approach an object. The position of main insertion link only is increased in the joint space, while
	#keping fixed the position of the other joints previously set. The position is increased at every iteration of a certain amount
	#if the force read is lower than the target force, it is decreased of the same amount if the force read is higher than the target
	#force. Once the target force is reached, if the force read remains within a bandwidth for a specified number of iteration, exit the loop.

	
	def approach_goal_Z(self, m_start):
		f = 50
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
			if self.count_mi_loop == 5:
				self.count_mi_loop = 0
				break
			
			psm_handle_pel.set_joint_pos(0, self.m)
			force_old2 = force_old1
			force_old1 = self.force

			self.graph_f = np.append(self.graph_f, self.force)
			self.graph_fd = np.append(self.graph_fd, self.force_const)
			self.error_force = np.append(self.error_force, 0)
			#self.count_time_ef()

			
			self.count_time()		
			self.graph_px = np.append(self.graph_px, 0)
			self.graph_py = np.append(self.graph_py, 0)
			self.graph_pz = np.append(self.graph_pz, -0.2051)#soft
			#self.graph_pz = np.append(self.graph_pz,-0.21688)
			PID = 1

			self.graph_frn = np.append(self.graph_frn, force_raw_now)



			self.px = np.append(self.px, 0)
			self.py = np.append(self.py, 0)
			#self.pz = np.append(self.pz, -0.23)
			self.pz = np.append(self.pz, -0.2051)
			self.q1_r = np.append(self.q1_r, 0)
			self.q2_r = np.append(self.q2_r, 0)
			self.q3_r = np.append(self.q3_r, 0)
			self.graph_pxd = np.append(self.graph_pxd, 0)
			self.graph_pyd = np.append(self.graph_pyd, 0)
			self.graph_pzd = np.append(self.graph_pzd, -0.2051)



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
			self.er_x = np.append(self.er_x, 0)
			ey = 0
			self.er_y = np.append(self.er_y, 0)
			ez = 0
			self.er_z = np.append(self.er_z, 0)
			self.error_abs = np.append(self.error_abs, 0)

			wait = 1/f - (time.time() - self.time_start_a) 
			if wait>0:
				time.sleep(wait)

	#Cartesian control is applied here, reaching an (x,y) position trying to keep constant the force in z direction. 
	#Initially, a start boolean is specified: if it is True, it means that is the first time this function is called. So, out of the internal 
	#cycle I am updating the desired position. The difference between the goal and the starting point is computed both for x and y, then the 
	#longer path (difference) between the two is found. The desired longer delta space is set, so that is possible to determine also the shortest one.
	#Having now both the increments dx and dy it is possible to increase the values of x_desired and y_desired at every iteration until the goal is reached. 
	
	def define_path(self, goal_x, goal_y):
		print("Processing:  Defining cartesian path ....")
		
		time_vect = []
		posx_vect = []
		posy_vect = []
		posz_vect = []

		time_el = 0
		for i in range(0, self.f_cycle*self.exp_time):
			time_vect = np.append(time_vect, time_el)
			time_el = time_el + 1/self.f_cycle

		time.sleep(0.5)
		q1_read, q2_read, q3_read = self.get_position_joints_PSM()
		
		x_el, y_el, z_el = self.forward_kinematics(q1_read, q2_read, q3_read)
	
		
		dx = (goal_x - x_el)/time_vect.size
		for i in range(0, self.f_cycle*self.exp_time):
			posx_vect = np.append(posx_vect, x_el)
			x_el = x_el + dx

		dy = (goal_y - y_el)/time_vect.size
		for i in range(0, self.f_cycle*self.exp_time):
			posy_vect = np.append(posy_vect, y_el)
			y_el = y_el + dy

		'''
		dz = (goal_z - z_el)/time_vect.size
		for i in range(0, self.f_cycle*self.exp_time):
			posz_vect = np.append(posz_vect, z_el)
			z_el = z_el + dz
		'''
		

		return time_vect, posx_vect, posy_vect
		
	
	def compute_inverse_kinematics(self, x_vect, y_vect, z_vect):
		
		print("Processing:  IK computation ....")
		q1_vect = []
		q2_vect = []
		q3_vect = []

		for i in range(0, x_vect.size):
			q1, q2, q3 = self.inverse_kinematics(x_vect[i], y_vect[i], z_vect[i])
			q1_vect = np.append(q1_vect, q1)
			q2_vect = np.append(q2_vect, q2)
			q3_vect = np.append(q3_vect, q3)

		return q1_vect, q2_vect, q3_vect

		'''
		#print(q1_vect)
		print("length q1:", q1_vect.size)
		print("length q2:", q2_vect.size)
		print("length q3:", q3_vect.size)
		'''

	def compute_forward_kinematics(self, time_vect, x_vect, y_vect, z_vect):
		
		print("Processing:  FK computation ....")
		x1_vect = []
		x2_vect = []
		x3_vect = []

		for i in range(0, time_vect.size):
			x1, x2, x3 = self.forward_kinematics(x_vect[i], y_vect[i], z_vect[i])
			x1_vect = np.append(x1_vect, x1)
			x2_vect = np.append(x2_vect, x2)
			x3_vect = np.append(x3_vect, x3)

		return x1_vect, x2_vect, x3_vect

		'''
		#print(q1_vect)
		print("length q1:", q1_vect.size)
		print("length q2:", q2_vect.size)
		print("length q3:", q3_vect.size)
		'''


	def reach_XY_force_control(self, goal_x, goal_y):

		self.f_cycle = 40
		self.exp_time = 10
		dim = self.f_cycle*self.exp_time

		time_v, x_v, y_v = self.define_path(goal_x, goal_y)
		q1_r,q2_r,q3_r = self.get_position_joints_PSM()
		time.sleep(1)
	
		

		print("Moving arm ....")
		j=0
		time_now = 0
		new = True
		#starttime=time.time()
		self.time_start_a = time.time()

		count = 0
		window = []
		window_size = 10
		sum = 0
		count1 = 0

		z_v = np.zeros(dim)
		xfk = np.zeros(dim)
		yfk = np.zeros(dim)
		zfk = np.zeros(dim)
		px_v = np.zeros(dim)
		py_v = np.zeros(dim)
		pz_v = np.zeros(dim)
		self.graph_f_cycle = np.zeros(dim)
		self.graph_fd_cycle = np.zeros(dim)
		self.error_force_cycle = np.zeros(dim)
		er_a = np.zeros(dim)
		q1_r = np.zeros(dim)
		q2_r = np.zeros(dim)
		q3_r = np.zeros(dim)
		flag = 0

		print(zfk[0])

		while(j<self.f_cycle*self.exp_time):

			starttime=time.time()
			self.count_time()
			self.count_time_ef()
			self.time_start_a = time.time()

			q1_r,q2_r,q3_r[j] = self.get_position_joints_PSM()
			xfk[j],yfk[j],zfk[j] = self.forward_kinematics(q1_r,q2_r,q3_r[j])
			
			#################################################################
			pos = psm_handle_trl.get_pos()  #if I want Z from AMBF function
			px = pos.x
			py = pos.y
			pz = pos.z

			px_v[j], py_v[j], pz_v[j] = self.from_AMBF2PSM_RF(px,py,pz)
			#################################################################

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
			er_a[j] = abs(error)
			e_rel = error/self.force_const
			self.P_value = (self.Kp * error)
		
			self.Integrator = self.Integrator + error
			self.I_value = self.Integrator * self.Ki
				
			PID = self.P_value + self.I_value
			#zd = zfk[j] + PID*zfk[j]
			zd = pz_v[j] + PID*pz_v[j]

			z_v[j] = zd

			q1,q2,q3 = self.inverse_kinematics(x_v[j],y_v[j], z_v[j])
		
			self.er_z = np.append(self.er_z, z_v[j]-pz_v[j])
			self.set_position_robot(q1,q2,q3)

			self.graph_f_cycle[j] = self.force
			self.graph_fd_cycle[j] = self.force_const
			self.error_force_cycle[j] = e_rel

			
			print(self.force)
			
			j=j+1

			wait = 1/self.f_cycle - (time.time() - starttime) 
			if wait>0:
				time.sleep(wait)
			#print(time.time()-starttime)

			
		self.graph_px = np.append(self.graph_px, x_v)
		self.graph_py = np.append(self.graph_py, y_v)
		self.graph_pz = np.append(self.graph_pz, z_v)
		self.graph_pxd = np.append(self.graph_pxd, xfk)
		self.graph_pyd = np.append(self.graph_pyd, yfk)
		self.graph_pzd = np.append(self.graph_pzd, pz_v)
		self.z = np.append(self.z, pz_v)


		self.px = np.append(self.px, px_v)
		self.py = np.append(self.py, py_v)
		self.pz = np.append(self.pz, pz_v)
		self.q1_r = np.append(self.q1_r, q1_r)
		self.q2_r = np.append(self.q2_r, q2_r)
		self.q3_r = np.append(self.q3_r, q3_r)
	
	
		



		for i in range (0,dim):

			self.er_x = np.append(self.er_x, x_v[i]-xfk[i])
			self.er_y = np.append(self.er_y, y_v[i]-yfk[i])
			
			self.graph_f = np.append(self.graph_f, self.graph_f_cycle[i])
			self.graph_fd = np.append(self.graph_fd, self.graph_fd_cycle[i])
			self.error_force = np.append(self.error_force, self.error_force_cycle[i])		
			self.graph_f2 = np.append(self.graph_f2, self.graph_f_cycle[i])
			self.error_force2 = np.append(self.error_force2, self.error_force_cycle[i])
			self.error_abs = np.append(self.error_abs, er_a[i])
			self.f = np.append(self.f, self.graph_f_cycle[i])
			self.fd = np.append(self.fd, self.graph_fd_cycle[i])
			self.er = np.append(self.er, er_a[i])
			



	

	def plot_properties(self):

		time = []
		time = self.time_ef

		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kLST_t.csv', time, delimiter=",")
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kLST_f.csv', self.f, delimiter=",") 
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kLST_fd.csv', self.fd, delimiter=",")
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kLST_er.csv', self.er, delimiter=",")
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kLST_z.csv', self.z, delimiter=",")

		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kMT_t.csv', time, delimiter=",")
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kMT_f.csv', self.f, delimiter=",") 
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kMT_fd.csv', self.fd, delimiter=",")
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kMT_er.csv', self.er, delimiter=",")
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kMT_z.csv', self.z, delimiter=",")

		font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 18}

		matplotlib.rc('font', **font)
		matplotlib.rc('legend',fontsize=15)
			
		fig, axs = plt.subplots(nrows = 3, sharex=True)
		fig.subplots_adjust(hspace=0.25)

		axs[0].plot(time, self.f, color = 'r', label = "F read")
		axs[0].plot(time, self.fd, color = 'b', label = "F des")
		#axs[0].set_title('SPONGE', fontsize=24)
		axs[0].set(ylabel = 'Force [N]')	
		axs[0].legend(loc='best')
		axs[0].grid()
		
		axs[1].plot(time, self.er, color = 'r', label = "abs force err")		
		axs[1].set(ylabel = 'Abs_err [N]')	
		axs[1].legend(loc='best')
		axs[1].grid()

		axs[2].plot(time, self.z, color = 'g', label = "Z pos")
		axs[2].set(ylabel = 'Z pos [m]')
		axs[2].legend(loc='best')
		axs[2].grid()

		'''
		axs[3].plot(time, self.er_z, color = 'g', label = "err_posz")
		axs[3].set(ylabel = 'z_err [m]')
		axs[3].set(xlabel = 'Time [s]')	
		axs[3].legend(loc='best')
		axs[3].grid()
		'''

		plt.show()














	def plot_positions(self):
		
		time = []
		time = self.time
		time_ef = []
		time_ef = self.time_ef
	
		font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 18}

		matplotlib.rc('font', **font)
		matplotlib.rc('legend',fontsize=15)
			
		fig, axs = plt.subplots(nrows = 4, sharex=True)
		fig.subplots_adjust(hspace=0.25)

		axs[0].plot(time, self.graph_pxd, color = 'r', label = "actual x")
		axs[0].plot(time, self.graph_px, color = 'b', label = "target x")
		axs[0].plot(time, self.graph_pyd, color = 'g', label = "actual y")
		axs[0].plot(time, self.graph_py, color = 'm', label = "target y")
		axs[0].set_title('SPONGE', fontsize=24)
		axs[0].set(ylabel = 'Pos_x [m]')	
		axs[0].legend(loc='upper left')
		axs[0].grid()
		'''
		axs[1].plot(time, self.graph_pyd, color = 'm', label = "actual y")
		axs[1].plot(time, self.graph_py, color = 'v', label = "target y")
		axs[1].set(ylabel = 'Pos_y [m]')	
		axs[1].legend(loc='best')
		axs[1].grid()
		'''
		
		axs[1].plot(time, self.graph_pzd, color = 'r', label = "actual z")
		axs[1].plot(time, self.graph_pz, color = 'b', label = "target z")
		axs[1].set(ylabel = 'Pos_z [m]')	
		axs[1].legend(loc='best')
		axs[1].grid()

		axs[2].plot(time, self.er_x, color = 'b', label = "err_posx")
		axs[2].plot(time, self.er_y, color = 'r', label = "err_posy")
		axs[2].set(ylabel = 'x_y_err [m]')
		axs[2].legend(loc='best')
		axs[2].grid()

		axs[3].plot(time, self.er_z, color = 'g', label = "err_posz")
		axs[3].set(ylabel = 'z_err [m]')
		axs[3].set(xlabel = 'Time [s]')	
		axs[3].legend(loc='best')
		axs[3].grid()

		plt.show()

	def	issueZ(self):


		time = []
		time = self.time
		time_ef = []
		time_ef = self.time_ef

		font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 18}

		matplotlib.rc('font', **font)
		matplotlib.rc('legend',fontsize=15)
			
		fig, axs = plt.subplots(nrows = 3, sharex=True)
		fig.subplots_adjust(hspace=0.25)
		
		axs[0].plot(time, self.graph_px , color = 'b', label = "x commanded")
		axs[0].set(ylabel = 'X [m]')	
		axs[0].legend(loc='best')
		axs[0].grid()

	
		axs[1].plot(time, self.graph_py , color = 'b', label = "y commanded")
		axs[1].set(ylabel = 'Y [m]')
		axs[1].legend(loc='best')
		axs[1].grid()

		axs[2].plot(time,self.pz, color = 'r', label = "z AMBF")
		axs[2].plot(time, self.graph_pz, color = 'g', label = "z FK")
		axs[2].set(xlabel = 'Time [s]', ylabel = 'Z [m]')	
		axs[2].legend(loc='best')
		axs[2].grid()

		plt.show()



	def define_path_cal(self, goal_x, goal_y, goal_z,start):
		print("Processing:  Defining cartesian path ....")
		
		time_vect = []
		posx_vect = []
		posy_vect = []
		posz_vect = []

		time_el = 0
		for i in range(0, self.f_cycle*self.exp_time):
			time_vect = np.append(time_vect, time_el)
			time_el = time_el + 1/self.f_cycle

		time.sleep(2)
		q1_read, q2_read, q3_read = self.get_position_joints_PSM()

		print("EEEEEEEEEEEEEEEIIIIIIIIII")
		print(q1_read, q2_read, q3_read)
		print("\n")
		
		x_el, y_el, z_el = self.forward_kinematics(q1_read, q2_read, q3_read)
	
		
		dx = (goal_x - x_el)/time_vect.size
		for i in range(0, self.f_cycle*self.exp_time):
			posx_vect = np.append(posx_vect, x_el)
			x_el = x_el + dx

		dy = (goal_y - y_el)/time_vect.size
		for i in range(0, self.f_cycle*self.exp_time):
			posy_vect = np.append(posy_vect, y_el)
			y_el = y_el + dy

		dz = (goal_z - z_el)/time_vect.size
		for i in range(0, self.f_cycle*self.exp_time):
			posz_vect = np.append(posz_vect, z_el)
			z_el = z_el + dz
		
		

		return posx_vect, posy_vect, posz_vect



	
	def reach_XY_pos_control_cal(self, goal_x, goal_y, goal_z, start):

		self.f_cycle = 50
		self.exp_time = 0.5

		x_v, y_v, z_v = self.define_path_cal(goal_x, goal_y, goal_z,True)
		q1_v, q2_v, q3_v = self.compute_inverse_kinematics(x_v, y_v, z_v)		

		print("Moving arm for calibration ....")
		j=0
		time_now = 0
		flag = 0

	
		self.time_start_a = time.time()

		while(j<self.f_cycle*self.exp_time):

			#self.count_time()
			starttime=time.time()
			self.time_start_a = time.time()

			self.set_position_robot(q1_v[j], q2_v[j], q3_v[j])			

			q1_r,q2_r,q3_r = self.get_position_joints_PSM()
			xfk,yfk,zfk = self.forward_kinematics(q1_r,q2_r,q3_r)
		
			self.set_position_robot(q1_v[j], q2_v[j], q3_v[j])
			
			q1_r,q2_r,q3_r = self.get_position_joints_PSM()
			xfk,yfk,zfk = self.forward_kinematics(q1_r,q2_r,q3_r)

			j=j+1					
			
			time.sleep(1/self.f_cycle)
			
			print(xfk,yfk,zfk)

		return xfk, yfk, zfk


	def get_cart_pos_cal(self):

		q1_r,q2_r,q3_r = self.get_position_joints_PSM()
		xfk,yfk,zfk = self.forward_kinematics(q1_r,q2_r,q3_r)
		

		return xfk, yfk, zfk





	def calibration(self):

		#xcal, ycal, zcal = self.reach_XY_pos_control_cal(0,0,0, True)
		xcal, ycal, zcal = self.get_cart_pos_cal()
		time.sleep(3)

		pos = psm_handle_trl.get_pos()  #if I want Z from AMBF function
		tx = pos.x
		ty = pos.y
		tz = pos.z

		T = np.zeros((4,4))

		T[0,0] = -1
		T[0,1] = 0
		T[0,2] = 0
		T[0,3] = tx - xcal
		T[1,0] = 0
		T[1,1] = -1
		T[1,2] = 0
		T[1,3] = ty - ycal
		T[2,0] = 0
		T[2,1] = 0
		T[2,2] = 1
		T[2,3] = tz - zcal
		T[3,0] = 0
		T[3,1] = 0
		T[3,2] = 0
		T[3,3] = 1

		self.T = np.linalg.inv(T) 
		print(self.T)
		print("CALIBRATED!!!")


	
	def from_AMBF2PSM_RF(self,x0,y0,z0):

		vect0 = np.ones(4)
		vect1 = np.ones(4)
		vect0[0] = x0
		vect0[1] = y0
		vect0[2] = z0

		vect1 = np.dot(self.T, vect0)

		return(vect1[0], vect1[1], vect1[2])	


	
	
def main():

	psm_handle_pel.set_joint_pos(0, 0)
	raw_input("Number of joints of pitchfrontLink")
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

	cart_c = Cartesian_control()

	psm_handle_pel.set_joint_pos(0, 0)
	psm_handle_pfl.set_joint_pos(0, 0)
	psm_handle_base.set_joint_pos(0, math.radians(0))
	time.sleep(2)
	psm_handle_pel.set_joint_pos(0, 0)
	time.sleep(1)
	
	#cart_c.calibration()

	psm_handle_pel.set_joint_pos(0, 0)
	
	m_start =  0.135#0.155 cheese3
	psm_handle_pel.set_joint_pos(0, m_start)
	time.sleep(2)
	

	
	
	print("go in 2 seconds!!!")
	#time.sleep(2)
	'''
	cart_c.approach_goal_Z(m_start)
	cart_c.reach_pos_XY(0.01, 0.02, True)
	cart_c.reach_pos_XY(-0.01, 0.04, False)
	cart_c.reach_pos_XY(0.02, 0.02, False)
	'''
	#cart_c.reach_pos_XY(0.01, -0.01, False)
	#cart_c.reach_pos_XY(-0.03, -0.05, False)
	
	
	cart_c.approach_goal_Z(m_start)
	cart_c.calibration()
	'''
	cart_c.reach_XY_force_control(0.09, 0.05)
	cart_c.reach_XY_force_control(0.05, -0.02)
	cart_c.reach_XY_force_control(0.01, 0.10)
	
	cart_c.reach_XY_force_control(0.05,0.02)
	cart_c.reach_XY_force_control(-0.02,0.04)
	cart_c.reach_XY_force_control(0.10,-0.02)
	'''

	#cart_c.reach_XY_force_control(0.05, 0.1)
	#cart_c.reach_XY_force_control(0.01,-0.08)


	########this two!!!!!!!!
	cart_c.reach_XY_force_control(0.01, 0.1)
	cart_c.reach_XY_force_control(0.1,-0.02)
	#cart_c.reach_XY_force_control(0.01, 0.1)
	#cart_c.reach_XY_force_control(0.1,-0.02)

	#cart_c.reach_XY_force_control(0.07, 0.07)

	#cart_c.issueZ()


	#cart_c.plot_new0()
	
	

	
	print('STEP1')	

	cart_c.plot_force()
	#time.sleep(1)
	#cart_c.plot_positions()

	#cart_c.plot_properties()
	

	#raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()



'''
FREQUENZA_CICLO=100Hz
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

















m_start =  0.155#0.155 cheese3
