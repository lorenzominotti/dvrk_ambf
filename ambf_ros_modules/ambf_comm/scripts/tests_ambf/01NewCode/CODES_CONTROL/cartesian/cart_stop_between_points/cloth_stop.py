#!/usr/bin/env python2.7
from __future__ import division
# Import the Client from ambf_client package
from ambf_client import Client
import time
import math
import rospy
from std_msgs.msg import Float64
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
	er = []

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
	delta_m_start = 0.0001
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
	l_tool = l_tool_original + 0.04

	T = np.zeros((4,4))

	graph_pxd =[]
	graph_pyd = []
	graph_pzd = []


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

	Kp = 0.001 #test new control pz
	Ki = 0.0008

	Integrator = 0
	Integratorx = 0
	Integratory = 0
	Integratorz = 0
	Derivator = 0
	time_now = 0

	flag_first_pos = True
	
	def __init__(self):
		pass

	def init_ROS(self):

		self.pub_t = rospy.Publisher('time', Float64, queue_size=10)
		self.pub_f = rospy.Publisher('force_read', Float64, queue_size=10)
		self.pub_fd = rospy.Publisher('force_desired', Float64, queue_size=10)
		self.pub_ea = rospy.Publisher('force_ea', Float64, queue_size=10)
		self.pub_er = rospy.Publisher('force_er', Float64, queue_size=10)
		self.pub_xd = rospy.Publisher('posx_d', Float64, queue_size=10)
		self.pub_yd = rospy.Publisher('posy_d', Float64, queue_size=10)
		self.pub_zd = rospy.Publisher('posz_d', Float64, queue_size=10)
		self.pub_xr = rospy.Publisher('posx_r', Float64, queue_size=10)
		self.pub_yr = rospy.Publisher('posy_r', Float64, queue_size=10)
		self.pub_zr = rospy.Publisher('posz_r', Float64, queue_size=10)

		rospy.init_node('ambf_client')


	def publish_to_plot(self, er_a, er_rel, xd, yd, zd, xr, yr, zr):

		self.pub_f.publish(self.force)
		self.pub_t.publish(self.deltat_a)
		self.pub_fd.publish(self.force_const)
		self.pub_ea.publish(er_a)
		self.pub_er.publish(er_rel)
		self.pub_xd.publish(xd)
		self.pub_yd.publish(yd)
		self.pub_zd.publish(zd)
		self.pub_xr.publish(xr)
		self.pub_yr.publish(yr)
		self.pub_zr.publish(zr)
		
	
	def inverse_kinematics(self, X_des, Y_des, Z_des):

		r = math.sqrt(math.pow(X_des,2)+math.pow(Y_des,2)+math.pow(Z_des,2))

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

	
	



	
	#Get in contact with a body and reach a certain force
	
	def approach_goal_Z(self, m_start):
		f = 50
		self.m = m_start
		force_old2 = 0
		force_old1 = 0
		count = 0
		window = []
		window_size = 10
		sum = 0
		count1 = 0
		while self.m < self.limit_mi:
			
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

			print(self.force)

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

			self.publish_to_plot(0,0,0,0,-0.22,0,0,-0.22)

			self.graph_f = np.append(self.graph_f, self.force)
			self.graph_fd = np.append(self.graph_fd, self.force_const)
			self.error_force = np.append(self.error_force, 0)
			#self.count_time_ef()

			
			self.count_time()		
			self.graph_px = np.append(self.graph_px, 0)
			self.graph_py = np.append(self.graph_py, 0)
			#self.graph_pz = np.append(self.graph_pz, -0.23)
			self.graph_pz = np.append(self.graph_pz,-0.22)
			PID = 1

			self.graph_frn = np.append(self.graph_frn, force_raw_now)



			self.px = np.append(self.px, 0)
			self.py = np.append(self.py, 0)
			#self.pz = np.append(self.pz, -0.23)
			self.pz = np.append(self.pz, -0.22)
			self.q1_r = np.append(self.q1_r, 0)
			self.q2_r = np.append(self.q2_r, 0)
			self.q3_r = np.append(self.q3_r, 0)
			self.graph_pxd = np.append(self.graph_pxd, 0)
			self.graph_pyd = np.append(self.graph_pyd, 0)
			self.graph_pzd = np.append(self.graph_pzd, -0.22)



			#lines below to plot even x and y components of the force in the world reference frame. If these components are required, 
			# modify the function .get_force() in ambf/ambf_ros_modules/ambf_comm/scripts/ambf_object.py
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

		return time_vect, posx_vect, posy_vect
		



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

			
			print(zfk[j])

			self.publish_to_plot(er_a[j], e_rel, x_v[j], y_v[j], z_v[j], xfk[j], yfk[j], pz_v[j])
			
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
			self.er = np.append(self.er, er_a[i])


	def plot_force(self):

		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_cart_time.csv', time2, delimiter=",")
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_cart_force.csv', self.graph_f2, delimiter=",") 
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_cl_cart_error.csv', self.er, delimiter=",")

		time = []
		time = self.time
		time_ef = []
		time2 = self.time_ef

		#fdim = 12
		font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 30}

		matplotlib.rc('font', **font)
		matplotlib.rc('legend',fontsize=24)
			
		fig, axs = plt.subplots(nrows = 3, sharex=True)
		fig.subplots_adjust(hspace=0.25)
		
		custom_xlim = (0,26)
		plt.setp(axs, xlim=custom_xlim)

		axs[0].plot(time, self.graph_f, color = 'r', label = "actual force")
		axs[0].plot(time, self.graph_fd, color = 'b', label = "target force")
		axs[0].set(ylabel = 'Force [N]')
		axs[0].set_title('CLOTH', fontsize=24)
		axs[0].legend(loc='upper left')
		axs[0].grid()

		axs[1].plot(time, self.error_abs, color = 'r', label = "abs_error")
		axs[1].set(ylabel = 'Abs_F_er [N]')
		axs[1].legend(loc='upper left')
		axs[1].grid()

		#axs[3].plot(time, self.graph_pz, color = 'r', label = "posz" )
		axs[2].plot(time, self.graph_pzd, color = 'b', label = "Z_des")
		axs[2].plot(time, self.graph_pz, color = 'g', label = "Z_read")
		axs[2].set(ylabel = 'posZ [m]')
		axs[2].legend(loc='upper left')
		axs[2].set(xlabel = 'Time [s]')	
		axs[2].grid()
		
	
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
		axs[0].set_title('CLOTH', fontsize=24)
		axs[0].set(ylabel = 'Pos_x [m]')	
		axs[0].legend(loc='upper left')
		axs[0].grid()
		
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

	#Initial calibration to get the transformation matrix to move from the simulation reference frame to the PSM base reference frame.
	#This is then applied to the body substituted to the tools, whose position is read through a function given by AMBF in 
	#the simulation reference frame

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

	cart_c.init_ROS()

	psm_handle_pel.set_joint_pos(0, 0)
	psm_handle_pfl.set_joint_pos(0, 0)
	psm_handle_base.set_joint_pos(0, math.radians(0))
	time.sleep(2)
	psm_handle_pel.set_joint_pos(0, 0)
	time.sleep(1)

	psm_handle_pel.set_joint_pos(0, 0)
	m_start = 0.165#0.17 inclined
	psm_handle_pel.set_joint_pos(0, m_start)
	time.sleep(2)
	
	
	cart_c.approach_goal_Z(m_start)
	cart_c.calibration()
	

	cart_c.reach_XY_force_control(0.01, 0.1)
	cart_c.reach_XY_force_control(0.1,-0.02)
	
	cart_c.plot_force()
	time.sleep(1)
	#cart_c.plot_positions()

	
	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()
