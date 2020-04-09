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





# class for cartesian control

class Cartesian_control:
	
	xd_plot = []
	yd_plot = []
	zd_plot = []
	xr_plot = []
	yr_plot = []
	zr_plot = []
	time_plot = []

	graph_f2 = []
	error_force2 = []

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
	delta_m_start = 0.0003 #0.0001 originally
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
	l_tool = 0.05 + l_tool_original

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
	Kp = 0.0002 #good2  <--------
	Ki = 0.000008

	Kp = 0.0004 #bad2
	Ki = 0.0000008
	'''
	Kp = 0.000002 #bad3 <--------
	Ki = 0.0000008
	
	'''
	Kp = 0.000001
	Ki = 0.00000008
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


	def init_ROS(self):

		self.pub_t = rospy.Publisher('time', Float64, queue_size=10)
		self.pub_f = rospy.Publisher('force_read', Float64, queue_size=10)
		self.pub_fd = rospy.Publisher('force_desired', Float64, queue_size=10)
		self.pub_ea = rospy.Publisher('force_ea', Float64, queue_size=10)
		self.pub_er = rospy.Publisher('force_error', Float64, queue_size=10)
		self.pub_ex = rospy.Publisher('posx_er', Float64, queue_size=10)
		self.pub_ey = rospy.Publisher('posy_er', Float64, queue_size=10)
		self.pub_ez = rospy.Publisher('posz_er', Float64, queue_size=10)
		self.pub_x = rospy.Publisher('posX', Float64, queue_size=10)
		self.pub_y = rospy.Publisher('posY', Float64, queue_size=10)
		self.pub_z = rospy.Publisher('posZ', Float64, queue_size=10)
		self.pub_zr = rospy.Publisher('posZr', Float64, queue_size=10)



		rospy.init_node('ambf_client')



	def publish_to_plot(self, er_a, er_rel, xd, yd, zd, xr, yr, zr, pz):

		self.pub_f.publish(self.force)
		self.pub_t.publish(self.deltat_a)
		self.pub_fd.publish(self.force_const)
		self.pub_ea.publish(er_a)
		self.pub_er.publish(er_rel)
		self.pub_ex.publish(xd-xr)
		self.pub_ey.publish(yd-yr)
		self.pub_ez.publish(zd-zr)
		self.pub_x.publish(xd)
		self.pub_y.publish(yd)
		self.pub_z.publish(zd)
		self.pub_zr.publish(pz)
		

	
	def inverse_kinematics(self, X_des, Y_des, Z_des):

		r = math.sqrt(math.pow(X_des,2)+math.pow(Y_des,2)+math.pow(Z_des,2))

		q1 = math.asin((X_des)/(math.sqrt(math.pow(X_des,2)+math.pow(Z_des,2))))
		q2 = math.asin(-Y_des/r)
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

		return self.jac
		



	def count_time(self):
		time_end_a = time.time()
		self.deltat_a = (time_end_a-self.time_start_a) + self.deltat_a 
		#self.time = np.append(self.time, self.deltat_a)

	def count_time_ef(self):
		time_end_a_ef = time.time()
		self.deltat_a_ef = (time_end_a_ef-self.time_start_a) + self.deltat_a_ef 
		self.time_ef = np.append(self.time_ef, self.deltat_a_ef)




	def plots(self):
		
		time = []
		time = self.time
		time_ef = []
		time2 = self.time_ef


		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_cart_time.csv', time2, delimiter=",")
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_cart_force.csv', self.graph_f2, delimiter=",") 
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_cart_error.csv', self.error_force2, delimiter=",") 

		fdim = 12
		fig, axs = plt.subplots(nrows = 6)

		axs[0].plot(time, self.graph_f, color = 'r', label = "actual force")
		axs[0].plot(time, self.graph_fd, color = 'b', label = "target force")
		axs[0].set(ylabel = 'Force [N]')
		#axs[0].set_xticklabels([],rotation=0, fontsize=1)
		#axs[0].tick_params(labelsize=fdim)	
		axs[0].legend(loc='best')
		axs[0].grid()

		axs[1].plot(time, self.error_force, color = 'r', label = "error")
		axs[1].set(ylabel = 'Force_error_norm')
		#axs[1].set_xticklabels([],rotation=0, fontsize=1)
		#axs[1].tick_params(labelsize=fdim)	
		axs[1].legend(loc='best')
		axs[1].grid()

		axs[2].plot(time, self.er_x, label = "err_posx")
		axs[2].set(ylabel = 'posx_error [m]')
		#axs[2].set_xticklabels([],rotation=0, fontsize=1)
		#axs[2].tick_params(labelsize=fdim)	
		axs[2].legend(loc='best')
		axs[2].grid()

		axs[3].plot(time, self.er_y, label = "err_posy")
		axs[3].set(ylabel = 'posy_error [m]')
		#axs[3].set_xticklabels([],rotation=0, fontsize=1)
		#axs[3].tick_params(labelsize=fdim)	
		axs[3].legend(loc='best')
		axs[3].grid()

		axs[4].plot(time, self.er_z, label = "err_posz")
		axs[4].set(ylabel = 'posz_error [m]')
		axs[4].set(xlabel = 'Time [s]')	
		#axs[4].set_xticklabels([],rotation=0, fontsize=1)	
		#axs[4].tick_params(labelsize=fdim)
		axs[4].legend(loc='best')
		axs[4].grid()

		axs[5].plot(time, self.graph_px, label = "posx")
		axs[5].plot(time, self.graph_py, label = "posy")
		axs[5].set(ylabel = 'position xy [m]')
		axs[5].set(xlabel = 'Time [s]')	
		#axs[5].set_xticklabels([],rotation=0, fontsize=1)
		#axs[5].tick_params(labelsize=fdim)	
		axs[5].legend(loc='best')
		axs[5].grid()
		
		plt.show()


	
	#this function to approach an object. The position of main insertion link only is increased in the joint space, while
	#keeping fixed the position of the other joints (previously set). The position is increased at every iteration of a certain amount
	#if the force read is lower than the target force, it is decreased of the same amount if the force read is higher than the target
	#force. Once the target force is reached, if the force read remains within a bandwidth for a specified number of iteration, exit the loop.

	
	def approach_goal_Z(self, m_start):
		#f = 70
		f = self.f_cycle
		self.m = m_start
		force_old2 = 0
		force_old1 = 0
		count = 0
		window = []
		window_size = 10
		sum = 0
		count1 = 0

		#r = rospy.Rate(100)

		while self.m < self.limit_mi:
			
			self.time_start_a = time.time()

			########################################################
			pos = psm_handle_trl.get_pos()
			pz = pos.z + self.deltaZ
			########################################################

			#_,_,force_raw_now = psm_handle_mi.get_force()
			force_raw_now = psm_handle_mi.get_force()
			self.force = force_raw_now
			#print(self.force)
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

		
			#'''
			psm_handle_pel.set_joint_pos(0, self.m)
			force_old2 = force_old1
			force_old1 = self.force

			self.graph_f = np.append(self.graph_f, self.force)
			self.graph_fd = np.append(self.graph_fd, self.force_const)
			self.error_force = np.append(self.error_force, 0)
			#self.count_time_ef()

			
			#self.count_time()		
			self.graph_px = np.append(self.graph_px, 0)
			self.graph_py = np.append(self.graph_py, 0)
			PID = 1

			self.graph_frn = np.append(self.graph_frn, force_raw_now)

			#lines below to plot even x and y components of the force in the world reference frame
			
			
			ex = 0
			self.er_x = np.append(self.er_x, ex)
			ey = 0
			self.er_y = np.append(self.er_y, ey)
			ez = 0
			self.er_z = np.append(self.er_z, ez)
			#'''
			self.count_time()


			self.publish_to_plot(0, 0, 0, 0, 0, 0, 0, 0, 0)
			

			wait = 1/f-(time.time()-self.time_start_a)
			#print(self.force, wait)    

			if (wait)>0:
				time.sleep(wait)
			print(self.force, (time.time()-self.time_start_a)) 
			
			
   			


	 
	#compute vector of time and of x y cartesian position, from the start position to the goal
	
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
		
	
	'''

	def compute_inverse_kinematics(self, time_vect, x_vect, y_vect, z_vect):
		
		print("Processing:  IK computation ....")
		q1_vect = []
		q2_vect = []
		q3_vect = []

		for i in range(0, time_vect.size):
			q1, q2, q3 = self.inverse_kinematics(x_vect[i], y_vect[i], z_vect[i])
			q1_vect = np.append(q1_vect, q1)
			q2_vect = np.append(q2_vect, q2)
			q3_vect = np.append(q3_vect, q3)

		return q1_vect, q2_vect, q3_vect


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

	#Cartesian control old

	def reach_XY_force_control(self, goal_x, goal_y):

		self.f_cycle = 50
		self.exp_time = 10 
		dim = self.f_cycle*self.exp_time

		time_v, x_v, y_v = self.define_path(goal_x, goal_y)
		q1_r,q2_r,q3_r = self.get_position_joints_PSM()
		time.sleep(1)	
		

		print("Moving arm ....")
		j=0
		time_now = 0
		new = True
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
		self.graph_f_cycle = np.zeros(dim)
		self.graph_fd_cycle = np.zeros(dim)
		self.error_force_cycle = np.zeros(dim)

		print(zfk[0])

		#start_timer = time.time()
		int_er_force = 0

		while(j<self.f_cycle*self.exp_time):

			self.count_time()
			self.count_time_ef()
			starttime=time.time()
			self.time_start_a = time.time()

			q1_r,q2_r,q3_r = self.get_position_joints_PSM()
			xfk[j],yfk[j],zfk[j] = self.forward_kinematics(q1_r,q2_r,q3_r)

			force_raw_now = psm_handle_mi.get_force()

			#filter force read with moving average

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

			#PI control on force

			error = self.force_const - self.force
			e_abs = error
			e_rel = error/self.force_const
			self.P_value = (self.Kp * error)
		
			self.Integrator = self.Integrator + error
			self.I_value = self.Integrator * self.Ki
				
			PID = self.P_value + self.I_value
			zd = zfk[j] + PID*zfk[j]

			z_v[j] = zd

			#inverse kinematics to get joint positions. Joint positions used to set the new robot position in simulation

			q1,q2,q3 = self.inverse_kinematics(x_v[j],y_v[j],z_v[j])
			self.set_position_robot(q1,q2,q3)
			#'''
			self.graph_f_cycle[j] = self.force
			self.graph_fd_cycle[j] = self.force_const
			self.error_force_cycle[j] = e_rel
			#'''

			self.publish_to_plot(e_abs, e_rel, x_v[j], y_v[j], z_v[j], xfk[j], yfk[j], zfk[j])
			
			j=j+1

			#time.sleep(1/self.f_cycle)
			#print(time.time()-starttime)
			wait = 1/self.f_cycle - ((time.time() - starttime))
			if wait > 0:
				time.sleep(wait)
			print(time.time()-starttime)
			#time.sleep(wait)

	
		self.graph_px = np.append(self.graph_px, x_v)
		self.graph_py = np.append(self.graph_py, y_v)


		for i in range (0,dim):

			self.er_x = np.append(self.er_x, x_v[i]-xfk[i])
			self.er_y = np.append(self.er_y, y_v[i]-yfk[i])
			self.er_z = np.append(self.er_z, z_v[i]-zfk[i])
			self.graph_f = np.append(self.graph_f, self.graph_f_cycle[i])
			self.graph_fd = np.append(self.graph_fd, self.graph_fd_cycle[i])
			self.error_force = np.append(self.error_force, self.error_force_cycle[i])	
			self.graph_f2 = np.append(self.graph_f2, self.graph_f_cycle[i])
			self.error_force2 = np.append(self.error_force2, self.error_force_cycle[i])
		


		
		


	def plot_new(self):
		
		time = []
		time = self.time
		time_ef = []
		time_ef = self.time_ef
	
		fig, axs = plt.subplots(nrows = 6)

		axs[0].plot(time, self.xr_plot, color = 'r', label = "actual x")
		axs[0].plot(time, self.xd_plot, color = 'b', label = "target x")
		axs[0].set(ylabel = 'Pos_x [m]')	
		axs[0].legend(loc='best')
		axs[0].grid()

		axs[1].plot(time, self.yr_plot, color = 'r', label = "actual y")
		axs[1].plot(time, self.yd_plot, color = 'b', label = "target y")
		axs[1].set(ylabel = 'Pos_y [m]')	
		axs[1].legend(loc='best')
		axs[1].grid()

		axs[2].plot(time, self.zr_plot, color = 'r', label = "actual z")
		axs[2].plot(time, self.zd_plot, color = 'b', label = "target z")
		axs[2].set(ylabel = 'Pos_z [m]')	
		axs[2].legend(loc='best')
		axs[2].grid()

		axs[3].plot(time, self.er_x, label = "err_posx")
		axs[3].set(ylabel = 'posx_error [m]')
		axs[3].legend(loc='best')
		axs[3].grid()

		axs[4].plot(time, self.er_y, label = "err_posy")
		axs[4].set(ylabel = 'posy_error [m]')
		axs[4].legend(loc='best')
		axs[4].grid()

		axs[5].plot(time, self.er_z, label = "err_posz")
		axs[5].set(ylabel = 'posz_error [m]')
		axs[5].set(xlabel = 'Time [s]')	
		axs[5].legend(loc='best')
		axs[5].grid()

		plt.show()



	#define paths from one point to another

	def def_paths(self,points):
		
		self.f_cycle = 40
		self.exp_time = 10 
		dim = self.f_cycle*self.exp_time
		n_points = len(points)

		time_vect = []
		posx_mat = np.zeros((n_points, self.f_cycle*self.exp_time))
		posy_mat = np.zeros((n_points, self.f_cycle*self.exp_time))

		
		
		
		print(n_points)
		j = 0
		while j<n_points:

			print("Processing:  Defining cartesian paths ....")

			if j == 0:

				time.sleep(0.5)
				q1_read, q2_read, q3_read = self.get_position_joints_PSM()
				x_el, y_el, z_el = self.forward_kinematics(q1_read, q2_read, q3_read)
				point_goal = points[j]
				goal_x = point_goal[0]
				goal_y = point_goal[1]

			else:

				point = points[j-1]
				x_el = point[0]
								
				y_el = point[1]

				point_goal = points[j]
				goal_x = point_goal[0]
				goal_y = point_goal[1]		
		
			dx = (goal_x - x_el)/dim
			for i in range(0, self.f_cycle*self.exp_time):
				posx_mat[j][i] = x_el
				x_el = x_el + dx

			dy = (goal_y - y_el)/dim
			for i in range(0, self.f_cycle*self.exp_time):
				posy_mat[j][i] = y_el
				y_el = y_el + dy		

			j = j+1			

		return posx_mat, posy_mat
	
	
	#define movement from a point to another

	def reach_XY_force_control_continuous(self,x_v,y_v):

		dim = self.f_cycle*self.exp_time

		q1_r,q2_r,q3_r = self.get_position_joints_PSM()		

		print("Moving arm ....")
		j=0
		time_now = 0
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
		self.graph_f_cycle = np.zeros(dim)
		self.graph_fd_cycle = np.zeros(dim)
		self.error_force_cycle = np.zeros(dim)


		while(j<self.f_cycle*self.exp_time):

			self.count_time()
			self.count_time_ef()
			starttime=time.time()
			self.time_start_a = time.time()

			########################################################
			pos = psm_handle_trl.get_pos()
			pz = pos.z + self.deltaZ
			########################################################

			q1_r,q2_r,q3_r = self.get_position_joints_PSM()
			xfk[j],yfk[j],zfk[j] = self.forward_kinematics(q1_r,q2_r,q3_r)

			force_raw_now = psm_handle_mi.get_force()

			#filter force read with moving average

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

			#PI control on force

			error = self.force_const - self.force
			e_abs = error
			e_rel = error/self.force_const
			self.P_value = (self.Kp * error)
		
			self.Integrator = self.Integrator + error
			self.I_value = self.Integrator * self.Ki
				
			PID = self.P_value + self.I_value
			zd = zfk[j] + PID*zfk[j]

			z_v[j] = zd

			#inverse kinematics to get joint positions. Joint positions used to set the new robot position in simulation

			q1,q2,q3 = self.inverse_kinematics(x_v[j],y_v[j],z_v[j])
			self.set_position_robot(q1,q2,q3)

			self.graph_f_cycle[j] = self.force
			self.graph_fd_cycle[j] = self.force_const
			self.error_force_cycle[j] = e_rel

			self.publish_to_plot(e_abs, e_rel, x_v[j], y_v[j], z_v[j], xfk[j], yfk[j], zfk[j], pz)
			
			j=j+1

			wait = 1/self.f_cycle - (time.time() - self.time_start_a) 
			if wait>0:
				time.sleep(wait)
			#print(time.time()-self.time_start_a)
			print(self.force)
	
		'''
		self.graph_px = np.append(self.graph_px, x_v)
		self.graph_py = np.append(self.graph_py, y_v)


		for i in range (0,dim):

			self.er_x = np.append(self.er_x, x_v[i]-xfk[i])
			self.er_y = np.append(self.er_y, y_v[i]-yfk[i])
			self.er_z = np.append(self.er_z, z_v[i]-zfk[i])
			self.graph_f = np.append(self.graph_f, self.graph_f_cycle[i])
			self.graph_fd = np.append(self.graph_fd, self.graph_fd_cycle[i])
			self.error_force = np.append(self.error_force, self.error_force_cycle[i])	
			self.graph_f2 = np.append(self.graph_f2, self.graph_f_cycle[i])
			self.error_force2 = np.append(self.error_force2, self.error_force_cycle[i])
		'''

	def define_parabolic_path(self, goal_x):

		print("Processing:  Defining cartesian parabolic path ....")
		
		time_vect = []
		posx_vect = []
		posy_vect = []
		posz_vect = []

		self.f_cycle = 50
		self.exp_time = 15 
		dim = self.f_cycle*self.exp_time

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
			y_el = (-32.28)*math.pow(x_el,2) - (61/285)*x_el + 0.11
			posy_vect = np.append(posy_vect, y_el)
			x_el = x_el + dx

		return time_vect, posx_vect, posy_vect


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

	#set initial position

	psm_handle_pel.set_joint_pos(0, 0)
	psm_handle_pfl.set_joint_pos(0, 0)
	psm_handle_base.set_joint_pos(0, math.radians(0))
	time.sleep(2)
	psm_handle_pel.set_joint_pos(0, 0)
	time.sleep(1)
	psm_handle_pel.set_joint_pos(0, 0)
	m_start = 0.155
	psm_handle_pel.set_joint_pos(0, m_start)
	time.sleep(2)
	
	#define points

	cart_c = Cartesian_control()

	###############tip position in psm ref frame#################
	time.sleep(1)
	pos = psm_handle_trl.get_pos()
	pz = pos.z
	q1, q2, q3 = cart_c.get_position_joints_PSM()
	xfk, yfk, zfk, = cart_c.forward_kinematics(q1,q2,q3)

	print("pos Z in RFsim:     ", pz)
	print("pos Z in RFpsm:     ", zfk)
	print("\n\n")

	cart_c.deltaZ = abs(pz-zfk)
	
	print("deltaZ:				", cart_c.deltaZ)

	if pz>zfk:
		cart_c.deltaZ = -cart_c.deltaZ
	if pz<zfk:
		cart_c.deltaZ = cart_c.deltaZ

	#############################################################
	
	cart_c.init_ROS()



	################ cartesian continuous ###############
	
	'''
	point1 = [0.09, 0.05]
	point2 = [0.05, -0.02]
	point3 = [0.01, 0.10]
	'''
	point1 = [0.05, 0.02]
	point2 = [-0.02, 0.04]
	point3 = [0.09, -0.02]
	
	points = [point1,point2,point3]

	#define paths

	posx_mat, posy_mat = cart_c.def_paths(points)

	posx_vect = {}
	for i in range(0,len(points)):
		posx_vect[i]=[]
		for j in range(0,cart_c.f_cycle*cart_c.exp_time):
			posx_vect[i].append(posx_mat[i][j])

	posy_vect = {}
	for i in range(0,len(points)):
		posy_vect[i]=[]
		for j in range(0,cart_c.f_cycle*cart_c.exp_time):
			posy_vect[i].append(posy_mat[i][j])

	#approach to the body

	cart_c.approach_goal_Z(m_start)

	#execute movement through paths previously defined

	for i in range(0,len(points)):
		cart_c.reach_XY_force_control_continuous(posx_vect[i],posy_vect[i])

	stop = 100
	cart_c.pub_f.publish(stop)
	
	
	############## cartesian NOT continuous #############
	'''
	cart_c.approach_goal_Z(m_start)
	cart_c.reach_XY_force_control(0.09, 0.07)

	stop = 100
	cart_c.pub_f.publish(stop)


	'''
	####################  parabolic  ####################
	'''
	cart_c.reach_XY_force_control(0.045, 0.035)
	_,x_v,y_v = cart_c.define_parabolic_path(-0.05)
	cart_c.reach_XY_force_control_continuous(x_v,y_v)
	'''
	#####################################################

	#cart_c.reach_XY_force_control(0.05, -0.01)
	#cart_c.reach_XY_force_control(0.01, 0.10)
	#cart_c.reach_pos_XY(-0.04, 0.06, False)
	#cart_c.reach_pos_XY(-0.04, 0.01, False)

	
	
	print('STEP1')	
	#cart_c.plot_new()

	#cart_c.plots()


	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
	main()
  		
    	

