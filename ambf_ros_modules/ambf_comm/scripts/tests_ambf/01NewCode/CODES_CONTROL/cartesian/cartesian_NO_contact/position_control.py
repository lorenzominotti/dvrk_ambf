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

	
	
	def define_path(self, goal_x, goal_y, goal_z,start):
		print("Processing:  Defining cartesian path ....")
		
		time_vect = []
		posx_vect = []
		posy_vect = []
		posz_vect = []

		time_el = 0
		for i in range(0, self.f_cycle*self.exp_time):
			time_vect = np.append(time_vect, time_el)
			time_el = time_el + 1/self.f_cycle

		q1_read, q2_read, q3_read = self.get_position_joints_PSM()
		
		x_el, y_el, z_el = self.forward_kinematics(q1_read, q2_read, q3_read)
		print("ZEL:       ", z_el)
		print("z_GOAL:       ", goal_z)
	
		
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

		return time_vect, posx_vect, posy_vect, posz_vect
	


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



	def reach_XY_pos_control(self, goal_x, goal_y, goal_z, start):

		self.f_cycle = 40
		self.exp_time = 10
		dim = self.f_cycle*self.exp_time

		time_v, x_v, y_v, z_v = self.define_path(goal_x, goal_y, goal_z, start)
		q1_v, q2_v, q3_v = self.compute_inverse_kinematics(time_v, x_v, y_v, z_v)
		self.set_position_robot(q1_v[0], q2_v[0], q3_v[0])
		time.sleep(2)
	
		print("Moving arm ....")
		j=0
		time_now = 0
		#starttime=time.time()
		self.time_start_a = time.time()
		self.xr_plot = np.zeros(dim)
		self.yr_plot = np.zeros(dim)
		self.zr_plot = np.zeros(dim)

		while(j<self.f_cycle*self.exp_time):
			self.count_time()
			starttime=time.time()
			self.time_start_a = time.time()
			self.set_position_robot(q1_v[j], q2_v[j], q3_v[j])
			

			q1_r,q2_r,q3_r = self.get_position_joints_PSM()
			xfk,yfk,zfk = self.forward_kinematics(q1_r,q2_r,q3_r)
			self.xr_plot[j] = xfk
			self.yr_plot[j] = yfk
			self.zr_plot[j] = zfk
			
			j=j+1
			time.sleep(1/self.f_cycle)
			
			print(time.time() - self.time_start_a, 1/self.f_cycle - (time.time() - starttime))
		
		for i in range(0, time_v.size):
	
			self.xd_plot = np.append(self.xd_plot, x_v[i])
			self.yd_plot = np.append(self.yd_plot, y_v[i])
			self.zd_plot = np.append(self.zd_plot, z_v[i])
			self.er_x = np.append(self.er_x, self.xd_plot[i] - self.xr_plot[i])	
			self.er_y = np.append(self.er_y, self.yd_plot[i] - self.yr_plot[i])
			self.er_z = np.append(self.er_z, self.zd_plot[i] - self.zr_plot[i])

		


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
	m_start =  0.16095557808876038
	psm_handle_pel.set_joint_pos(0, m_start)
	time.sleep(2)
	

	cart_c = Cartesian_control()
	
	print("go in 2 seconds!!!")
	
	time.sleep(3)
	q1,q2,q3 = cart_c.get_position_joints_PSM()
	_,_,zFK = cart_c.forward_kinematics(q1,q2,q3)
	print("FK START:   ", zFK)
	print("Q3 START:   ", q3)

	cart_c.reach_XY_pos_control(0.0, 0.1, -0.196, True)
	
	print('STEP1')	
	cart_c.plot_new()



	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()

