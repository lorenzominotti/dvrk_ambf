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

	graph_f2 = []
	graph_fd2 = []
	error_force2 = []
	window = []
	xd_plot = []
	yd_plot = []
	zd_plot = []
	xr_plot = []
	yr_plot = []
	zr_plot = []
	time_plot = []
	abs_err = []

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
	delta_m_start = 0.00008
	band = 0.01
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

	Kp = 0.08 #rqt_plot
	Ki = 0.008

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

		rospy.init_node('ambf_client')



	def publish_to_plot(self):

		self.pub_f.publish(self.force)
		#self.pub_t.publish(self.deltat_a)
		self.pub_fd.publish(self.force_const)
	
	
	def inverse_kinematics(self, X_des, Y_des, Z_des):

		r = math.sqrt(math.pow(X_des,2)+math.pow(Y_des,2)+math.pow(Z_des,2))
		
		#self.q1 = 0.5*(math.acos((math.pow(Z_des,2)-math.pow(X_des,2))/(math.pow(X_des,2)+math.pow(Z_des,2))))   #original

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


	def count_time(self):
		time_end_a = time.time()
		self.deltat_a = (time_end_a-self.time_start_a) + self.deltat_a 
		self.time = np.append(self.time, self.deltat_a)

	def count_time_ef(self):
		time_end_a_ef = time.time()
		self.deltat_a_ef = (time_end_a_ef-self.time_start_a) + self.deltat_a_ef 
		self.time_ef = np.append(self.time_ef, self.deltat_a_ef)


	def approach_goal_Z(self, m_start):
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
			#_,_,force_raw_now = psm_handle_mi.get_force()
			force_raw_now = psm_handle_mi.get_force()
			self.force = force_raw_now
			count = count + 1
			if count < window_size + 1:
				self.window = np.append(self.window, force_raw_now)
				self.force = force_raw_now
			else:
				for i in range(1, window_size):
					self.window[i-1] = self.window[i]
					if i == (window_size - 1):
						self.window[i] = force_raw_now
					sum = sum + self.window[i-1]
				self.force = sum / window_size
				sum = 0

			print(self.force)
			if self.force > (self.force_const + self.band):
				self.m = self.m - self.delta_m_start/2
				psm_handle_pel.set_joint_pos(0, self.m)
			if self.force < (self.force_const - self.band):
				self.m = self.m + self.delta_m_start/2
				psm_handle_pel.set_joint_pos(0, self.m)

			if (self.force >= self.force_const):
				break
			
			psm_handle_pel.set_joint_pos(0, self.m)
			force_old2 = force_old1
			force_old1 = self.force

			self.error_force = np.append(self.error_force, 0)
					
			self.graph_px = np.append(self.graph_px, 0)
			self.graph_py = np.append(self.graph_py, 0)
			PID = 1

			self.graph_frn = np.append(self.graph_frn, force_raw_now)

			ex = 0
			self.er_x = np.append(self.er_x, ex)
			ey = 0
			self.er_y = np.append(self.er_y, ey)
			ez = 0
			self.er_z = np.append(self.er_z, ez)


	def plot_sin(self):

		print("plot...")
		time = []
		time = self.time
		time_ef = []
		time_ef = self.time_ef

		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_cl_sin_time.csv', time, delimiter=",")
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_cl_sin_force.csv', self.graph_f, delimiter=",") 
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_cl_sin_forced.csv', self.graph_fd, delimiter=",") 
		#np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_cl_sin_error.csv', self.abs_err, delimiter=",")

		fig, axs = plt.subplots(nrows = 2)
		axs[0].plot(time, self.graph_f, color = 'r', label = "actual force")
		axs[0].plot(time, self.graph_fd, color = 'b', label = "target force")
		#axs[0].set(xlabel = 'Time [s]', ylabel = 'Force [N]')	
		axs[0].set(ylabel = 'Force [N]')	
		axs[0].legend(loc='best')
		axs[0].grid()
	
		axs[1].plot(time, self.abs_err, color = 'r', label = "error")
		axs[1].set(xlabel = 'Time [s]', ylabel = 'Force_error_norm')
		axs[1].legend(loc='best')
		axs[1].grid()

		plt.show()


	def exert_sin_force2(self, m_start):
		
		force_base = self.force_const
		count = 0
		window = []
		window_size = 10
		sum = 0
		count1 = 0
		
		step = 3.46
		times = 0
		angle = -3.46

		self.f_cycle = 60
		self.exp_time = 1.734*20
		dim = self.f_cycle*self.exp_time
		dim = int(dim)

		t_cycle = 1/self.f_cycle
		print(t_cycle)

		self.graph_f_cycle = np.zeros(dim)
		self.graph_fd_cycle = np.zeros(dim)
		self.error_force_cycle = np.zeros(dim)
		
		xfk = np.zeros(dim)
		yfk = np.zeros(dim)
		e_a = np.zeros(dim)

		force_target = np.zeros(dim)


		for i in range(0,dim):

			if angle >= 360:
				angle = 0
			angle = angle + step
			force_target[i] = force_base + self.amplitude*np.sin(math.radians(angle))

			
		
		set_angle = 20
		print("SET INCLINATION")
		psm_handle_pfl.set_joint_pos(0,math.radians(-set_angle))
		time.sleep(0.5)
		self.approach_goal_Z(m_start)
		q1_r,q2_r,q3_r = self.get_position_joints_PSM()
		x_fk,y_fk,z_fk = self.forward_kinematics(q1_r,q2_r,q3_r)

		Kps = 0.007 #good for step = 5
		Kis = 0.0006
		
		j=0

		while(j<self.f_cycle*self.exp_time):
			
			self.count_time()
			starttime=time.time()
			self.time_start_a = time.time()

			q1_r,q2_r,q3_r = self.get_position_joints_PSM()
			xfk[j],yfk[j],zfk = self.forward_kinematics(q1_r,q2_r,q3_r)
			

			force_raw_now = psm_handle_mi.get_force()

			
			for i in range(1, window_size):
				self.window[i-1] = self.window[i]
				if i == (window_size - 1):
					self.window[i] = force_raw_now
				sum = sum + self.window[i-1]
			self.force = sum / window_size
			sum = 0

			error = force_target[j] - self.force
			e_rel = error/force_target[j]
			e_a[j] = abs(error)
			self.P_value = (Kps * error)
		
			self.Integrator = self.Integrator + error
			self.I_value = self.Integrator * Kis
				
			PID = self.P_value + self.I_value
			zd = zfk + PID*zfk
			
			q1,q2,q3 = self.inverse_kinematics(xfk[0],yfk[0],zd)
		
			
			self.set_position_robot(q1,q2,q3)


			#print(force_target[j],self.force)
			self.graph_f_cycle[j] = self.force
			self.graph_fd_cycle[j] = force_target[j]
			self.error_force_cycle[j] = e_rel


			self.force_const = force_target[j]
			self.publish_to_plot()
			
			j=j+1
			if j>dim-1:
				break

			wait = 1/self.f_cycle - (time.time() - starttime) 
			if wait>0:
				time.sleep(wait)

			print(time.time() - starttime,   j)
			#print(time.time()-starttime)


		for i in range (0,dim):

			self.graph_f = np.append(self.graph_f, self.graph_f_cycle[i])
			self.graph_fd = np.append(self.graph_fd, self.graph_fd_cycle[i])
			self.error_force = np.append(self.error_force, self.error_force_cycle[i])
			self.abs_err = np.append(self.abs_err, e_a[i])



	
	
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
	m_start = 0.16
	psm_handle_pel.set_joint_pos(0, m_start)
	time.sleep(2)
	

	cart_c = Cartesian_control()
	cart_c.init_ROS()
	
	cart_c.exert_sin_force2(m_start)
	cart_c.plot_sin()

	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()


