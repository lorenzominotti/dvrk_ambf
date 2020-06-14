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
	pz = []

	degree = 0
	delta = 0.6 
	delta_m = 0.00005
	delta_m_start = 0.00004
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
	T = np.zeros((4,4))

	posX = 0
	posY = 0
	posZ = 0
	pi = math.pi
	l_RCC = 0.4318
	l_tool_original = 0.4162
	deltalen = 0.05
	l_tool = deltalen + l_tool_original
	deltaZ = 0

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
	
	Kp = 0.0015 #standard
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


	# Initialize ROS communication
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


	# Publish on previously defined ROS topics
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
		
		

	# inverse kinematics computation	
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


	#forward kinematics computation
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

	
	# Get in contact with a body and reach a certain target force. At every iteration, just increase/decrease the value of joint q3
	# until a certain value of the target force is reached. Then, maintain this value for a definite number of iterations.
	
	def approach_goal_Z(self, m_start):
		#f = 70
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
			########################################################
			pos = psm_handle_trl.get_pos()
			pz = pos.z + self.deltaZ
			########################################################
			#_,_,force_raw_now = psm_handle_mi.get_force()
			force_raw_now = psm_handle_mi.get_force()
			self.force = force_raw_now
		
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

			self.count_time()

			self.publish_to_plot(0,0,0,0,-0.2045,0,0,-0.2045)			

			wait = 1/f-(time.time()-self.time_start_a)
			#print(self.force, wait)    

			if (wait)>0:
				time.sleep(wait)
			print(self.force, time.time()-self.time_start_a)


	# compute the trajectory of the end effector in the X and Y coordinates given its current position and a final goal (in X and Y)

	def def_paths(self,points):
		
		self.f_cycle = 50
		self.exp_time = 7 
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
	
	
	# Execute a pre-defined movement in X and Y coordinates while controlling the force in Z direction in order to keep it constant. In the main iteration 
	# loop: get the current joint positions and apply forward kinematics, get the contact force in Z direction (then filtered), and get the position of 
	# the end effector in the simulation reference frame (this is then trasformed in order to express it with respect to the PSM base reference frame). 
	# Apply PI control to get the new value of Z, compute the inverse kinematics and set the new joints positions to the robot.

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
		px_v = np.zeros(dim)
		py_v = np.zeros(dim)
		pz_v = np.zeros(dim)


		while(j<self.f_cycle*self.exp_time):

			self.count_time()
			#self.count_time_ef()
			starttime=time.time()
			self.time_start_a = time.time()

			#################################################################
			pos = psm_handle_trl.get_pos()  #if I want Z from AMBF function
			px = pos.x
			py = pos.y
			pz = pos.z

			px_v[j], py_v[j], pz_v[j] = self.from_AMBF2PSM_RF(px,py,pz)
			#################################################################

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
			#zd = zfk[j] + PID*zfk[j]
			zd = pz_v[j] + PID*pz_v[j]

			z_v[j] = zd

			#inverse kinematics to get joint positions. Joint positions used to set the new robot position in simulation

			q1,q2,q3 = self.inverse_kinematics(x_v[j],y_v[j],z_v[j])
			self.set_position_robot(q1,q2,q3)

			self.graph_f_cycle[j] = self.force
			self.graph_fd_cycle[j] = self.force_const
			self.error_force_cycle[j] = e_rel

			self.publish_to_plot(e_abs, e_rel, x_v[j], y_v[j], z_v[j], xfk[j], yfk[j], pz_v[j])
			
			j=j+1

			wait = 1/self.f_cycle - (time.time() - self.time_start_a) 
			if wait>0:
				time.sleep(wait)
			print(time.time()-self.time_start_a)
	
		

	#Initial calibration to get the transformation matrix to move from the simulation reference frame to the PSM base reference frame.
	#This is then applied to the body substituted to the tools, whose position is read through a function given by AMBF in 
	#the simulation reference frame


	def get_cart_pos_cal(self):

		q1_r,q2_r,q3_r = self.get_position_joints_PSM()
		xfk,yfk,zfk = self.forward_kinematics(q1_r,q2_r,q3_r)
		

		return xfk, yfk, zfk


	def calibration(self):

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
	m_start = 0.185
	psm_handle_pel.set_joint_pos(0, m_start)
	time.sleep(2)
	
	#define points

	cart_c = Cartesian_control()	
	
	cart_c.init_ROS()

	# Specify the points of interest through which the end effector should pass
	
	point1 = [0.09, 0.05]
	point2 = [0.05, -0.03]
	point3 = [-0.01, 0.08]
	point4 = [-0.08, 0.05]
	point5 = [0.02, -0.02]
	point6 = [0.09, 0.05]
	point7 = [0.05, -0.03]
	point8 = [-0.01, 0.08]
	point9 = [-0.08, 0.05]	
	
	points = [point1,point2,point3,point4,point5,point6,point7,point8,point9]

	# define all the linear trajectories between points in X and Y coordinates
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
	cart_c.calibration()

	#execute movement through paths previously defined
	for i in range(0,len(points)):
		cart_c.reach_XY_force_control_continuous(posx_vect[i],posy_vect[i])

	#Stop data trasmission
	stop = 100
	cart_c.pub_f.publish(stop)	

	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
	main()
  		
    	

