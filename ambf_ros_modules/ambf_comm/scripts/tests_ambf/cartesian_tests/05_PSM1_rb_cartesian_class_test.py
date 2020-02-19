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
	pi = math.pi
	l_RCC = 0.4318
	l_tool = 0.4162

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
	

	def inverse_kinematics(self, X_desired, Y_desired, Z_desired):

		r = math.sqrt(math.pow(X_desired,2)+math.pow(Y_desired,2)+math.pow(Z_desired,2))
		
		#self.q1 = 0.5*(math.acos((math.pow(Z_desired,2)-math.pow(X_desired,2))/(math.pow(X_desired,2)+math.pow(Z_desired,2))))   #original

		self.q1 = math.asin((X_desired)/(math.sqrt(math.pow(X_desired,2)+math.pow(Z_desired,2))))
		self.q2 = math.asin(-Y_desired/r)
		self.q3 = self.l_RCC - self.l_tool + r

		print("acos arg")
		#print(math.pow(X_desired,2)-math.pow(Y_desired,2))/(math.pow(X_desired,2)+math.pow(Z_desired,2))
		#print(arcsin_q1)
		#self.q1 = arcsin_q1/2
		print(self.q1)
		print("joints angles prescribed")
		print(self.q1*180/self.pi, self.q2*180/self.pi, self.q3)


	def set_position_robot(self, q1_set, q2_set, q3_set):
		
		#set joint values		
		psm_handle_base.set_joint_pos(0, q1_set)
		psm_handle_pfl.set_joint_pos(0, q2_set)
		psm_handle_pel.set_joint_pos(0, q3_set)


	def get_position_joints_PSM(self):

		self.q1_read = psm_handle_base.get_joint_pos(0)
		self.q2_read = psm_handle_base.get_joint_pos(3)
		self.q3_read = psm_handle_base.get_joint_pos(4)

		print("actual angles")
		print(self.q1_read*180/self.pi, self.q2_read*180/self.pi, self.q3_read)
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
		print("jacobian")
		print(self.jac)
		

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


	#reach a certain position defined by X and Y coordinates trying to mantain constant the force along Z 
	def reach_pos_XY(self, goal_x, goal_y, goal_z):

		self.inverse_kinematics(goal_x, goal_y, goal_z)
		self.set_position_robot(self.q1, self.q2, self.q3)
		time.sleep(0.01)
		self.get_position_joints_PSM()
		self.forward_kinematics(self.q1_read, self.q2_read, self.q3_read)
		print("\n")
		print("pos end loop")
		print(self.q1_read, self.q2_read, self.q3_read)

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
	'''
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
	m_start = 0.15
	psm_handle_pel.set_joint_pos(0, m_start)
	time.sleep(1)
	print(psm_handle_trl.get_pos())

	#psm_handle_pel.set_joint_pos(0, math.radians(0))
	#psm_handle_tyl.set_joint_pos(1, math.radians(-20))
	time.sleep(1)
	print("Start")
	time.sleep(1)
	'''
	cart_c = Cartesian_control()
	
	print("go in 3 seconds!!!")
	time.sleep(3)
	#cart_c.temp(math.radians(-20), math.radians(-30), 0.08)
	#cart_c.temp(math.radians(30), -0.0, 0.09)
	'''
	time.sleep(2)
	i = 0
	while i<0.15:
		cart_c.temp(0.0001, 0.000001, 0.00001-i)
		i = i+0.001
		print(i)

	while i<0.15:
		cart_c.temp(0.0001, 0.000001, 0.00001-i)
		i = i+0.001
		print(i)
	'''
	
	#cart_c.temp(0.0-0.08, 4.608079977738305e-05-0.04, -0.00042080074000368033-0.08)
	time.sleep(1)
	
	#cart_c.temp(-0.06, 0.04, 0.0)
	time.sleep(1)
	cart_c.temp(-0.09, -0.04, -0.04)
	'''
	cart_c.temp(6.751461962159085e-06-0.09, 4.608079977738305e-05-0.04, -0.00042080074000368033-0.08)
	time.sleep(1)
	cart_c.temp(6.751461962159085e-06-0.095, 4.608079977738305e-05-0.04, -0.00042080074000368033-0.08)
	time.sleep(1)
	#cart_c.temp(6.751461962159085e-06, 4.608079977738305e-05+0.09, -0.00042080074000368033+0.09)
	time.sleep(1)
	'''
	
	print('STEP1')



	time.sleep(5)
	
	
	#cart_c.plots()


	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()

