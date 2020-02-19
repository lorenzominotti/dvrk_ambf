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


	#reach a certain position defined by X and Y coordinates trying to mantain constant the force along Z 
	def reach_pos_XY(self, goal_x, goal_y, goal_z):
		
		pos_start = psm_handle_trl.get_pos()
		self.posx_start0 = pos_start.x
		self.posy_start0 = pos_start.y
		self.posz_start0 = pos_start.z

		self.internal_block(goal_x, goal_y, goal_z, self.posx_start0, self.posy_start0, self.posz_start0)
	


	def internal_block(self, X_desired, Y_desired, Z_desired, X_real, Y_real, Z_real):
		
		self.q1 = 0
		self.q2 = 0
		self.q3 = 0.16

		print("In internal block desired vs actual")
		print(X_desired, Y_desired, Z_desired)
		print(X_real, Y_real, Z_real)
		print("pos as the desired")
		print(X_desired-X_real, Y_desired-Y_real, Z_desired-Z_real)

		#compute positions variations in cartesian space
		deltaX = X_desired
		deltaY = Y_desired
		deltaZ = Z_desired

		#inverse kinematics
		pi = math.pi
		l_RCC = 0.4318
		l_tool = 0.4162
		r = math.sqrt(math.pow(deltaX,2)+math.pow(deltaY,2)+math.pow(deltaZ,2))

		self.q1 = pi/2 - (math.acos((math.pow(deltaX,2)-math.pow(deltaY,2))/(math.pow(deltaX,2)+math.pow(deltaZ,2))))/2
		#delta_q1 = (math.asin((math.pow(deltaX,2)-math.pow(deltaY,2))/(math.pow(deltaX,2)+math.pow(deltaZ,2))))
		self.q2 = math.asin(deltaZ/(math.sqrt(abs(-deltaY/r))))
		self.q3 = l_RCC - l_tool + r
		
		#set joint values
		
		psm_handle_base.set_joint_pos(0, self.q1)
		psm_handle_pfl.set_joint_pos(0, self.q2)
		psm_handle_pel.set_joint_pos(0, self.q3)
		time.sleep(1)
		print("\n")
		print("actual position after movement")
		pos_tool = psm_handle_trl.get_pos()
		px = pos_tool.x
		py = pos_tool.y
		pz = pos_tool.z

		#px = math.cos(self.q2)*math.sin(self.q1)*(l_tool-l_RCC+self.q3)
		#py = -math.sin(self.q2)*(l_tool-l_RCC+self.q3)
		#pz = -math.cos(self.q1)*math.cos(self.q2)*(l_tool-l_RCC+self.q3)

		print(px, py, pz)
		print(px-X_real, py-Y_real, pz-Z_real)
		


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
	#psm_handle_pfl.set_joint_pos(0, math.radians(-60))
	psm_handle_pfl.set_joint_pos(0, 0)
	psm_handle_base.set_joint_pos(0, math.radians(0))
	time.sleep(2)




	#psm_handle_pel.set_joint_pos(0, math.radians(0))
	#psm_handle_tyl.set_joint_pos(1, math.radians(-20))
	time.sleep(1)
	print("Start")
	time.sleep(1)

	q1 = math.radians(30)
	q2 = math.radians(60)
	q3 = 0.14
	psm_handle_base.set_joint_pos(0, math.radians(30))
	psm_handle_pfl.set_joint_pos(0, math.radians(60))
	psm_handle_pel.set_joint_pos(0, 0.14)
	pos_tool = psm_handle_trl.get_pos()
	px = pos_tool.x
	py = pos_tool.y
	pz = pos_tool.z
	time.sleep(1)
	print("position trl AMBF")
	print(px, py, pz)

	time.sleep(2)

	pi = math.pi
	l_RCC = 0.4318
	l_tool = 0.4162

	px = math.cos(q2)*math.sin(q1)*(l_tool-l_RCC+q3)
	py = -math.sin(q2)*(l_tool-l_RCC+q3)
	pz = -math.cos(q1)*math.cos(q2)*(l_tool-l_RCC+q3)

	print("position FK")
	print(px, py, pz)

	print(psm_handle_base.get_all_joint_pos())
	print(psm_handle_base.get_joint_pos(3))
	print(psm_handle_base.get_joint_pos(4))
	# i want the 0, the 3 and the 4
	




	'''
	cart_c = Cartesian_control()
	
	print("go in 3 seconds!!!")
	time.sleep(3)
	cart_c.reach_pos_XY(0.08, -0.1, -0.02)
	print('STEP1')
	time.sleep(5)
	
	
	#cart_c.plots()

	'''
	raw_input("Let's clean up. Press Enter to continue...")
	# Lastly to cleanup
	_client.clean_up()

if __name__ == "__main__":
    main()

