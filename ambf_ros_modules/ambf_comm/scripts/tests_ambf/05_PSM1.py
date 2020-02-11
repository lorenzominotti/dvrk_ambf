#!/usr/bin/env python2.7
# Import the Client from ambf_client package
from ambf_client import Client
import time
import math

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
psm_handle_dev = _client.get_obj_handle('psm/simulated_device_1/omega7')

# Let's sleep for a very brief moment to give the internal callbacks
# to sync up new data from the running simulator
time.sleep(0.2)
'''
print('\n\n----')
raw_input("Let's Get Some Pose Info. Press Enter to continue...")
# Not we can print the pos and rotation of object in the World Frame
print('PSM Base Pos:')
print(psm_handle_base.get_pos())

print('\n\n----')
raw_input("Let's get Joints and Children Info. Press Enter to continue...")
# We can get the number of children and joints connected to each object as
psm_num_joints = psm_handle_base.get_num_joints() # Get the number of joints of this object
psm_children_names = psm_handle_base.get_children_names() # Get a list of children names belonging to this obj
print('Number of Joints in PSM:')
print(psm_num_joints)

print(' ')
print('Name of PSM\'s children:')
print(psm_children_names)

print('\n\n----')
raw_input("Control PSMs joint positions. Press Enter to continue...")
# Now let's control some joints
# The 1st joint
psm_handle_base.set_joint_pos(0, math.radians(45))
time.sleep(2)
#psm_handle_pfl.set_joint_pos(0, math.radians(-45))
#time.sleep(2)
# psm_handle_mi.set_joint_pos(1, 10) #problemmmmmmm
'''
print('\n\n----')

raw_input("Number of joints of mainInsertionLink")
num_joints_mi = psm_handle_dev.get_num_joints()
print(num_joints_mi)

raw_input("Name of joints of mainInsertionLink")
name_joints_mi = psm_handle_dev.get_joint_names()
print(name_joints_mi)

raw_input("Number of joints of pitchEndLink")
num_joints_pel = psm_handle_dev.get_num_joints()
print(num_joints_pel)

raw_input("Name of joints of pitchEndLink")
name_joints_pel = psm_handle_dev.get_joint_names()
print(name_joints_pel)

raw_input("Display movement...")
#psm_handle_pfl.set_joint_pos(0, math.radians(-60))
psm_handle_dev.set_joint_pos(0, 0)
psm_handle_dev.set_joint_pos(0, math.radians(0))
time.sleep(2)

#psm_handle_mi.set_joint_pos(0, 0)
psm_handle_dev.set_joint_pos(0, 0)
time.sleep(1)
#psm_handle_pel.set_joint_pos(0, 0.1)
#time.sleep(1)
psm_handle_dev.set_joint_pos(0, 0)
time.sleep(1)

'''	
for degree in range(0, 60):
	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	time.sleep(0.1)	
for degree in range(-60, 60):
	psm_handle_pfl.set_joint_pos(0, math.radians(degree))
	time.sleep(0.1)	
for degree in range(-60, 60):
	psm_handle_pfl.set_joint_pos(0, math.radians(-degree))
	time.sleep(0.1)	
'''
'''
for degree in range(0, 60):
	psm_handle_pfl.set_joint_pos(0, math.radians(-60 + degree))
	time.sleep(0.1)	
'''
cm = 0
while cm < 0.25:
	psm_handle_dev.set_joint_pos(0, cm)
	cm = cm +0.005
        time.sleep(0.2)

#for cm in range(0, 10):
        #print("movement")
	#psm_handle_pel.set_joint_pos(0, cm/100)
	#time.sleep(0.1)
'''
time.sleep(5)
# The 2nd joint
psm_handle.set_joint_pos(1, math.radians(45))

print('\n\n----')
raw_input("Mixed Pos and Effort control of PSM\'s joints. Press Enter to continue...")
# For the PSM let's control some in position and some in effort mode
# The 1st joint, which the PSM Yaw
psm_handle.set_joint_effort(0, 0.5)
# The 3rd Kinematic joint, which is the PSM Insertion Joint
psm_handle.set_joint_pos(3, -0.2)

print('\n\n----')
raw_input("Set force on MTM's Wrist Yaw link for 5 secs. Press Enter to continue...")
# Let's directly control the forces and torques on the mtmWristYaw Link
# Notice that these are in the world frame. Another important thing to notice
# is that unlike position control, forces control requires a continuous update
# to meet a watchdog timing condition otherwise the forces are reset Null. This
# is purely for safety reasons to prevent unchecked forces in case of malfunctioning
# python client code
for i in range(0, 500):
    mtm_wrist_handle.set_force(0, 0, 10) # Set 10 N in the World Z axis
    time.sleep(0.01) # Run the loop for 10 seconds

print('\n\n----')
raw_input("Set wrench on MTM's Wrist Yaw link for 5 secs. Press Enter to continue...")
# Similarly we can individually apply the torque
for i in range(0, 500):
    mtm_wrist_handle.set_torque(0, 3, 0) # Set 10 Nm in the World Y axis
    time.sleep(0.01) # Run the loop for 10 seconds
'''
print('\n\n----')
raw_input("Let's clean up. Press Enter to continue...")
# Lastly to cleanup
_client.clean_up()

