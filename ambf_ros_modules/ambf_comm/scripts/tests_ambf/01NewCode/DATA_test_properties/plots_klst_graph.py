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
import numpy as np
from numpy import asarray
from numpy import savetxt
from numpy import loadtxt
import pandas as pd
# load array

force_sp_01= [] 
forced_sp_01= [] 
error_sp_01= [] 
time_sp_01= []

force_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/01_sp_kLST_f.csv', delimiter = ",")
forced_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/01_sp_kLST_fd.csv', delimiter = ",")
error_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/01_sp_kLST_er.csv', delimiter = ",")
time_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/01_sp_kLST_t.csv', delimiter = ",")
z_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/01_sp_kLST_z.csv', delimiter = ",")

force_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/02_sp_kLST_f.csv', delimiter = ",")
forced_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/02_sp_kLST_fd.csv', delimiter = ",")
error_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/02_sp_kLST_er.csv', delimiter = ",")
time_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/02_sp_kLST_t.csv', delimiter = ",")
z_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/02_sp_kLST_z.csv', delimiter = ",")

force_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kLST_f.csv', delimiter = ",")
forced_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kLST_fd.csv', delimiter = ",")
error_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kLST_er.csv', delimiter = ",")
time_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kLST_t.csv', delimiter = ",")
z_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kLST_z.csv', delimiter = ",")

print(len(force_sp_01))
print(len(force_sp_02))
print(len(force_sp_03))

#print(len(force_sp_01))

minlen = len(force_sp_01)
if len(force_sp_02) < minlen:
	minlen = len(force_sp_02)
if len(force_sp_03) < minlen:
	minlen = len(force_sp_03)

time = time_sp_01[0:minlen]
force_sp_01 = force_sp_01[0:minlen]
force_sp_02 = force_sp_02[0:minlen]
force_sp_03 = force_sp_03[0:minlen]

forced_sp_01 = forced_sp_01[0:minlen]
forced_sp_02 = forced_sp_02[0:minlen]
forced_sp_03 = forced_sp_03[0:minlen]

error_sp_01 = error_sp_01[0:minlen]
error_sp_02 = error_sp_02[0:minlen]
error_sp_03 = error_sp_03[0:minlen]

z_sp_01 = z_sp_01[0:minlen]
z_sp_02 = z_sp_02[0:minlen]
z_sp_03 = z_sp_03[0:minlen]


print(len(force_sp_01))
print(len(force_sp_02))
print(len(force_sp_03))


font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 30}

matplotlib.rc('font', **font)
matplotlib.rc('legend',fontsize=24)
matplotlib.rcParams['lines.linewidth'] = 3
			
fig, axs = plt.subplots(nrows = 2, sharex=True)
fig.subplots_adjust(hspace=0.10)

axs[0].plot(time, force_sp_03, color = '#3371ff', label = "F kLST=0.008")
axs[0].plot(time, force_sp_02, color = '#ff9f33', label = "F kLST=0.08")
axs[0].plot(time, force_sp_01, color = '#ff335e', label = "F kLST=0.8")
axs[0].plot(time, forced_sp_01, color = 'k', label = "F des")
#axs[0].set_title('SPONGE', fontsize=24)
axs[0].set(ylabel = 'Force [N]')	
axs[0].legend(loc='upper right')
axs[0].grid()
'''
axs[1].plot(time, error_sp_03, color = 'g', label = "err k=0.008")
axs[1].plot(time, error_sp_02, color = '#ff9f33', label = "err k=0.08")
axs[1].plot(time, error_sp_01, color = 'r', label = "err k=0.8")		
axs[1].set(ylabel = 'Abs_err [N]')	
axs[1].legend(loc='best')
axs[1].grid()
'''
axs[1].plot(time, z_sp_03, color = '#3371ff', label = "Z kLST=0.008")
axs[1].plot(time, z_sp_02, color = '#ff9f33', label = "Z kLST=0.08")
axs[1].plot(time, z_sp_01, color = '#ff335e', label = "Z kLST=0.8")
axs[1].set(ylabel = 'Z pos [m]')
axs[1].set(xlabel = 'Time [s]')
axs[1].legend(loc='lower right')
axs[1].grid()

plt.show()















