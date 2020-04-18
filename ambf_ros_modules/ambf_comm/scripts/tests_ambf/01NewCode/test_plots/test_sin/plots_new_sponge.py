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

force_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_sp_sin_force.csv', delimiter = ",")
forced_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_sp_sin_forced.csv', delimiter = ",")
error_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_sp_sin_error.csv', delimiter = ",")
time_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_sp_sin_time.csv', delimiter = ",")

force_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_sp_sin_force.csv', delimiter = ",")
forced_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_sp_sin_forced.csv', delimiter = ",")
error_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_sp_sin_error.csv', delimiter = ",")
time_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_sp_sin_time.csv', delimiter = ",")

force_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_sp_sin_force.csv', delimiter = ",")
forced_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_sp_sin_forced.csv', delimiter = ",")
error_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_sp_sin_error.csv', delimiter = ",")
time_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_sp_sin_time.csv', delimiter = ",")

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


print(len(force_sp_01))
print(len(force_sp_02))
print(len(force_sp_03))


font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 18}

matplotlib.rc('font', **font)
matplotlib.rc('legend',fontsize=15)
	
#plt.legend(fontsize=20)
#plt.rc('legend',fontsize=20)

fig, axs = plt.subplots(nrows = 4, sharex=True)
fig.subplots_adjust(hspace=0.25)

axs[0].plot(time, force_sp_01, color = 'r', label = "F_read 0.1Hz")
axs[0].plot(time, forced_sp_01, color = 'b', label = "F_taget 0.1Hz")	
axs[0].set(ylabel = 'Force [N]')	
axs[0].set_title('SPONGE', fontsize=24)
axs[0].legend(loc='best')
axs[0].grid()

axs[1].plot(time, force_sp_02, color = 'r', label = "F_read 0.5Hz")
axs[1].plot(time, forced_sp_02, color = 'b', label = "F_taget 0.5Hz")	
axs[1].set(ylabel = 'Force [N]')	
axs[1].legend(loc='best')
axs[1].grid()

axs[2].plot(time, force_sp_03, color = 'r', label = "F_read 1Hz")
axs[2].plot(time, forced_sp_03, color = 'b', label = "F_taget 1Hz")	
axs[2].set(ylabel = 'Force [N]')	
axs[2].legend(loc='best')
axs[2].grid()
	
axs[3].plot(time, error_sp_01, color = 'r', label = "err 0.1Hz")
axs[3].plot(time, error_sp_02, color = 'b', label = "err 0.5Hz")
axs[3].plot(time, error_sp_03, color = 'g', label = "err 1Hz")
axs[3].set(xlabel = 'Time [s]', ylabel = 'Abs_err [N]')
axs[3].legend(loc='best')
axs[3].grid()

plt.show()














