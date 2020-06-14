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

time1 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/000_rb_cart_time.csv', delimiter = ",")
force_1 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/000_rb_cart_force.csv', delimiter = ",")
force_d1 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/000_rb_cart_forced.csv', delimiter = ",")



time2 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/001_rb_cart_time.csv', delimiter = ",")
force_2 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/001_rb_cart_force.csv', delimiter = ",")
force_d2 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/001_rb_cart_forced.csv', delimiter = ",")


er1 = []
er2 = []


for i in range(0,len(time1)):
	er1 = abs(force_1-force_d1)
	er2 = abs(force_2-force_d2)
	

print(len(time1))
print(len(force_1))
print(len(force_d1))
print(len(time2))
print(len(force_2))
print(len(force_d2))



font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 18}

matplotlib.rc('font', **font)
matplotlib.rc('legend',fontsize=15)
			
fig, axs = plt.subplots(nrows = 2, sharex=True)
		

axs[0].plot(time1, force_1, color = 'r', label = "F_case1")
axs[0].plot(time1, force_2, color = 'g', label = "F_case2")
axs[0].plot(time1, force_d1, color = 'b', label = "target force")
axs[0].set(ylabel = 'Force [N]')
			
axs[0].legend(loc='best')
axs[0].grid()

axs[1].plot(time1, er1, color = 'r', label = "er_case1")
axs[1].plot(time1, er2, color = 'g', label = "er_case2")
axs[1].set(ylabel = 'Abs_err [N]')
axs[1].set(xlabel = 'Time [s]')
			
axs[1].legend(loc='best')
axs[1].grid()

		
plt.show()




