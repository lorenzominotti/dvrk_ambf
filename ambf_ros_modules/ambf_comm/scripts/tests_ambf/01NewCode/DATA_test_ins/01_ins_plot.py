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

ins_rb = [] 
t_rb = [] 
ins_sp = [] 
t_sp = []
ins_cl = [] 
t_cl = []

ins_rb = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/rb_ins_f.csv', delimiter = ",")
t_rb = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/rb_ins_t.csv', delimiter = ",")
ins_sp = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/sp_ins_f.csv', delimiter = ",")
t_sp = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/sp_ins_t.csv', delimiter = ",")
ins_cl = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/cl_ins_f.csv', delimiter = ",")
t_cl = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/cl_ins_t.csv', delimiter = ",")



font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 30}

matplotlib.rc('font', **font)
matplotlib.rc('legend',fontsize=24)
matplotlib.rcParams['lines.linewidth'] = 3
			
fig, axs = plt.subplots(nrows = 3, sharex=True)
fig.subplots_adjust(hspace=0.10)

custom_xlim = (0,35.5)
custom_ylim = (-0.1,6)
plt.setp(axs, xlim = custom_xlim, ylim = custom_ylim)

axs[0].plot(t_rb, ins_rb, color = '#3374ff', label = "Rigid body")
axs[0].set(ylabel = 'Force [N]')	
axs[0].legend(loc='upper left')
axs[0].grid()

axs[1].plot(t_sp, ins_sp, color = '#ff5b33', label = "Sponge")
#axs[1].set(ylabel = 'Force [N]')
axs[1].set(ylabel = 'Force [N]')	
axs[1].legend(loc='upper left')
axs[1].grid()

axs[2].plot(t_cl, ins_cl, color = '#2d730f', label = "Cloth")
#axs[1].set(ylabel = 'Force [N]')	
axs[2].legend(loc='upper left')
axs[2].set(ylabel = 'Force [N]')
axs[2].grid()
axs[2].set(xlabel = 'Time [s]')

plt.show()















