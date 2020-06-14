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

force_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/01_sp_kMT_f.csv', delimiter = ",")
forced_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/01_sp_kMT_fd.csv', delimiter = ",")
error_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/01_sp_kMT_er.csv', delimiter = ",")
time_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/01_sp_kMT_t.csv', delimiter = ",")
z_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/01_sp_kMT_z.csv', delimiter = ",")

force_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/02_sp_kMT_f.csv', delimiter = ",")
forced_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/02_sp_kMT_fd.csv', delimiter = ",")
error_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/02_sp_kMT_er.csv', delimiter = ",")
time_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/02_sp_kMT_t.csv', delimiter = ",")
z_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/02_sp_kMT_z.csv', delimiter = ",")

force_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kMT_f.csv', delimiter = ",")
forced_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kMT_fd.csv', delimiter = ",")
error_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kMT_er.csv', delimiter = ",")
time_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kMT_t.csv', delimiter = ",")
z_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/03_sp_kMT_z.csv', delimiter = ",")

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


error_sp_03 = [70.2899,   57.9853,   72.8039,   28.9588,   66.1893,   83.1695,   85.6725,   79.8754,   32.0344,   79.3267]

error_sp_02 = [39.7661,   45.5311,   26.5379,    6.5327,   23.7074,   42.6000,   49.2776,   34.8645,   66.6958,   29.8399]

error_sp_01 = [19.2341,   23.7284,   16.4003,    6.2673,   16.9293,   21.2740,   26.8107,   19.7937,   32.6104,   11.7567]




font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 30}

matplotlib.rc('font', **font)
matplotlib.rc('legend',fontsize=24)
			

fig, axes = plt.subplots(ncols=3, sharey=True)
#fig.suptitle('ERROR: dependency on kLST', fontsize=24)
fig.subplots_adjust(wspace=0)


bp0 = axes[0].boxplot(error_sp_03, patch_artist=True, showfliers=False, widths = 0.4)
for box in bp0['boxes']:
    # change outline color
    box.set( color='k', linewidth=1)
    # change fill color
    box.set( facecolor = '#3371ff')

    for whisker in bp0['whiskers']:
        whisker.set(color='k', linewidth=2)

    for cap in bp0['caps']:
        cap.set(color='k', linewidth=2)

    for median in bp0['medians']:
        median.set(color='k', linewidth=2)
    
    for flier in bp0['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)

axes[0].set_xticklabels(['kMT = 0.0005'],rotation=0)
#axes[0].tick_params(labelsize=18)
axes[0].set_ylabel('Integral force error [N]')#, fontsize=18)
axes[0].grid()


bp1 = axes[1].boxplot(error_sp_02, patch_artist=True, showfliers=False, widths = 0.4)
for box in bp1['boxes']:
    # change outline color
    box.set( color='k', linewidth=1)
    # change fill color
    box.set( facecolor = '#ff9f33')

    for whisker in bp1['whiskers']:
        whisker.set(color='k', linewidth=2)

    for cap in bp1['caps']:
        cap.set(color='k', linewidth=2)

    for median in bp1['medians']:
        median.set(color='k', linewidth=2)

    for flier in bp1['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)

axes[1].set_xticklabels(['kMT = 0.005'],rotation=0)#, fontsize=18)
axes[1].grid()


bp2 = axes[2].boxplot(error_sp_01, patch_artist=True, showfliers=False, widths = 0.4)
for box in bp2['boxes']:
    # change outline color
    box.set( color='k', linewidth=1)
    # change fill color
    box.set( facecolor = '#ff335e' )

    for whisker in bp2['whiskers']:
        whisker.set(color='k', linewidth=2)

    for cap in bp2['caps']:
        cap.set(color='k', linewidth=2)

    for median in bp2['medians']:
        median.set(color='k', linewidth=2)

    for flier in bp2['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)

axes[2].set_xticklabels(['kMT = 0.05'],rotation=0)#, fontsize=18)
axes[2].grid()


plt.show()















