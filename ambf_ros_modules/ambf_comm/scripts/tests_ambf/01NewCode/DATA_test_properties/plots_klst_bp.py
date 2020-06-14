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

error_sp_03 = [75.7637, 84.5931, 73.4688, 19.1294, 61.6297, 93.1376, 100.3705, 115.7667, 32.0860, 71.5211]
error_sp_02 = [49.4302, 53.2296, 64.1358, 11.5610, 59.3762, 61.4421, 66.6548,  84.0497,   32.5793,   40.4609]
error_sp_01 = [51.0943, 48.9852, 38.4587, 12.2759,   25.3412,   42.5305,   44.3070,   37.9084,   57.7909,   27.5247]




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

axes[0].set_xticklabels(['kLST = 0.008'],rotation=0)
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

axes[1].set_xticklabels(['kLST = 0.08'],rotation=0)#, fontsize=18)
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

axes[2].set_xticklabels(['kLST = 0.8'],rotation=0)#, fontsize=18)
axes[2].grid()


plt.show()















