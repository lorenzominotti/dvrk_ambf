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



error_rb_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_rb_sin_error.csv', delimiter = ",")
time_rb_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_rb_sin_time.csv', delimiter = ",")

error_rb_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_rb_sin_error.csv', delimiter = ",")
time_rb_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_rb_sin_time.csv', delimiter = ",")

error_rb_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_rb_sin_error.csv', delimiter = ",")
time_rb_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_rb_sin_time.csv', delimiter = ",")

error_rb_04 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_rb_sin_error.csv', delimiter = ",")
time_rb_04 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_rb_sin_time.csv', delimiter = ",")

error_rb_05 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/05_rb_sin_error.csv', delimiter = ",")
time_rb_05 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/05_rb_sin_time.csv', delimiter = ",")


'''
print(len(force_sp_01))
print(len(force_sp_02))
print(len(force_sp_03))
'''

#print(len(force_sp_01))

minlen = len(error_rb_01)
if len(error_rb_02) < minlen:
	minlen = len(error_rb_02)
if len(error_rb_03) < minlen:
	minlen = len(error_rb_03)
if len(error_rb_04) < minlen:
	minlen = len(error_rb_04)
if len(error_rb_05) < minlen:
	minlen = len(error_rb_05)


error_rb_01 = error_rb_01[0:minlen]
error_rb_02 = error_rb_02[0:minlen]
error_rb_03 = error_rb_03[0:minlen]
error_rb_04 = error_rb_04[0:minlen]
error_rb_05 = error_rb_05[0:minlen]



font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 18}

matplotlib.rc('font', **font)
matplotlib.rc('legend',fontsize=15)
			


fig, axes = plt.subplots(ncols=5, sharey=True)
#fig.suptitle('ERROR: dependency on kLST', fontsize=24)
fig.subplots_adjust(wspace=0)


bp0 = axes[0].boxplot(error_rb_01, patch_artist=True, showfliers=False)
for box in bp0['boxes']:
    # change outline color
    box.set( color='k', linewidth=2)
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

axes[0].set_xticklabels(['0.1HZ'],rotation=0)
axes[0].tick_params(labelsize=18)
axes[0].set_ylabel('Abs force error [N]', fontsize=18)
axes[0].grid()


bp1 = axes[1].boxplot(error_rb_02, patch_artist=True, showfliers=False)
for box in bp1['boxes']:
    # change outline color
    box.set( color='k', linewidth=2)
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

axes[1].set_xticklabels(['0.5HZ'],rotation=0, fontsize=18)
axes[1].grid()


bp2 = axes[2].boxplot(error_rb_03, patch_artist=True, showfliers=False)
for box in bp2['boxes']:
    # change outline color
    box.set( color='k', linewidth=2)
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

axes[2].set_xticklabels(['1HZ'],rotation=0, fontsize=18)
axes[2].grid()


bp2 = axes[3].boxplot(error_rb_04, patch_artist=True, showfliers=False)
for box in bp2['boxes']:
    # change outline color
    box.set( color='k', linewidth=2)
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

axes[3].set_xticklabels(['5HZ'],rotation=0, fontsize=18)
axes[3].grid()


bp2 = axes[4].boxplot(error_rb_05, patch_artist=True, showfliers=False)
for box in bp2['boxes']:
    # change outline color
    box.set( color='k', linewidth=2)
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

axes[4].set_xticklabels(['10HZ'],rotation=0, fontsize=18)
axes[4].grid()







plt.show()












