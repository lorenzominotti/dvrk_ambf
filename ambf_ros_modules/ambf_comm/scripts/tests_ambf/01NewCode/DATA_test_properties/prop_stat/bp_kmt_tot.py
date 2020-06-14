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



error_joint0= [None] * 5
error_cart0= [None] * 5 


error_joint1=[None] * 5
error_cart1=[None] * 5


error_joint2=[None] * 5
error_cart2=[None] * 5


error_joint0[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/11_sp_kMT_er.csv', delimiter = ",")
error_joint0[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/12_sp_kMT_er.csv', delimiter = ",")
error_joint0[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/13_sp_kMT_er.csv', delimiter = ",")
error_joint0[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/14_sp_kMT_er.csv', delimiter = ",")
error_joint0[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/15_sp_kMT_er.csv', delimiter = ",")


error_joint1[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/21_sp_kMT_er.csv', delimiter = ",")
error_joint1[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/22_sp_kMT_er.csv', delimiter = ",")
error_joint1[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/23_sp_kMT_er.csv', delimiter = ",")
error_joint1[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/24_sp_kMT_er.csv', delimiter = ",")
error_joint1[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/25_sp_kMT_er.csv', delimiter = ",")


error_joint2[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/31_sp_kMT_er.csv', delimiter = ",")
error_joint2[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/32_sp_kMT_er.csv', delimiter = ",")
error_joint2[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/33_sp_kMT_er.csv', delimiter = ",")
error_joint2[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/34_sp_kMT_er.csv', delimiter = ",")
error_joint2[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_properties/prop_stat/35_sp_kMT_er.csv', delimiter = ",")



error_jointr = []
error_jointr.append(0)


error_joints = []
error_joints.append(0)


error_jointc = []
error_jointc.append(0)




for i in range(0,5):
    
    error_jointr = np.concatenate((error_jointr, error_joint0[i]), axis=None)
    
for i in range(0,5):
    error_joints = np.concatenate((error_joints, error_joint1[i]), axis=None)
    

for i in range(0,5):
    error_jointc = np.concatenate((error_jointc, error_joint2[i]), axis=None)
   

data_to_plot_er0 = [70.2899,   57.9853   ,72.8039  , 28.9588   ,66.1893 ,  83.1695   ,85.6725 ,  79.8754   ,32.0344,   79.3267]


data_to_plot_er1 = [39.7661 ,  45.5311   ,26.5379   , 6.5327   ,23.7074  , 42.6000  , 49.2776  , 34.8645  , 66.6958 ,  29.8399]


data_to_plot_er2 = [19.2341  , 23.7284  , 16.4003    ,6.2673  , 16.9293   ,21.2740 ,  26.8107  , 19.7937 ,  32.6104  , 11.7567]


#data_to_plot_er0 = error_jointr


#data_to_plot_er1 = error_joints


#data_to_plot_er2 = error_jointc


font = {'family' : 'normal',
       	#'weight' : 'normal',
        'size'   : 18}

matplotlib.rc('font', **font)
matplotlib.rc('legend',fontsize=15)

			

fig, axes = plt.subplots(ncols=3, sharey=True)
#fig.suptitle('ERROR: dependency on kLST', fontsize=24)
fig.subplots_adjust(wspace=0)


bp0 = axes[0].boxplot(data_to_plot_er0, patch_artist=True, showfliers=False,widths=0.3)
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

axes[0].set_xticklabels(['kMT=0.0005'],rotation=0)
axes[0].tick_params(labelsize=20)
axes[0].set_ylabel('Integral Force Error [N]', fontsize=20)
axes[0].grid()


bp1 = axes[1].boxplot(data_to_plot_er1, patch_artist=True, showfliers=False,widths=0.3)
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

axes[1].set_xticklabels(['kMT=0.005'],rotation=0, fontsize=20)
axes[1].grid()


bp2 = axes[2].boxplot(data_to_plot_er2, patch_artist=True, showfliers=False,widths=0.3)
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

axes[2].set_xticklabels(['kMT=0.05'],rotation=0, fontsize=20)
axes[2].grid()


plt.show()









