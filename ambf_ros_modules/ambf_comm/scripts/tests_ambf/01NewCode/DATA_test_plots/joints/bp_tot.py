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


error_joint0[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_rb_joint_error.csv', delimiter = ",")
error_cart0[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_rb_cart_error.csv', delimiter = ",")
error_joint0[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/02_rb_joint_error.csv', delimiter = ",")
error_cart0[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/02_rb_cart_error.csv', delimiter = ",")
error_joint0[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/03_rb_joint_error.csv', delimiter = ",")
error_cart0[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/03_rb_cart_error.csv', delimiter = ",")
error_joint0[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/04_rb_joint_error.csv', delimiter = ",")
error_cart0[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/04_rb_cart_error.csv', delimiter = ",")
error_joint0[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/05_rb_joint_error.csv', delimiter = ",")
error_cart0[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/05_rb_cart_error.csv', delimiter = ",")


error_joint1[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_sp_joint_error.csv', delimiter = ",")
error_cart1[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_sp_cart_error.csv', delimiter = ",")
error_joint1[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/02_sp_joint_error.csv', delimiter = ",")
error_cart1[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/02_sp_cart_error.csv', delimiter = ",")
error_joint1[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/03_sp_joint_error.csv', delimiter = ",")
error_cart1[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/03_sp_cart_error.csv', delimiter = ",")
error_joint1[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/04_sp_joint_error.csv', delimiter = ",")
error_cart1[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/04_sp_cart_error.csv', delimiter = ",")
error_joint1[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/05_sp_joint_error.csv', delimiter = ",")
error_cart1[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/05_sp_cart_error.csv', delimiter = ",")



error_joint2[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_cl_joint_error.csv', delimiter = ",")
error_cart2[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_cl_cart_error.csv', delimiter = ",")
error_joint2[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/02_cl_joint_error.csv', delimiter = ",")
error_cart2[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/02_cl_cart_error.csv', delimiter = ",")
error_joint2[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/03_cl_joint_error.csv', delimiter = ",")
error_cart2[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/03_cl_cart_error.csv', delimiter = ",")
error_joint2[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/04_cl_joint_error.csv', delimiter = ",")
error_cart2[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/04_cl_cart_error.csv', delimiter = ",")
error_joint2[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/05_cl_joint_error.csv', delimiter = ",")
error_cart2[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/05_cl_cart_error.csv', delimiter = ",")


error_jointr = []
error_jointr.append(0)
error_cartr = []
error_cartr.append(0)

error_joints = []
error_joints.append(0)
error_carts = []
error_carts.append(0)


error_jointc = []
error_jointc.append(0)
error_cartc = []
error_cartc.append(0)




for i in range(0,5):
    
    error_jointr = np.concatenate((error_jointr, error_joint0[i]), axis=None)
    error_cartr = np.concatenate((error_cartr, error_cart0[i]), axis=None)

for i in range(0,5):
    error_joints = np.concatenate((error_joints, error_joint1[i]), axis=None)
    error_carts = np.concatenate((error_carts, error_cart1[i]), axis=None)

for i in range(0,5):
    error_jointc = np.concatenate((error_jointc, error_joint2[i]), axis=None)
    error_cartc = np.concatenate((error_cartc, error_cart2[i]), axis=None)

error_jointr = [21.7974  , 30.8278 ,  20.1343  , 18.7727  , 19.8583  , 13.6287  , 9.2185   , 8.7215  , 15.0089   , 9.3430]

error_cartr = [12.5111    ,4.5746  ,  2.7416  ,  2.5530  ,  5.6844  ,  6.3751  ,  4.6892   , 3.3240  , 10.2989 ,   4.1096]

error_joints = [54.1041   ,20.5375  ,28.7812  , 25.1608  , 26.7390  , 40.5484 ,  16.5055   , 6.9321 ,  18.2635  , 17.5966]

error_carts = [12.0129   ,15.8499   , 9.3993  ,  3.4528  , 10.3469   ,14.2460,   12.2888   , 8.1723 ,  10.5200  , 11.9173]

error_jointc = [29.2235  , 31.5103  , 22.7044  , 20.2594 ,  29.6914 ,  40.9722 ,  33.1287 ,  31.7512 ,  45.4093  , 28.4335]

error_cartc = [13.9892  , 18.0493   , 7.7075   , 2.0458  ,  8.2399  , 25.1384 ,  31.6357 ,  25.2515  , 20.1756 ,  18.8201]


data_to_plot_er0 = [error_jointr, error_cartr]


data_to_plot_er1 = [error_joints, error_carts]


data_to_plot_er2 = [error_jointc, error_cartc]


#data_to_plot_er0 = [error_jointr, error_cartr]


#data_to_plot_er1 = [error_joints, error_carts]


#data_to_plot_er2 = [error_jointc, error_cartc]

font = {'family' : 'normal',
        'size'   : 30}

matplotlib.rc('font', **font)
matplotlib.rc('legend',fontsize=15)



fig, axes = plt.subplots(ncols=3, sharey=True)
#fig.suptitle('Force', fontsize=24)
fig.subplots_adjust(wspace=0)


name = 'Rigid Body'
bp0 = axes[0].boxplot(data_to_plot_er0, patch_artist=True, showfliers=False, widths=0.5)
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

axes[0].set_xticklabels(['Joint_control', 'Cart_control'],rotation=0)
axes[0].set_xlabel(name, fontsize=22, color ='#3371ff')
axes[0].tick_params(labelsize=20)
axes[0].set_ylabel('Integral Force Error [N]', fontsize=20)
axes[0].grid()



name = 'Sponge'
bp1 = axes[1].boxplot(data_to_plot_er1, patch_artist=True, showfliers=False, widths=0.5)
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

axes[1].set_xticklabels(['Joint_control', 'Cart_control'],rotation=0, fontsize=18)
axes[1].set_xlabel(name, fontsize=22, color ='#ff9f33')
axes[1].tick_params(labelsize=20)
axes[1].grid()


name = 'Cloth'
bp2 = axes[2].boxplot(data_to_plot_er2, patch_artist=True, showfliers=False, widths=0.5)
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
        flier.set(marker='o', color='#00b33c', alpha=0.2)
axes[2].set_xticklabels(['Joint_control', 'Cart_control'],rotation=0, fontsize=18)
axes[2].set_xlabel(name, fontsize=22, color = '#ff335e')
axes[2].tick_params(labelsize=20)
axes[2].grid()

plt.show()













