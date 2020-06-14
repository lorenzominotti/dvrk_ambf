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

force_joint0= [None] * 5
force_cart0= [None] * 5
error_joint0= [None] * 5
error_cart0= [None] * 5 

force_joint1=[None] * 5
force_cart1=[None] * 5
error_joint1=[None] * 5
error_cart1=[None] * 5

force_joint2=[None] * 5
force_cart2=[None] * 5
error_joint2=[None] * 5
error_cart2=[None] * 5


force_joint0[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_rb_joint_force.csv', delimiter = ",")
force_cart0[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_rb_cart_force.csv', delimiter = ",")
error_joint0[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_rb_joint_error.csv', delimiter = ",")
error_cart0[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_rb_cart_error.csv', delimiter = ",")
force_joint0[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_rb_joint_force.csv', delimiter = ",")
force_cart0[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_rb_cart_force.csv', delimiter = ",")
error_joint0[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_rb_joint_error.csv', delimiter = ",")
error_cart0[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_rb_cart_error.csv', delimiter = ",")
force_joint0[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_rb_joint_force.csv', delimiter = ",")
force_cart0[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_rb_cart_force.csv', delimiter = ",")
error_joint0[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_rb_joint_error.csv', delimiter = ",")
error_cart0[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_rb_cart_error.csv', delimiter = ",")
force_joint0[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_rb_joint_force.csv', delimiter = ",")
force_cart0[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_rb_cart_force.csv', delimiter = ",")
error_joint0[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_rb_joint_error.csv', delimiter = ",")
error_cart0[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_rb_cart_error.csv', delimiter = ",")
force_joint0[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_joint_force.csv', delimiter = ",")
force_cart0[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_cart_force.csv', delimiter = ",")
error_joint0[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_joint_error.csv', delimiter = ",")
error_cart0[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_rb_cart_error.csv', delimiter = ",")

force_joint1[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_sponge_joint_force.csv', delimiter = ",")
force_cart1[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_sponge_cart_force.csv', delimiter = ",")
error_joint1[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_sponge_joint_error.csv', delimiter = ",")
error_cart1[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_sponge_cart_error.csv', delimiter = ",")
force_joint1[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_sponge_joint_force.csv', delimiter = ",")
force_cart1[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_sponge_cart_force.csv', delimiter = ",")
error_joint1[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_sponge_joint_error.csv', delimiter = ",")
error_cart1[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_sponge_cart_error.csv', delimiter = ",")
force_joint1[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_sponge_joint_force.csv', delimiter = ",")
force_cart1[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_sponge_cart_force.csv', delimiter = ",")
error_joint1[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_sponge_joint_error.csv', delimiter = ",")
error_cart1[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_sponge_cart_error.csv', delimiter = ",")
force_joint1[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_sponge_joint_force.csv', delimiter = ",")
force_cart1[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_sponge_cart_force.csv', delimiter = ",")
error_joint1[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_sponge_joint_error.csv', delimiter = ",")
error_cart1[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_sponge_cart_error.csv', delimiter = ",")
force_joint1[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_sponge_joint_force.csv', delimiter = ",")
force_cart1[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_sponge_cart_force.csv', delimiter = ",")
error_joint1[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_sponge_joint_error.csv', delimiter = ",")
error_cart1[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_sponge_cart_error.csv', delimiter = ",")

force_joint2[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_joint_force.csv', delimiter = ",")
force_cart2[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_cart_force.csv', delimiter = ",")
error_joint2[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_joint_error.csv', delimiter = ",")
error_cart2[0] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_cart_error.csv', delimiter = ",")
force_joint2[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_cloth_joint_force.csv', delimiter = ",")
force_cart2[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_cloth_cart_force.csv', delimiter = ",")
error_joint2[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_cloth_joint_error.csv', delimiter = ",")
error_cart2[1] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/02_cloth_cart_error.csv', delimiter = ",")
force_joint2[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_cloth_joint_force.csv', delimiter = ",")
force_cart2[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_cloth_cart_force.csv', delimiter = ",")
error_joint2[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_cloth_joint_error.csv', delimiter = ",")
error_cart2[2] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/03_cloth_cart_error.csv', delimiter = ",")
force_joint2[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_cloth_joint_force.csv', delimiter = ",")
force_cart2[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_cloth_cart_force.csv', delimiter = ",")
error_joint2[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_cloth_joint_error.csv', delimiter = ",")
error_cart2[3] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/04_cloth_cart_error.csv', delimiter = ",")
force_joint2[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_cloth_joint_force.csv', delimiter = ",")
force_cart2[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_cloth_cart_force.csv', delimiter = ",")
error_joint2[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_cloth_joint_error.csv', delimiter = ",")
error_cart2[4] = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/05_cloth_cart_error.csv', delimiter = ",")

force_jointr = []
force_jointr.append(2)
force_cartr = []
force_cartr.append(2)
error_jointr = []
error_jointr.append(0)
error_cartr = []
error_cartr.append(0)

force_joints = []
force_joints.append(2)
force_carts = []
force_carts.append(2)
error_joints = []
error_joints.append(0)
error_carts = []
error_carts.append(0)

force_jointc = []
force_jointc.append(2)
force_cartc = []
force_cartc.append(2)
error_jointc = []
error_jointc.append(0)
error_cartc = []
error_cartc.append(0)




for i in range(0,5):
    force_jointr = np.concatenate((force_jointr, force_joint0[i]), axis=None)
    force_cartr = np.concatenate((force_cartr, force_cart0[i]), axis=None)
    error_jointr = np.concatenate((error_jointr, error_joint0[i]), axis=None)
    error_cartr = np.concatenate((error_cartr, error_cart0[i]), axis=None)

for i in range(0,5):
    force_joints = np.concatenate((force_joints, force_joint1[i]), axis=None)
    force_carts = np.concatenate((force_carts, force_cart1[i]), axis=None)
    error_joints = np.concatenate((error_joints, error_joint1[i]), axis=None)
    error_carts = np.concatenate((error_carts, error_cart1[i]), axis=None)

for i in range(0,5):
    force_jointc = np.concatenate((force_jointc, force_joint2[i]), axis=None)
    force_cartc = np.concatenate((force_cartc, force_cart2[i]), axis=None)
    error_jointc = np.concatenate((error_jointc, error_joint2[i]), axis=None)
    error_cartc = np.concatenate((error_cartc, error_cart2[i]), axis=None)
    
print(force_jointr)

data_to_plot0 = [force_jointr, force_cartr]
data_to_plot_er0 = [error_jointr, error_cartr]

data_to_plot1 = [force_joints, force_carts]
data_to_plot_er1 = [error_joints, error_carts]

data_to_plot2 = [force_jointc, force_cartc]
data_to_plot_er2 = [error_jointc, error_cartc]




fig, axes = plt.subplots(ncols=3, sharey=True)
fig.suptitle('Force', fontsize=24)
fig.subplots_adjust(wspace=0)

name = 'Rigid Body'
bp0 = axes[0].boxplot(data_to_plot0, patch_artist=True, showfliers=False)
for box in bp0['boxes']:
    # change outline color
    box.set( color='#3D9AF9', linewidth=2)
    # change fill color
    box.set( facecolor = '#80DAEB')

    for whisker in bp0['whiskers']:
        whisker.set(color='#7570b3', linewidth=2)

    for cap in bp0['caps']:
        cap.set(color='#7570b3', linewidth=2)

    for median in bp0['medians']:
        median.set(color='#3D9AF9', linewidth=2)
    
    for flier in bp0['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)

axes[0].set_xticklabels(['Joint_control', 'Cart_control'],rotation=0)
axes[0].set_xlabel(name, fontsize=18)
axes[0].tick_params(labelsize=18)
axes[0].set_ylabel('Force [N]', fontsize=18)
axes[0].grid()

name = 'Sponge'
bp1 = axes[1].boxplot(data_to_plot1, patch_artist=True, showfliers=False)
for box in bp1['boxes']:
    # change outline color
    box.set( color='#FF002C', linewidth=2)
    # change fill color
    box.set( facecolor = '#FFC42C')

    for whisker in bp1['whiskers']:
        whisker.set(color='#7570b3', linewidth=2)

    for cap in bp1['caps']:
        cap.set(color='#7570b3', linewidth=2)

    for median in bp1['medians']:
        median.set(color='#FF002C', linewidth=2)

    for flier in bp1['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)

axes[1].set_xticklabels(['Joint_control', 'Cart_control'],rotation=0, fontsize=18)
axes[1].set_xlabel(name, fontsize=18)
axes[1].grid()

name = 'Cloth'
bp2 = axes[2].boxplot(data_to_plot2, patch_artist=True, showfliers=False)
for box in bp2['boxes']:
    # change outline color
    box.set( color='#1b9e77', linewidth=2)
    # change fill color
    box.set( facecolor = '#84ff9f' )

    for whisker in bp2['whiskers']:
        whisker.set(color='#7570b3', linewidth=2)

    for cap in bp2['caps']:
        cap.set(color='#7570b3', linewidth=2)

    for median in bp2['medians']:
        median.set(color='#1b9e77', linewidth=2)

    for flier in bp2['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)
axes[2].set_xticklabels(['Joint_control', 'Cart_control'],rotation=0, fontsize=18)
axes[2].set_xlabel(name, fontsize=18)
axes[2].grid()


plt.show()



fig, axes = plt.subplots(ncols=3, sharey=True)
fig.suptitle('Force_error', fontsize=24)
fig.subplots_adjust(wspace=0)

name = 'Rigid Body'
bp0 = axes[0].boxplot(data_to_plot_er0, patch_artist=True, showfliers=False)
for box in bp0['boxes']:
    # change outline color
    box.set( color='#3D9AF9', linewidth=2)
    # change fill color
    box.set( facecolor = '#80DAEB')

    for whisker in bp0['whiskers']:
        whisker.set(color='#7570b3', linewidth=2)

    for cap in bp0['caps']:
        cap.set(color='#7570b3', linewidth=2)

    for median in bp0['medians']:
        median.set(color='#3D9AF9', linewidth=2)

    for flier in bp0['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)

axes[0].set_xticklabels(['Joint_control', 'Cart_control'],rotation=0)
axes[0].set_xlabel(name, fontsize=18)
axes[0].tick_params(labelsize=18)
axes[0].set_ylabel('Force_error_norm', fontsize=18)
axes[0].grid()

name = 'Sponge'
bp1 = axes[1].boxplot(data_to_plot_er1, patch_artist=True, showfliers=False)
for box in bp1['boxes']:
    # change outline color
    box.set( color='#FF002C', linewidth=2)
    # change fill color
    box.set( facecolor = '#FFC42C')

    for whisker in bp1['whiskers']:
        whisker.set(color='#7570b3', linewidth=2)

    for cap in bp1['caps']:
        cap.set(color='#7570b3', linewidth=2)

    for median in bp1['medians']:
        median.set(color='#FF002C', linewidth=2)

    for flier in bp1['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)

axes[1].set_xticklabels(['Joint_control', 'Cart_control'],rotation=0, fontsize=18)
axes[1].set_xlabel(name, fontsize=18)
axes[1].grid()

name = 'Cloth'
bp2 = axes[2].boxplot(data_to_plot_er2, patch_artist=True, showfliers=False)
for box in bp2['boxes']:
    # change outline color
    box.set( color='#1b9e77', linewidth=2)
    # change fill color
    box.set( facecolor = '#84ff9f' )

    for whisker in bp2['whiskers']:
        whisker.set(color='#7570b3', linewidth=2)

    for cap in bp2['caps']:
        cap.set(color='#7570b3', linewidth=2)

    for median in bp2['medians']:
        median.set(color='#1b9e77', linewidth=2)

    for flier in bp2['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)
axes[2].set_xticklabels(['Joint_control', 'Cart_control'],rotation=0, fontsize=18)
axes[2].set_xlabel(name, fontsize=18)
axes[2].grid()

plt.show()













