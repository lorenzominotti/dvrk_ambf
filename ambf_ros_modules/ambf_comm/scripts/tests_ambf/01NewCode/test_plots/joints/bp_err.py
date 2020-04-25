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



ej_rb = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_rb_joint_error.csv', delimiter = ",")
ec_rb = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_rb_cart_error.csv', delimiter = ",")
ej_sp = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_sp_joint_error.csv', delimiter = ",")
ec_sp = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_sp_cart_error.csv', delimiter = ",")
ej_cl = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/02_cl_joint_error.csv', delimiter = ",")
ec_cl = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/joints/01_cl_cart_error.csv', delimiter = ",")


data_to_plot0 = [ej_rb, ec_rb]

data_to_plot1 = [ej_sp, ec_sp]

data_to_plot2 = [ej_cl, ec_cl]



fig, axes = plt.subplots(ncols=3, sharey=True)
#fig.suptitle('Force', fontsize=24)
fig.subplots_adjust(wspace=0)

name = 'Rigid Body'
bp0 = axes[0].boxplot(data_to_plot0, patch_artist=True, showfliers=False)
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
axes[0].set_xlabel(name, fontsize=18)
axes[0].tick_params(labelsize=16)
axes[0].set_ylabel('Abs force err [N]', fontsize=18)
axes[0].grid()

name = 'Sponge'
bp1 = axes[1].boxplot(data_to_plot1, patch_artist=True, showfliers=False)
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
axes[1].set_xlabel(name, fontsize=18)
axes[1].tick_params(labelsize=16)
axes[1].grid()

name = 'Cloth'
bp2 = axes[2].boxplot(data_to_plot2, patch_artist=True, showfliers=False)
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
axes[2].set_xlabel(name, fontsize=18)
axes[2].tick_params(labelsize=16)
axes[2].grid()


plt.show()















