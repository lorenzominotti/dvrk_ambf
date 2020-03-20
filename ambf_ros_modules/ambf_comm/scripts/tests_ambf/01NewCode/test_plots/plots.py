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

force_joint = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_sponge_joint_force.csv', delimiter = ",")
force_cart = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_sponge_cart_force.csv', delimiter = ",")
error_joint = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_sponge_joint_error.csv', delimiter = ",")
error_cart = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_sponge_cart_error.csv', delimiter = ",")

force_joint2 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_joint_force.csv', delimiter = ",")
force_cart2 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_cart_force.csv', delimiter = ",")
error_joint2 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_joint_error.csv', delimiter = ",")
error_cart2 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/01_cloth_cart_error.csv', delimiter = ",")

data_to_plot = [force_joint, force_cart]
data_to_plot_er = [error_joint, error_cart]
data_to_plot2 = [force_joint2, force_cart2]
data_to_plot_er2 = [error_joint2, error_cart2]
'''
fig = plt.figure(1)
ax = fig.add_subplot(111)
bp = ax.boxplot(data_to_plot2)
plt.show()
'''
'''
fig = plt.figure(1)
ax = fig.add_subplot(111)
bp = ax.boxplot(data_to_plot2, patch_artist=True)
for box in bp['boxes']:
    # change outline color
    box.set( color='#84ff9f', linewidth=2)
    # change fill color
    box.set( facecolor = '#1b9e77' )

## change color and linewidth of the whiskers
for whisker in bp['whiskers']:
    whisker.set(color='#7570b3', linewidth=2)

## change color and linewidth of the caps
for cap in bp['caps']:
    cap.set(color='#7570b3', linewidth=2)

## change color and linewidth of the medians
for median in bp['medians']:
    median.set(color='#b2df8a', linewidth=2)

## change the style of fliers and their fill
for flier in bp['fliers']:
    flier.set(marker='o', color='#e7298a', alpha=0.5)


ax.set_xticklabels(['Joint_control','Cart_control'])

ax.grid()

plt.show()
'''


fig, axes = plt.subplots(ncols=2, sharey=True)
fig.subplots_adjust(wspace=0)

name = 'Sponge'
bp1 = axes[0].boxplot(data_to_plot_er, patch_artist=True)
for box in bp1['boxes']:
    # change outline color
    box.set( color='#FFC42C', linewidth=2)
    # change fill color
    box.set( facecolor = '#FF002C')

    for whisker in bp1['whiskers']:
        whisker.set(color='#7570b3', linewidth=2)

    for cap in bp1['caps']:
        cap.set(color='#7570b3', linewidth=2)

    for median in bp1['medians']:
        median.set(color='#b2df8a', linewidth=2)

    for flier in bp1['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)

axes[0].set(xticklabels=['Joint_control', 'Cart_control'], xlabel=name)
axes[0].grid()

name = 'Cloth'
bp2 = axes[1].boxplot(data_to_plot_er2, patch_artist=True)
for box in bp2['boxes']:
    # change outline color
    box.set( color='#84ff9f', linewidth=2)
    # change fill color
    box.set( facecolor = '#1b9e77' )

    for whisker in bp2['whiskers']:
        whisker.set(color='#7570b3', linewidth=2)

    for cap in bp2['caps']:
        cap.set(color='#7570b3', linewidth=2)

    for median in bp2['medians']:
        median.set(color='#b2df8a', linewidth=2)

    for flier in bp2['fliers']:
        flier.set(marker='o', color='#e7298a', alpha=0.2)
axes[1].set(xticklabels=['Joint_control', 'Cart_control'], xlabel=name)
axes[1].grid()



plt.show()














