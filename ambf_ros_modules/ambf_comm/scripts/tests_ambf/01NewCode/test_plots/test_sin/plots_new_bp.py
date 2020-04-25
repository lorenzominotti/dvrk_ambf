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

error_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_sp_sin_error.csv', delimiter = ",")
time_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_sp_sin_time.csv', delimiter = ",")

error_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_sp_sin_error.csv', delimiter = ",")
time_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_sp_sin_time.csv', delimiter = ",")

error_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_sp_sin_error.csv', delimiter = ",")
time_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_sp_sin_time.csv', delimiter = ",")

error_sp_04 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_sp_sin_error.csv', delimiter = ",")
time_sp_04 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_sp_sin_time.csv', delimiter = ",")

error_cl_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_cl_sin_error.csv', delimiter = ",")
time_cl_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_cl_sin_time.csv', delimiter = ",")

error_cl_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_cl_sin_error.csv', delimiter = ",")
time_cl_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_cl_sin_time.csv', delimiter = ",")

error_cl_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_cl_sin_error.csv', delimiter = ",")
time_cl_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_cl_sin_time.csv', delimiter = ",")

error_cl_04 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_cl_sin_error.csv', delimiter = ",")
time_cl_04 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_cl_sin_time.csv', delimiter = ",")





#print(len(force_sp_01))

data_to_plot0 = [error_rb_01, error_rb_02, error_rb_03, error_rb_04]

data_to_plot1 = [error_sp_01, error_sp_02, error_sp_03, error_sp_04]

data_to_plot2 = [error_cl_01, error_cl_02, error_cl_03, error_cl_04]

x1 = error_rb_01.mean(axis=0)
x2 = error_rb_02.mean(axis=0)
x3 = error_rb_03.mean(axis=0)
x4 = error_rb_04.mean(axis=0)

vect01 = [1, 2, 3, 4]
vect1 = [x1, x2, x3, x4]

y1 = error_sp_01.mean(axis=0)
y2 = error_sp_02.mean(axis=0)
y3 = error_sp_03.mean(axis=0)
y4 = error_sp_04.mean(axis=0)

vect02 = [1, 2, 3, 4]
vect2 = [y1, y2, y3, y4]

z1 = error_cl_01.mean(axis=0)
z2 = error_cl_02.mean(axis=0)
z3 = error_cl_03.mean(axis=0)
z4 = error_cl_04.mean(axis=0)

vect03 = [1, 2, 3, 4]
vect3 = [z1, z2, z3, z4]

fig, axes = plt.subplots(ncols=3, sharey=True)
#fig.suptitle('Force', fontsize=24)
fig.subplots_adjust(wspace=0)

name = 'Rigid body'
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

axes[0].plot(vect01, vect1, '#3371ff')

axes[0].set_xticklabels(['0.1 Hz', '0.5 Hz', '1 Hz', '5 Hz'],rotation=0)
axes[0].set_xlabel(name, fontsize=18)
axes[0].tick_params(labelsize=16)
axes[0].set_ylabel('Abs force error [N]', fontsize=18)
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

axes[1].plot(vect02, vect2, '#ff9f33')
axes[1].set_xticklabels(['0.1 Hz', '0.5 Hz', '1 Hz', '5 Hz'],rotation=0)
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
        flier.set(marker='o', color='#e7298a', alpha=0.2)

axes[2].plot(vect03, vect3, '#ff335e')

axes[2].set_xticklabels(['0.1 Hz', '0.5 Hz', '1 Hz', '5 Hz'],rotation=0)
axes[2].set_xlabel(name, fontsize=18)
axes[2].tick_params(labelsize=16)
axes[2].grid()




plt.show()



















































'''

#print(len(force_sp_01))

data_to_plot0 = [error_rb_01, error_sp_01, error_cl_01]

data_to_plot1 = [error_rb_02, error_sp_02, error_cl_02]

data_to_plot2 = [error_rb_03, error_sp_03, error_cl_03]

data_to_plot3 = [error_rb_04, error_sp_04, error_cl_04]



fig, axes = plt.subplots(ncols=4, sharey=True)
fig.suptitle('Force', fontsize=24)
fig.subplots_adjust(wspace=0)

name = 'f=0.1 Hz'
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

axes[0].set_xticklabels(['Rigid body', 'Sponge', 'Cloth'],rotation=0)
axes[0].set_xlabel(name, fontsize=18)
axes[0].tick_params(labelsize=18)
axes[0].set_ylabel('Abs force error [N]', fontsize=18)
axes[0].grid()

name = 'f=0.5 Hz'
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

axes[1].set_xticklabels(['Rigid body', 'Sponge', 'Cloth'],rotation=0, fontsize=18)
axes[1].set_xlabel(name, fontsize=18)
axes[1].grid()

name = 'f = 1 Hz'
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
axes[2].set_xticklabels(['Rigid body', 'Sponge', 'Cloth'],rotation=0, fontsize=18)
axes[2].set_xlabel(name, fontsize=18)
axes[2].grid()


name = 'f = 5 Hz'
bp2 = axes[3].boxplot(data_to_plot3, patch_artist=True, showfliers=False)
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
axes[3].set_xticklabels(['Rigid body', 'Sponge', 'Cloth'],rotation=0, fontsize=18)
axes[3].set_xlabel(name, fontsize=18)
axes[3].grid()


plt.show()
'''










