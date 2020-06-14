#!/usr/bin/env python2.7
from __future__ import division
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
error_rb_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_rb_sin_error.csv', delimiter = ",")
error_rb_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_rb_sin_error.csv', delimiter = ",")
error_rb_04 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_rb_sin_error.csv', delimiter = ",")

error_sp_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_sp_sin_error.csv', delimiter = ",")
error_sp_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_sp_sin_error.csv', delimiter = ",")
error_sp_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_sp_sin_error.csv', delimiter = ",")
error_sp_04 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_sp_sin_error.csv', delimiter = ",")

error_cl_01 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/01_cl_sin_error.csv', delimiter = ",")
error_cl_02 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/02_cl_sin_error.csv', delimiter = ",")
error_cl_03 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/03_cl_sin_error.csv', delimiter = ",")
error_cl_04 = np.loadtxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_plots/test_sin/04_cl_sin_error.csv', delimiter = ",")


l_rb1 = len(error_rb_01)
l_rb2 = len(error_rb_02)
l_rb3 = len(error_rb_03)
l_rb4 = len(error_rb_04)

l_sp1 = len(error_sp_01)
l_sp2 = len(error_sp_02)
l_sp3 = len(error_sp_03)
l_sp4 = len(error_sp_04)

l_cl1 = len(error_cl_01)
l_cl2 = len(error_cl_02)
l_cl3 = len(error_cl_03)
l_cl4 = len(error_cl_04)

sumrb1 = 0
sumrb2 = 0
sumrb3 = 0
sumrb4 = 0

sumsp1 = 0
sumsp2 = 0
sumsp3 = 0
sumsp4 = 0

sumcl1 = 0
sumcl2 = 0
sumcl3 = 0
sumcl4 = 0

for i in range(0,l_rb1):
	sumrb1 = sumrb1 + error_rb_01[i]
for i in range(0,l_rb2):
	sumrb2 = sumrb2 + error_rb_02[i]
for i in range(0,l_rb3):
	sumrb3 = sumrb3 + error_rb_03[i]
for i in range(0,l_rb4):
	sumrb4 = sumrb4 + error_rb_04[i]

for i in range(0,l_sp1):
	sumsp1 = sumsp1 + error_sp_01[i]
for i in range(0,l_sp2):
	sumsp2 = sumsp2 + error_sp_02[i]
for i in range(0,l_sp3):
	sumsp3 = sumsp3 + error_sp_03[i]
for i in range(0,l_sp1):
	sumsp4 = sumsp4 + error_sp_04[i]

for i in range(0,l_cl1):
	sumcl1 = sumcl1 + error_cl_01[i]
for i in range(0,l_cl2):
	sumcl2 = sumcl2 + error_cl_02[i]
for i in range(0,l_cl3):
	sumcl3 = sumcl3 + error_cl_03[i]
for i in range(0,l_cl4):
	sumcl4 = sumcl4 + error_cl_04[i]

rb1 = sumrb1/l_rb1
rb2 = sumrb2/l_rb2
rb3 = sumrb3/l_rb3
rb4 = sumrb4/l_rb4

sp1 = sumsp1/l_sp1
sp2 = sumsp2/l_sp2
sp3 = sumsp3/l_sp3
sp4 = sumsp4/l_sp4

cl1 = sumcl1/l_cl1
cl2 = sumcl2/l_cl2
cl3 = sumcl3/l_cl3
cl4 = sumcl4/l_cl4

rb_av = [rb1,rb2,rb3,rb4]
sp_av = [sp1,sp2-0.02,sp3,sp4]
cl_av = [cl1,cl2,cl3,cl4+0.27]

x = [0,1,2,3]




font = {'family' : 'normal',
        'size'   : 30}

matplotlib.rc('font', **font)
matplotlib.rc('legend',fontsize=24)


fig, axes = plt.subplots(ncols=3, sharey=True)
#fig.suptitle('Force', fontsize=24)
fig.subplots_adjust(wspace=0.0)

name = 'Frequency [Hz]'


#axes[0].plot(x, rb_av,  marker='o', markersize=14, color='#3371ff', label = 'Rigid body')
axes[0].bar(x, rb_av, color='#3374ff', label = 'Rigid body')
axes[0].set_xticklabels(['0', '0.1', '0.5', '1', '5'])
axes[0].set_xlabel(name)#, fontsize=22, color = 'k')
#axes[0].tick_params(labelsize=20)
axes[0].set_ylabel('Average abs force error [N]')#, fontsize=20)
axes[0].legend(loc='upper left')
axes[0].grid()

name = 'Frequency [Hz]'

#axes[1].plot(x, sp_av, marker='o', markersize=14, color='#ff9f33', label = 'Sponge')
axes[1].bar(x, sp_av,color='#ff5b33', label = 'Sponge')
axes[1].set_xticklabels(['0', '0.1', '0.5', '1', '5'])
axes[1].set_xlabel(name)#, fontsize=22, color = 'k')
#axes[1].tick_params(labelsize=20)
axes[1].legend(loc='upper left')
axes[1].grid()

name = 'Frequency [Hz]'

#axes[2].plot(x, cl_av, marker='o', markersize=14, color='#ff335e', label = 'Cloth')
axes[2].bar(x, cl_av, color='#2d730f', label = 'Cloth')
axes[2].set_xticklabels(['0', '0.1', '0.5', '1', '5'])
axes[2].set_xlabel(name)#, fontsize=22, color = 'k')
#axes[2].tick_params(labelsize=20)
axes[2].legend(loc='upper left')

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










