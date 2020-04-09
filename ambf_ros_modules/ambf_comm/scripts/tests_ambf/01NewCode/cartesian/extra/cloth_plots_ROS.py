#!/usr/bin/env python2.7
from __future__ import division
# Import the Client from ambf_client package
import time
import math
import rospy
from std_msgs.msg import Float64
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from scipy import signal
from numpy import asarray
from numpy import savetxt

	
xd_plot = []
yd_plot = []
zd_plot = []
xr_plot = []
yr_plot = []
zr_plot = []
time_plot = []

graph_f2 = []
error_force2 = []

force_raw = []
graph_f = []
force1 = []
force_vect = []
error_force = []
error_abs = []
error_pos = []
fr_x = []
fr_y = []
fr_z = []
er_x = []
er_y = []
er_z = []
graph_px = []
graph_py = []
graph_pz = []
graph_frn = []
graph_fd = []
z = []
	
time = []
time_ef = []
stop = 0



def callback_fd(data):
	global graph_fd
	graph_fd.append(data.data)

def callback_ea(data):
	global error_abs
	error_abs.append(data.data)

def callback_ef(data):
	global error_force
	error_force.append(data.data)

def callback_ex(data):
	global er_x
	er_x.append(data.data)

def callback_ey(data):
	global er_y
	er_y.append(data.data)

def callback_ez(data):
	global er_z
	er_z.append(data.data)

def callback_px(data):
	global graph_px
	graph_px.append(data.data)

def callback_py(data):
	global graph_py
	graph_py.append(data.data)

def callback_pz(data):
	global graph_pz
	if data.data == 0:
		data.data =  -0.24
	graph_pz.append(data.data)

def callback_t(data):
	global time
	time.append(data.data)

def callback_z(data):
	global z
	if data.data == 0:
		data.data = -0.235 #-0.2425rb, -0.22 sponge -235 cloth
	z.append(data.data)

def callback_f(data):
	global stop
	print(data.data)

	if data.data == 100:
		stop = 1
		test_plot()
	else:
		graph_f.append(data.data)


def subs():
	global stop
	
	rospy.init_node('Monello')

    	sb_fd = rospy.Subscriber("force_desired", Float64, callback_fd)
	sb_ea = rospy.Subscriber("force_ea", Float64, callback_ea)
	sb_ef = rospy.Subscriber("force_error", Float64, callback_ef)
	sb_ex = rospy.Subscriber("posx_er", Float64, callback_ex)
	sb_ey = rospy.Subscriber("posy_er", Float64, callback_ey)
	sb_ez = rospy.Subscriber("posz_er", Float64, callback_ez)
	sb_px = rospy.Subscriber("posX", Float64, callback_px)
	sb_py = rospy.Subscriber("posY", Float64, callback_py)
	sb_pz = rospy.Subscriber("posZ", Float64, callback_pz)
	sb_t = rospy.Subscriber("time", Float64, callback_t)
	sb_f = rospy.Subscriber("force_read", Float64, callback_f)
	sb_ = rospy.Subscriber("posZr", Float64, callback_z)

    	rospy.spin()
	
def test_plot():

	print("RICEVUTO CAPITANO, ECCOLI DI SEGUITO")
	
	global graph_f
	global error_abs
	global graph_fd
	global error_force
	global er_x
	global er_y
	global er_z
	global graph_px
	global graph_py
	global time
	global graph_pz
	global z


	print(len(graph_f))
	print(len(graph_fd))
	print(len(error_force))
	print(len(er_x))
	print(len(er_y))
	print(len(er_z))
	print(len(graph_px))
	print(len(graph_py))
	print(len(time))
	print(len(graph_pz))
	print(len(error_abs))
	print(len(z))
	print("\n\n")

	v_len = {}
	v_len[0] = len(graph_f)
	v_len[1] = len(graph_fd)
	v_len[2] = len(error_force)
	v_len[3] = len(er_x)
	v_len[4] = len(er_y)
	v_len[5] = len(er_z)
	v_len[6] = len(graph_px)
	v_len[7] = len(graph_py)
	v_len[8] = len(time)
	v_len[9] = len(graph_pz)
	v_len[10] = len(error_abs)
	v_len[11] = len(z)

	max_len = v_len[0]
	for i in range(1,len(v_len)):
		if v_len[i] > max_len:
			max_len = v_len[i]
	
	limit = 25
	graph_f = graph_f[0:(max_len-limit)]
	graph_fd = graph_fd[0:(max_len-limit)]
	error_force = error_force[0:(max_len-limit)]
	er_x = er_x[0:(max_len-limit)]
	er_y = er_y[0:(max_len-limit)]
	er_z = er_z[0:(max_len-limit)]
	graph_px = graph_px[0:(max_len-limit)]
	graph_py = graph_py[0:(max_len-limit)] #-1 for sponge bad y and z
	time = time[0:(max_len-limit)]
	graph_pz = graph_pz[0:(max_len-limit)] #
	error_abs = error_abs[0:(max_len-limit)]
	z = z[0:(max_len-limit)]


	print(len(graph_f))
	print(len(graph_fd))
	print(len(error_force))
	print(len(er_x))
	print(len(er_y))
	print(len(er_z))
	print(len(graph_px))
	print(len(graph_py))
	print(len(time))
	print(len(graph_pz))
	print(len(error_abs))
	print(len(z))

	print("f         ", graph_f[max_len-limit-1], graph_f[max_len-limit-2])
	print("fd         ", graph_fd[max_len-limit-1], graph_fd[max_len-limit-2])

	print("X         ", graph_px[max_len-limit-1], graph_px[max_len-limit-2])
	print("Y         ", graph_py[max_len-limit-1], graph_py[max_len-limit-2])

	#print(graph_py)
	
	'''
	print("f         ", graph_f[max_len-limit], graph_f[max_len-limit-1], graph_f[max_len-limit-2])
	print("fd         ", graph_fd[max_len-limit], graph_fd[max_len-limit-1], graph_fd[max_len-limit-2])

	print("X         ", graph_px[max_len-limit], graph_px[max_len-limit-1], graph_px[max_len-limit-2])
	print("Y         ", graph_py[max_len-limit], graph_py[max_len-limit-1], graph_py[max_len-limit-2])
	
	
	fdim = 12
	fig, axs = plt.subplots(nrows = 6)

	axs[0].plot(time, graph_f, color = 'r', label = "actual force")
	axs[0].plot(time, graph_fd, color = 'b', label = "target force")
	axs[0].set(ylabel = 'Force [N]')
	#axs[0].set_xticklabels([],rotation=0, fontsize=1)
	#axs[0].tick_params(labelsize=fdim)	
	axs[0].legend(loc='best')
	axs[0].grid()

	axs[1].plot(time, error_force, color = 'r', label = "error")
	axs[1].set(ylabel = 'Force_error_norm')
	#axs[1].set_xticklabels([],rotation=0, fontsize=1)
	#axs[1].tick_params(labelsize=fdim)	
	axs[1].legend(loc='best')
	axs[1].grid()

	axs[2].plot(time, er_x, label = "err_posx")
	axs[2].set(ylabel = 'posx_error [m]')
	#axs[2].set_xticklabels([],rotation=0, fontsize=1)
	#axs[2].tick_params(labelsize=fdim)	
	axs[2].legend(loc='best')
	axs[2].grid()

	axs[3].plot(time, er_y, label = "err_posy")
	axs[3].set(ylabel = 'posy_error [m]')
	#axs[3].set_xticklabels([],rotation=0, fontsize=1)
	#axs[3].tick_params(labelsize=fdim)	
	axs[3].legend(loc='best')
	axs[3].grid()

	axs[4].plot(time, er_z, label = "err_posz")
	axs[4].set(ylabel = 'posz_error [m]')
	axs[4].set(xlabel = 'Time [s]')	
	#axs[4].set_xticklabels([],rotation=0, fontsize=1)	
	#axs[4].tick_params(labelsize=fdim)
	axs[4].legend(loc='best')
	axs[4].grid()

	axs[5].plot(time, graph_px, label = "posx")
	axs[5].plot(time, graph_py, label = "posy")
	axs[5].set(ylabel = 'position xy [m]')
	axs[5].set(xlabel = 'Time [s]')	
	#axs[5].set_xticklabels([],rotation=0, fontsize=1)
	#axs[5].tick_params(labelsize=fdim)	
	axs[5].legend(loc='best')
	axs[5].grid()
	
	plt.show()
	'''
	for i in range(0,(max_len-limit)):
		graph_px[i] = graph_px[i]*100
		graph_py[i] = graph_py[i]*100
		#graph_pz[i] = graph_pz[i]*100
		er_x[i] = er_x[i]*1000
		er_y[i] = er_y[i]*1000
		er_z[i] = er_z[i]*1000

	font = {'family' : 'normal',
       		#'weight' : 'normal',
        	'size'   : 20}

	matplotlib.rc('font', **font)
	'''
	#fdim = 12
	fig, axs = plt.subplots(nrows = 4, sharex=True)
	fig.subplots_adjust(hspace=0.25)

	axs[0].plot(time, graph_f, color = 'r', label = "actual force")
	axs[0].plot(time, graph_fd, color = 'b', label = "target force")
	axs[0].set(ylabel = 'Force [N]')
	#axs[0].set_xticklabels([],rotation=0, fontsize=1)
	#axs[0].tick_params(labelsize=fdim)	
	axs[0].legend(loc='best')
	axs[0].grid()

	axs[1].plot(time, error_abs, color = 'r', label = "abs_error")
	axs[1].set(ylabel = 'Abs_F_err [N]')
	#axs[1].set_xticklabels([],rotation=0, fontsize=1)
	#axs[1].tick_params(labelsize=fdim)
	axs[1].legend(loc='best')
	axs[1].grid()

	axs[2].plot(time, error_force, color = 'r', label = "norm_error")
	axs[2].set(ylabel = 'Norm_F_er [-]')
	#axs[2].set_xticklabels([],rotation=0, fontsize=1)
	#axs[2].tick_params(labelsize=fdim)
	axs[2].legend(loc='best')
	axs[2].grid()

	axs[3].plot(time, graph_px, label = "posx")
	axs[3].plot(time, graph_py, label = "posy")
	axs[3].set(ylabel = 'pos_xy [cm]')
	#axs[3].set(xlabel = 'Time [s]')	
	#axs[3].set_xticklabels([],rotation=0, fontsize=1)
	#axs[3].tick_params(labelsize=fdim)
	axs[3].set(xlabel = 'Time [s]')	
	axs[3].legend(loc='best')
	axs[3].grid()

	
	plt.show()
	'''

	#fdim = 12
	fig, axs = plt.subplots(nrows = 3, sharex=True)
	fig.subplots_adjust(hspace=0.25)

	axs[0].plot(time, graph_f, color = 'r', label = "actual force")
	axs[0].plot(time, graph_fd, color = 'b', label = "target force")
	axs[0].set(ylabel = 'Force [N]')
	#axs[0].set_xticklabels([],rotation=0, fontsize=1)
	#axs[0].tick_params(labelsize=fdim)	
	axs[0].legend(loc='best')
	axs[0].grid()

	axs[1].plot(time, error_abs, color = 'r', label = "abs_error")
	axs[1].set(ylabel = 'Abs_F_err [N]')
	#axs[1].set_xticklabels([],rotation=0, fontsize=1)
	#axs[1].tick_params(labelsize=fdim)
	axs[1].legend(loc='best')
	axs[1].grid()

	axs[2].plot(time, graph_px, label = "posx")
	axs[2].plot(time, graph_py, label = "posy")
	axs[2].set(ylabel = 'pos_xy [cm]')
	#axs[3].set(xlabel = 'Time [s]')	
	#axs[3].set_xticklabels([],rotation=0, fontsize=1)
	#axs[3].tick_params(labelsize=fdim)
	axs[2].set(xlabel = 'Time [s]')	
	axs[2].legend(loc='best')
	axs[2].grid()

	
	plt.show()




	fig, axs = plt.subplots(nrows = 4, sharex=True)
	fig.subplots_adjust(hspace=0.25)

	axs[0].plot(time, graph_px, color = 'b', label = "posx")
	axs[0].plot(time, graph_py, color = 'r', label = "posy")
	axs[0].set(ylabel = 'pos_X_Y [cm]')
	#axs[0].set(xlabel = 'Time [s]')	
	#axs[0].set_xticklabels([],rotation=0, fontsize=1)
	#axs[0].tick_params(labelsize=fdim)	
	axs[0].legend(loc='best', fontsize = 'small')
	axs[0].grid()

	axs[1].plot(time, graph_pz, color = 'g', label = "posz" )
	axs[1].set(ylabel = 'pos_z [cm]')
	#axs[1].set(xlabel = 'Time [s]')	
	#axs[1].set_xticklabels([],rotation=0, fontsize=1)
	#axs[1].tick_params(labelsize=fdim)	
	axs[1].legend(loc='best', fontsize = 'small')
	axs[1].grid()	
	
	axs[2].plot(time, er_x, color = 'b', label = "err_posx")
	axs[2].plot(time, er_y, color = 'r', label = "err_posy")
	axs[2].set(ylabel = 'X_Y_er [mm]')
	#axs[2].set_xticklabels([],rotation=0, fontsize=1)
	#axs[2].tick_params(labelsize=fdim)	
	axs[2].legend(loc='3', fontsize = 'small')
	axs[2].grid()
	'''
	axs[3].plot(time, er_y, color = 'r', label = "err_posy")
	axs[3].set(ylabel = 'Y_er [mm]')
	#axs[3].set_xticklabels([],rotation=0, fontsize=1)
	#axs[3].tick_params(labelsize=fdim)	
	axs[3].legend(loc='best')
	axs[3].grid()
	'''
	axs[3].plot(time, er_z, color = 'g', label = "err_posz")
	axs[3].set(ylabel = 'Z_er [mm]')
	axs[3].set(xlabel = 'Time [s]')	
	#axs[4].set_xticklabels([],rotation=0, fontsize=1)	
	#axs[4].tick_params(labelsize=fdim)
	axs[3].set(xlabel = 'Time [s]')
	axs[3].legend(loc='3', fontsize = 'small')
	axs[3].grid()

	
	plt.show()

	
	



if __name__ == "__main__":
	subs()
  		
    	

