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


graph_tr = []
graph_rb = []
graph_ts = []
graph_sp = []
graph_tc = []
graph_cl = []

stop = 0



def callback_tr(data):
	global graph_tr
	graph_tr = np.append(graph_tr, data.data)
	

def callback_rb(data):
	global graph_rb
	graph_rb = np.append(graph_rb, data.data)
	

def callback_ts(data):
	global graph_ts
	graph_ts = np.append(graph_ts, data.data)
	

def callback_sp(data):
	global graph_sp
	graph_sp = np.append(graph_sp, data.data)
	

def callback_tc(data):
	global graph_tc
	graph_tc = np.append(graph_tc, data.data)
	

def callback_cl(data):
	global stop
	global graph_cl

	if data.data == 100:
		print("STOP RECEIVED")
		stop = 1
		test_plot()

	else:

		graph_cl = np.append(graph_cl,data.data)
		



def subs():
	global stop
	
	rospy.init_node('Monello')

    	sb_tr = rospy.Subscriber("time_rb", Float64, callback_tr)
	sb_rb = rospy.Subscriber("ins_rb", Float64, callback_rb)
	sb_ts = rospy.Subscriber("time_sp", Float64, callback_ts)
	sb_sp = rospy.Subscriber("ins_sp", Float64, callback_sp)
	sb_tc = rospy.Subscriber("time_cl", Float64, callback_tc)
	sb_cl = rospy.Subscriber("ins_cl", Float64, callback_cl)
	

    	rospy.spin()
	
def test_plot():

	print("PLOTS")

	global graph_tr 
	global graph_rb 
	global graph_ts
	global graph_sp 
	global graph_tc 
	global graph_cl 
	
	for i in range(0,len(graph_tr)):
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/time_rb.csv', graph_tr[i].reshape(1,), delimiter=",")

	for i in range(0,len(graph_rb)):
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/ins_rb.csv', graph_rb[i].reshape(1,), delimiter=",")

	for i in range(0,len(graph_ts)):
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/time_sp.csv', graph_ts[i].reshape(1,), delimiter=",")

	for i in range(0,len(graph_sp)):
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/ins_sp.csv', graph_sp[i].reshape(1,), delimiter=",")

	for i in range(0,len(graph_tc)):
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/time_cl.csv', graph_tc[i].reshape(1,), delimiter=",")

	for i in range(0,len(graph_cl)):
		np.savetxt('ambf/ambf_ros_modules/ambf_comm/scripts/tests_ambf/01NewCode/test_ins/ins_cl.csv', graph_cl[i].reshape(1,), delimiter=",")


	
	
	
	


	print(len(graph_tr))
	print(len(graph_rb))
	print(len(graph_ts))
	print(len(graph_sp))
	print(len(graph_tc))
	print(len(graph_cl))
	
	print("\n\n")

	v_len = {}
	v_len[0] = len(graph_tr)
	v_len[1] = len(graph_rb)
	v_len[2] = len(graph_ts)
	v_len[3] = len(graph_sp)
	v_len[4] = len(graph_tc)
	v_len[5] = len(graph_cl)
	
	limit = 5


	if v_len[0] == v_len[1]:
		max_len = v_len[0]

	if v_len[0] > v_len[1]:
		max_len = v_len[0]

	if v_len[0] < v_len[1]:
		max_len = v_len[1]
	
	graph_tr = graph_tr[0:(max_len-limit)]
	graph_rb = graph_rb[0:(max_len-limit)]



	if v_len[2] == v_len[3]:
		max_len = v_len[2]

	if v_len[2] > v_len[3]:
		max_len = v_len[2]

	if v_len[2] < v_len[3]:
		max_len = v_len[3]

	graph_ts = graph_ts[0:(max_len-limit)]
	graph_sp = graph_sp[0:(max_len-limit)]




	if v_len[4] == v_len[5]:
		max_len = v_len[4]

	if v_len[4] > v_len[5]:
		max_len = v_len[4]

	if v_len[4] < v_len[5]:
		max_len = v_len[5]

	graph_tc = graph_tc[0:(max_len-limit)]
	graph_cl = graph_cl[0:(max_len-limit)]
	


	print(len(graph_tr))
	print(len(graph_rb))
	print(len(graph_ts))
	print(len(graph_sp))
	print(len(graph_tc))
	print(len(graph_cl))
	
	

	font = {'family' : 'normal',
       		#'weight' : 'normal',
        	'size'   : 20}

	matplotlib.rc('font', **font)
	
	#fdim = 12
	fig, axs = plt.subplots(nrows = 3)
	fig.subplots_adjust(hspace=0.25)

	axs[0].plot(graph_tr, graph_rb, color = 'r', label = "force cylinder")
	axs[0].set(ylabel = 'Force [N]')
	#axs[0].set(xlabel = 'Time [s]')	
	axs[0].legend(loc='best')
	axs[0].grid()

	axs[1].plot(graph_ts, graph_sp, color = 'b', label = "force sponge")
	axs[1].set(ylabel = 'Force [N]')
	#axs[1].set(xlabel = 'Time [s]')	
	axs[1].legend(loc='best')
	axs[1].grid()

	axs[2].plot(graph_tc, graph_cl, color = 'g', label = "force cloth")
	axs[2].set(ylabel = 'Force [N]')
	axs[2].set(xlabel = 'Time [s]')	
	axs[2].legend(loc='best')
	axs[2].grid()
	
	
	plt.show()

	

	



if __name__ == "__main__":
	subs()
  		
    	

