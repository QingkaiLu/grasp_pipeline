import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import matplotlib.pyplot as plt

import numpy as np

def plot_j_acc_profile(j_traj,idx):

    for l in range(idx):
        # get joint_positions:
        j_arr=[]
        for i in range(len(j_traj.points)):
            j_arr.append(j_traj.points[i].positions[l])
            
        # compute acc:
        pos_arr=j_arr
        acc_prof=[]
        vel_prof=[]
        for t in range(1,len(pos_arr)-1):
            acc=pos_arr[t-1]-2.0*pos_arr[t]+pos_arr[t+1]
            acc_prof.append(acc)
            
        for t in range(1,len(pos_arr)):
            vel=-pos_arr[t-1]+pos_arr[t]
            vel_prof.append(vel)
        i=0
        plt.subplot(311)
        plt.plot(pos_arr,linewidth=4,label=str(l))
        plt.xlabel('Timestep')
        plt.ylabel('Joint Position(rad)')
        
        plt.subplot(312)
        plt.plot(vel_prof,linewidth=4,label=str(l))
        plt.xlabel('Timestep')
        plt.ylabel('Joint Velocity')
        plt.subplot(313)
        plt.plot(acc_prof,linewidth=4,label=str(l))
        plt.xlabel('Timestep')
        plt.ylabel('Joint acceleration')
        
    plt.legend()

    plt.show()
