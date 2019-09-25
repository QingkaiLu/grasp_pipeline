# Contains functions for receiving joint topics and sending joint angles
import rospy
from sensor_msgs.msg import JointState
from trajectory_smoothing.srv import GetSmoothTraj

import sys
from rospkg import RosPack
rp=RosPack()
rp.list()
#path=rp.get_path('optimization_pkg')+'/scripts'
#sys.path.insert(0,path)
#from trajectory_pub import trajectoryServer
import numpy as np
import time

class robotTrajInterface:
    def __init__(self,arm_prefix='/lbr4',hand_prefix='/allegro_hand_right',init_node=True,traj_topic='/grasp/plan'):
        if(init_node):
            rospy.init_node('robot_node')
        # subscribers:
        self.arm_joint_state=JointState()
        self.arm_joint_sub=None
        self.arm_joint_sub=rospy.Subscriber(arm_prefix+'/joint_states',
                                        JointState,self.arm_joint_state_cb,self.arm_joint_sub)
        self.got_state=False
        self.got_hand_state=False
        self.hand_joint_state=JointState()
        self.hand_joint_sub=None
        self.hand_joint_sub=rospy.Subscriber(hand_prefix+'/joint_states',
                                        JointState,self.hand_joint_state_cb,self.hand_joint_sub)

        #self.traj_server=trajectoryServer(100,robot_name='lbr4',topic_name=traj_topic,init_node=False)


        self.pub=rospy.Publisher(arm_prefix+'/joint_cmd',JointState,queue_size=1)
        self.loop_rate=rospy.Rate(100)


    def arm_joint_state_cb(self,joint_state):
        self.arm_joint_state=joint_state
        self.got_state=True

    def hand_joint_state_cb(self,joint_state):
        self.hand_joint_state=joint_state
        self.got_hand_state=True

    #def viz_traj(self,j_traj):
    #    self.traj_server.viz_joint_traj(j_traj)
        
    def send_jtraj(self,j_traj):
        # before sending the joint trajectory, smoothly transfer to the initial waypoint:
        for i in range(len(j_traj.points)):
            new_jc=JointState()
            new_jc.name=j_traj.joint_names
            new_jc.position=j_traj.points[i].positions
            new_jc.velocity=j_traj.points[i].velocities
            new_jc.effort=j_traj.points[i].accelerations
            self.pub.publish(new_jc)
            self.loop_rate.sleep()

        des_js=j_traj.points[-1].positions
        reached=False
        
        start_time = time.time()
        # Check if goal is reached:
        while(reached==False):
            if time.time() - start_time >= 2:
                break
            self.loop_rate.sleep()
            err=np.linalg.norm(np.array(des_js)-np.array(self.arm_joint_state.position))
            if(err<0.01):
                reached=True
        rospy.loginfo('***Arm reached: %s' %str(reached))
        rospy.loginfo('***Arm reach error: %s' %str(err))

    def get_smooth_traj(self,jtraj):
        max_acc=np.ones(7)*0.25
        max_vel=np.ones(7)*0.4
        # call service for smoothing:
        rospy.wait_for_service('/get_smooth_trajectory')
        traj_call=rospy.ServiceProxy('/get_smooth_trajectory',GetSmoothTraj)
        resp=traj_call(jtraj,max_acc,max_vel,0.1,0.01)
        #resp=traj_call(jtraj,max_acc,max_vel,0.2,0.01)
        #print resp.smooth_traj
        #smooth_traj=resp.smooth_traj
        return resp.success, resp.smooth_traj

