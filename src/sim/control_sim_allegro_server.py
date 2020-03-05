#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
from grasp_pipeline.srv import *
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Char
from sensor_msgs.msg import JointState
import moveit_msgs.msg
import numpy as np

class ControlAllegroConfig:
    def __init__(self, publish_prefix='/allegro_hand_right'):
        rospy.init_node('control_allegro_config_node')
        self.allegro_joint_cmd_pub = rospy.Publisher(publish_prefix+'/joint_cmd',
                                                     JointState, queue_size=100)
        rospy.Subscriber('allegro_hand_right/joint_states', JointState, self.get_allegro_joint_state_cb)
        self.allegro_init_joint_state = None
        self.allegro_joint_state = None
        self.run_rate = rospy.Rate(20)
        self.control_allegro_steps = 50
        self.reach_gap_thresh = 0.1
        self.dof = 16

    def get_allegro_joint_state_cb(self, allegro_joint_state):
        if self.allegro_init_joint_state is None:
            self.allegro_init_joint_state = allegro_joint_state 
        self.allegro_joint_state = allegro_joint_state

    def control_allegro(self):
        self.run_rate.sleep()
        rospy.loginfo('Service control_allegro_config:')
        rospy.loginfo('Joint control command published')

        jc = JointState()
        jc.name = self.allegro_target_joint_state.name
        target_joint_angles = np.array(self.allegro_target_joint_state.position)
        init_joint_angles = np.array(self.allegro_init_joint_state.position)
        delta = (target_joint_angles - init_joint_angles) / self.control_allegro_steps
        jc_angles = init_joint_angles
        for i in xrange(self.control_allegro_steps):
            jc_angles += delta
            #print i, jc_angles
            jc.position = jc_angles.tolist()
            self.allegro_joint_cmd_pub.publish(jc)
            self.run_rate.sleep()

    def control_allegro_home(self):
        self.run_rate.sleep()
        rospy.loginfo('Service control_allegro_config:')
        rospy.loginfo('Joint control command published')

        jc = JointState()
        jc.name = self.allegro_init_joint_state.name
        target_joint_angles = np.zeros(self.dof)
        init_joint_angles = np.array(self.allegro_init_joint_state.position)
        delta = (target_joint_angles - init_joint_angles) / self.control_allegro_steps
        jc_angles = init_joint_angles
        for i in xrange(self.control_allegro_steps):
            jc_angles += delta
            jc.position = jc_angles.tolist()
            self.allegro_joint_cmd_pub.publish(jc)
            self.run_rate.sleep()


    def reach_goal(self):
        reach_gap = np.array(self.allegro_target_joint_state.position) - \
                np.array(self.allegro_joint_state.position)
        rospy.loginfo('Service control_allegro_config:')
        rospy.loginfo('reach_gap: ')
        rospy.loginfo(str(reach_gap))
        return np.min(np.abs(reach_gap)) < self.reach_gap_thresh

    def handle_control_allegro(self, req):
        response = AllegroConfigResponse()
        if req.go_home:
            self.control_allegro_home()
            response.success = True
        else:
            self.allegro_target_joint_state = req.allegro_target_joint_state
            self.control_allegro() 
            response.success = self.reach_goal()
        self.allegro_init_joint_state = None
        return response

    def create_control_allegro_server(self):
        control_allegro_service = rospy.Service('control_allegro_config', 
                                    AllegroConfig, self.handle_control_allegro) 
        rospy.loginfo('Service control_allegro_config:')
        rospy.loginfo('Ready to control allegro to speficied configurations.')

if __name__ == '__main__':
    control_allegro = ControlAllegroConfig()
    control_allegro.create_control_allegro_server()
    rospy.spin()

