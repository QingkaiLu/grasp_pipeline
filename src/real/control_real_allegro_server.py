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
import roslib.packages as rp
import sys
sys.path.append(rp.get_pkg_dir('hand_services') 
                + '/scripts')
from hand_client import handClient

class ControlAllegroConfig:
    def __init__(self, publish_prefix='/allegro_hand_right'):
        rospy.init_node('control_allegro_config_node')
        self.hand_client = handClient()
        self.dof = 16

    def control_allegro(self, control_type='p'):
        rospy.loginfo('Control allegro to target joint states.')

        target_joint_angles = np.array(self.allegro_target_joint_state.position)
        reached = self.hand_client.send_pos_cmd(target_joint_angles)

    def control_allegro_home(self, control_type='p'):
        rospy.loginfo('Control allegro to go home.')

        target_joint_angles = np.zeros(self.dof)
        reached = self.hand_client.send_pos_cmd(target_joint_angles)

    def close_hand_manually(self, control_type='p'):
        #self.allegro_ctrl_type_pub.publish(Char(data=ord(control_type)))
        self.run_rate.sleep()
        rospy.loginfo('Close allegro manually.')

        jc = JointState()
        jc.name = self.allegro_target_joint_state.name
        if(control_type=='p'):
            target_joint_angles = np.array(self.allegro_target_joint_state.position)
            init_joint_angles = np.array(self.allegro_init_joint_state.position)
            delta = (target_joint_angles - init_joint_angles) / self.control_allegro_steps
            jc_angles = init_joint_angles
            delta = np.array([0., 0.015, 0.015, 0.015,
                                0., 0.015, 0.015, 0.015,
                                0., 0.015, 0.015, 0.015,
                                0., 0., 0.015, 0.015,])
            for i in xrange(28):
                jc_angles += delta
                jc.position = jc_angles.tolist()
                self.allegro_joint_cmd_pub.publish(jc)
                self.run_rate.sleep()

    def handle_control_allegro(self, req):
        response = AllegroConfigResponse()
        if req.close_hand:
            self.close_hand_manually()
            response.success = True
        elif req.go_home:
            self.control_allegro_home()
            response.success = True
        else:
            self.allegro_target_joint_state = req.allegro_target_joint_state
            self.control_allegro() 
            response.success = True
            self.allegro_init_joint_state = None
        return response

    def create_control_allegro_server(self):
        control_allegro_service = rospy.Service('control_allegro_config', AllegroConfig, self.handle_control_allegro) 
        rospy.loginfo('Service control_allegro_config:')
        rospy.loginfo('Ready to control allegro to speficied configurations.')

if __name__ == '__main__':
    control_allegro = ControlAllegroConfig()
    control_allegro.create_control_allegro_server()
    rospy.spin()

