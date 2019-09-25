#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from grasp_pipeline.srv import *

#To do: change this file to a ros server to load different objects.
class ManageSceneInMoveit:
    def __init__(self, gazebo_model_state_topic='/gazebo/model_states'):
        #roscpp_initialize(sys.argv)
        rospy.init_node('manage_moveit_scene_node')
        self.use_sim = rospy.get_param('~use_sim', False)
        #if self.use_sim:
        #    rospy.Subscriber(gazebo_model_state_topic, ModelStates, self.get_world_states)
        self.set_box_table_properties()
        self.box_exist = False

    def set_box_table_properties(self):
        self.box_len_x = 0.9125
        self.box_len_y = 0.61
        self.box_len_z = 0.59

        self.box_pose = PoseStamped()
        self.box_pose.header.frame_id = 'world'
        self.box_pose.pose.position.x = 0.
        self.box_pose.pose.position.y = -0.8
        self.box_pose.pose.position.z = 0.295
        self.box_pose.pose.orientation.x = 0. 
        self.box_pose.pose.orientation.y = 0. 
        self.box_pose.pose.orientation.z = 0. 
        self.box_pose.pose.orientation.w = 1. 


    def get_world_states(self, gz_model_msg):
        #rostopic echo /gazebo/model_states
        #name: ['ground_plane', 'kinect', 'grasping_object', 'unit_box', 'lbr4_allegro']
        self.obj_pos = gz_model_msg.pose[2].position
        self.obj_ort = gz_model_msg.pose[2].orientation
        self.box_pos = gz_model_msg.pose[3].position
        self.box_ort = gz_model_msg.pose[3].orientation
    
    def handle_create_moveit_scene(self, req):        
        scene = PlanningSceneInterface()
        rospy.sleep(1)
        
        if self.use_sim:
            if not self.box_exist:
                #box_pose = PoseStamped()
                #box_pose.header.frame_id = 'world'
                #box_pose.pose.position = self.box_pos
                #box_pose.pose.orientation = self.box_ort
                scene.add_box('box', self.box_pose, (self.box_len_x, self.box_len_y, self.box_len_z))
                self.object_exist = True

            #obj_pose = PoseStamped()
            #obj_pose.header.frame_id = 'world'
            #obj_pose.pose.position = self.obj_pos
            #obj_pose.pose.orientation = self.obj_ort
            scene.add_mesh('grasp_object', req.object_pose, req.object_mesh_path) 
            #print req.object_mesh_path
            #raw_input('Wait Wait')

        else:
            #obj_pose = req.object_pose_in_world
            obj_pose = req.object_pose
            #rospy.loginfo('obj_pose: %s'%str(obj_pose))

            scene.add_box('grasp_object', obj_pose, (req.box_width, req.box_height, req.box_depth))

        rospy.sleep(1)
        
        response = ManageMoveitSceneResponse()
        response.success = True
        return response

    def create_moveit_scene_server(self):
        rospy.Service('create_moveit_scene', ManageMoveitScene, self.handle_create_moveit_scene)
        rospy.loginfo('Service create_moveit_scene:')
        rospy.loginfo('Ready to create the moveit scene.')

    def handle_clean_moveit_scene(self, req):        
        scene = PlanningSceneInterface() 
    
        # clean the scene
        #scene.remove_world_object('box')
        scene.remove_world_object('grasp_object')
        rospy.sleep(1)
         
        response = ManageMoveitSceneResponse()
        response.success = True
        return response
   
    def clean_moveit_scene_server(self):
        rospy.Service('clean_moveit_scene', ManageMoveitScene, self.handle_clean_moveit_scene)
        rospy.loginfo('Service clean_moveit_scene:')
        rospy.loginfo('Ready to clean the moveit scene.')

if __name__=='__main__':
    ms = ManageSceneInMoveit()
    ms.create_moveit_scene_server()
    ms.clean_moveit_scene_server()
    rospy.spin()
