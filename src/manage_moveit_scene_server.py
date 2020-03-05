#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from grasp_pipeline.srv import *
import roslib.packages as rp
pkg_path = rp.get_pkg_dir('grasp_pipeline')


#To do: change this file to a ros server to load different objects.
class ManageSceneInMoveit:
    def __init__(self):
        rospy.init_node('manage_moveit_scene_node')
        self.use_sim = rospy.get_param('~use_sim', False)
        self.set_table_box_properties()
        self.table_box_exist = False


    def set_table_box_properties(self):
        if self.use_sim:
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
        else:
            self.box_pose = PoseStamped()
            self.box_pose.header.frame_id = 'world'
            # Change table collision box model position for the new robot arm pose
            self.box_pose.pose.position.x = 0.
            self.box_pose.pose.position.y = -0.88
            self.box_pose.pose.position.z = 0.59 + 0.015
            self.box_pose.pose.orientation.x = 0. 
            self.box_pose.pose.orientation.y = 0. 
            self.box_pose.pose.orientation.z = 0. 
            self.box_pose.pose.orientation.w = 1. 
            self.box_len_x = 1.4
            self.box_len_y = 1.4
            self.box_len_z = 0.015

            # self.wall_pose = PoseStamped()
            # self.wall_pose.header.frame_id = 'world'
            # # Change table collision wall model position for the new robot arm pose
            # self.wall_pose.pose.position.x = 0.
            # self.wall_pose.pose.position.y = -1.
            # self.wall_pose.pose.position.z = 0.
            # self.wall_pose.pose.orientation.x = 0. 
            # self.wall_pose.pose.orientation.y = 0. 
            # self.wall_pose.pose.orientation.z = 0. 
            # self.wall_pose.pose.orientation.w = 1. 
            # self.wall_len_x = 5.
            # self.wall_len_y = 0.01
            # self.wall_len_z = 5.

    
    def handle_create_moveit_scene(self, req):        
        scene = PlanningSceneInterface()
        rospy.sleep(0.5)
        
        #Add a box for the table.
        if not self.table_box_exist:
            scene.add_box('table_box', self.box_pose, 
                            (self.box_len_x, self.box_len_y, self.box_len_z))
            self.table_box_exist = True
        
        print req.object_mesh_path
        scene.add_mesh('obj_mesh', req.object_pose_world, req.object_mesh_path) 

        # if self.use_sim:
        #     scene.add_mesh('obj_mesh', req.object_pose_world, req.object_mesh_path) 
        # else:
        #     # scene.add_box('wall', self.wall_pose, 
        #     # (self.wall_len_x, self.wall_len_y, self.wall_len_z))

        # scene.add_box('obj_box', req.object_pose_world, 
        #               (req.obj_seg.width, req.obj_seg.height, req.obj_seg.depth))
        # scene.add_box('obj_box', req.object_pose_world, 
        #               (0.02, 0.02, req.obj_seg.depth))

        rospy.sleep(0.5)
        
        response = ManageMoveitSceneResponse()
        response.success = True
        return response


    def create_moveit_scene_server(self):
        rospy.Service('create_moveit_scene', ManageMoveitScene, self.handle_create_moveit_scene)
        rospy.loginfo('Service create_scene:')
        rospy.loginfo('Ready to create the moveit_scene.')


    def handle_clean_moveit_scene(self, req):        
        scene = PlanningSceneInterface()
        rospy.sleep(1)
    
        # clean the scene
        #scene.remove_world_object('table_box')
        scene.remove_world_object('obj_mesh')
         
        response = ManageMoveitSceneResponse()
        response.success = True
        return response
   

    def clean_moveit_scene_server(self):
        rospy.Service('clean_moveit_scene', ManageMoveitScene, self.handle_clean_moveit_scene)
        rospy.loginfo('Service clean_scene:')
        rospy.loginfo('Ready to clean the moveit_scene.')


if __name__=='__main__':
    ms = ManageSceneInMoveit()
    ms.create_moveit_scene_server()
    ms.clean_moveit_scene_server()
    rospy.spin()
