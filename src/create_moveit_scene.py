#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

#To do: change this file to a ros server to load different objects.
class CreateMoveitScene:
    def __init__(self, gazebo_model_state_topic='/gazebo/model_states'):
        roscpp_initialize(sys.argv)
        rospy.init_node('moveit_scene')
        rospy.Subscriber(gazebo_model_state_topic, ModelStates, self.get_world_states)

    def get_world_states(self, gz_model_msg):
        #rostopic echo /gazebo/model_states
        #name: ['ground_plane', 'kinect', 'grasping_object', 'unit_box', 'lbr4_allegro']
        self.obj_pos = gz_model_msg.pose[2].position
        self.obj_ort = gz_model_msg.pose[2].orientation
        self.box_pos = gz_model_msg.pose[3].position
        self.box_ort = gz_model_msg.pose[3].orientation
    
    def create_scene(self):        
        scene = PlanningSceneInterface()
        rospy.sleep(1)
    
        # clean the scene
        scene.remove_world_object('box')
        scene.remove_world_object('object')
        
        #return
        obj_pose = PoseStamped()
        #p.header.frame_id = robot.get_planning_frame()
        obj_pose.header.frame_id = 'world'
        obj_pose.pose.position = self.obj_pos
        obj_pose.pose.orientation = self.obj_ort
        scene.add_mesh('grasping_object', obj_pose, 
                '/home/qingkai/Workspace/urlg_robot_ws/src/urlg_robots_gazebo/worlds/objects/pringle/optimized_poisson_texture_mapped_mesh.dae')
       
        box_pose = PoseStamped()
        #p.header.frame_id = robot.get_planning_frame()
        box_pose.header.frame_id = 'world'
        box_pose.pose.position = self.box_pos
        box_pose.pose.orientation = self.box_ort
        scene.add_box('box', box_pose, (0.5, 0.5, 0.5))
        rospy.sleep(1)



if __name__=='__main__':
    cs = CreateMoveitScene()
    cs.create_scene()
    rospy.spin()
