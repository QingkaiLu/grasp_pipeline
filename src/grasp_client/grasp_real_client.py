#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from grasp_pipeline.srv import *
from point_cloud_segmentation.srv import *
from prob_grasp_planner.srv import *
import os
import tf
from sensor_msgs.msg import JointState
from grasp_control.srv import *
import copy
import numpy as np
import time
import roslib.packages as rp
import sys
#sys.path.append(rp.get_pkg_dir('ll4ma_planner') 
#                + '/scripts')
#from palm_planner import PalmPosePlanner
sys.path.append(rp.get_pkg_dir('grasp_pipeline') 
                + '/script')
import plot_traj
import robot_traj_interface
import h5py
import align_object_frame as align_obj
import pickle
sys.path.append(rp.get_pkg_dir('hand_services') 
                + '/scripts')
from hand_client import handClient
import os.path
import mcubes
pkg_path = rp.get_pkg_dir('grasp_pipeline')


class GraspClient:
    def __init__(self):
        rospy.init_node('grasp_client')
        self.use_sim = rospy.get_param('~use_sim', False)
        self.use_hd = rospy.get_param('~use_hd', True)
        self.num_grasps_per_object = rospy.get_param('~num_grasps_per_object', 10)
        self.save_visual_data_pre_path = rospy.get_param('~save_visual_data_pre_path', '')
        self.smooth_plan_traj = rospy.get_param('~smooth_plan_traj', False)
        self.mount_desired_world = None
        self.set_up_place_range()
        self.palm_desired_world = None
        self.object_world_pose = None
        #self.kinect2_sd_frame_id = 'kinect2_ir_optical_frame'
        self.kinect2_hd_frame_id = 'kinect2_rgb_optical_frame'
        self.listener = tf.TransformListener()
        self.kinect2_hd_pcd_topic = '/kinect2/hd/points'
        self.kinect2_qhd_pcd_topic = '/kinect2/qhd/points'
        # self.palm_planner = None
        self.robot_traj_manager = None
        self.grasp_obj_frame_id = 'grasp_object'

        self.data_recording_path = rospy.get_param('~data_recording_path', '')
        #self.grasp_file_name = self.data_recording_path + 'grasp_data.h5'

        hand_joint_states_topic = '/allegro_hand_right/joint_states'
        rospy.Subscriber(hand_joint_states_topic, JointState, self.get_hand_joint_state)

        self.create_obj_grasp_id_file()
        # Number of grasps for a trial of a given object pose.
        self.grasps_num_per_trial = 3

        #Create paths to save initialization heuristic grasp
        self.init_hand_js_path = self.data_recording_path + 'init_grasp/object_hand_js'
        self.init_palm_pose_in_obj_path = self.data_recording_path + 'init_grasp/object_palm_pose_in_obj'
        #self.init_palm_pose_in_cam_path = self.data_recording_path + 'init_grasp/object_palm_pose_in_cam'

        #Offset for robot modelling error.
        #Can't add the offset if it's too close to the object, since 
        #the planner can't find the plan if the inaccurate robot model
        #is too close to the object. This might not be the best way to solve 
        #the collision checking problem. I need to dig this deeper to figure out 
        #a better way.
        self.model_err_x_offset = 0. #0.04

        self.hand_client = handClient()
        self.get_visual_data_response = None

        self.active = rospy.get_param('~active', True)
        self.bag_tactile = False


    def create_obj_grasp_id_file(self):
        self.obj_grasp_id_file_name = self.data_recording_path + 'grasp_data/grasp_obj_id.h5'
        #a: Read/write if exists, create otherwise (default)
        grasp_id_file = h5py.File(self.obj_grasp_id_file_name, 'a')
        cur_object_id_key = 'cur_object_id'
        if cur_object_id_key not in grasp_id_file:
            grasp_id_file.create_dataset(cur_object_id_key, data=-1)
        cur_object_name_key = 'cur_object_name'
        if cur_object_name_key not in grasp_id_file:
            grasp_id_file.create_dataset(cur_object_name_key, data='empty')
        cur_grasp_id_key = 'cur_grasp_id'
        if cur_grasp_id_key not in grasp_id_file:
            grasp_id_file.create_dataset(cur_grasp_id_key, data=-1)
        # cur_grasp_type_key = 'cur_grasp_type'
        # if cur_grasp_type_key not in grasp_id_file:
        #     grasp_id_file.create_dataset(cur_grasp_type_key, data='empty')
        grasp_id_file.close()

    def update_obj_grasp_id_db(self, object_name):
        # Update the object and grasp ID in h5 database
        #'r+': Read/write, file must exist
        grasp_id_file = h5py.File(self.obj_grasp_id_file_name, 'r+')
        grasp_id_file['cur_grasp_id'][()] +=1
        if grasp_id_file['cur_grasp_id'][()] % self.grasps_num_per_trial == 0:
            grasp_id_file['cur_object_id'][()] +=1
            grasp_id_file['cur_grasp_id'][()] = 0
            self.get_cur_object_id()
        grasp_id_file['cur_object_name'][()] = object_name 
        self.get_cur_grasp_id()
        grasp_id_file.close()

    def update_obj_grasp_id(self): 
        # Keep the object id and grasp id consistent with grasp data recording 
        # Update the object id and grasp id before grasp data recording,
        # for 1. tactile data recording and 2. grasp inference. The tactile data
        # and grasp inference files would be over-written by the next grasp if 
        # the grasp is not recorded at data recording stage.
        self.get_cur_object_id()
        self.get_cur_grasp_id()
        cur_object_id = self.cur_object_id
        cur_grasp_id = self.cur_grasp_id + 1
        if cur_grasp_id % self.grasps_num_per_trial == 0:
            cur_object_id +=1
            cur_grasp_id = 0
        return cur_object_id, cur_grasp_id

    def set_object_name(self, object_name):
        self.object_name = object_name

    def get_cur_grasp_id(self):
        grasp_id_file = h5py.File(self.obj_grasp_id_file_name, 'r')
        self.cur_grasp_id = grasp_id_file['cur_grasp_id'][()]
        grasp_id_file.close()

    def get_cur_object_id(self):
        grasp_id_file = h5py.File(self.obj_grasp_id_file_name, 'r')
        self.cur_object_id = grasp_id_file['cur_object_id'][()]
        grasp_id_file.close()

    def get_hand_joint_state(self, hand_js):
        self.true_hand_joint_state = hand_js 

    def set_up_place_range(self):
        #Box table position: x: -0.171 y: 0.596 z: 0.25
        self.table_x = -0.171
        self.table_y = 0.596
        self.table_len_x = .5
        self.table_len_y = .5
        place_x_half_range = (self.table_len_x - 0.1) * .5                
        place_y_half_range = (self.table_len_y - 0.1) * .5
        self.place_x_min = self.table_x - place_x_half_range
        self.place_x_max = self.table_x + place_x_half_range  
        self.place_y_min = self.table_y - place_y_half_range
        self.place_y_max = self.table_y + place_y_half_range  

    def create_moveit_scene_client(self, object_pose):
        rospy.loginfo('Waiting for service create_moveit_scene.')
        rospy.wait_for_service('create_moveit_scene')
        rospy.loginfo('Calling service create_moveit_scene.')
        try:
            create_scene_proxy = rospy.ServiceProxy('create_moveit_scene', ManageMoveitScene)
            create_scene_request = ManageMoveitSceneRequest()
            create_scene_request.create_scene = True
            create_scene_request.obj_seg = self.object_segment_response.obj 
            create_scene_request.object_pose_world = object_pose
            create_scene_request.object_mesh_path = pkg_path + \
                                                    '/obj_meshes/partial_obj.obj' 
            self.create_scene_response = create_scene_proxy(create_scene_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service create_moveit_scene call failed: %s'%e)
        rospy.loginfo('Service create_moveit_scene is executed %s.' %str(self.create_scene_response))

    def segment_object_client(self, align_obj_frame=True):
        rospy.loginfo('Waiting for service object_segmenter.')
        rospy.wait_for_service('object_segmenter')
        rospy.loginfo('Calling service object_segmenter.')
        try:
            object_segment_proxy = rospy.ServiceProxy('object_segmenter', SegmentGraspObject)
            object_segment_request = SegmentGraspObjectRequest()
            self.object_segment_response = object_segment_proxy(object_segment_request) 
            if align_obj_frame:
                self.object_segment_response.obj = \
                        align_obj.align_object(self.object_segment_response.obj, self.listener)
        except rospy.ServiceException, e:
            rospy.loginfo('Service object_segmenter call failed: %s'%e)
        rospy.loginfo('Service object_segmenter is executed.')
        if not self.object_segment_response.object_found:
            rospy.logerr('No object found from segmentation!')
            return False
        return True

    def get_visual_data_client(self):
        rospy.loginfo('Waiting for service get_visual_data.')
        rospy.wait_for_service('get_visual_data')
        rospy.loginfo('Calling service get_visual_data.')
        try:
            get_visual_data_proxy = rospy.ServiceProxy('get_visual_data', GetVisualData)
            get_visual_data_request = GetVisualDataRequest()
            get_visual_data_response = get_visual_data_proxy(get_visual_data_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service get_visual_data call failed: %s'%e)
        rospy.loginfo('Service get_visual_data is executed.')
        return get_visual_data_response

    def gen_grasp_preshape_client(self):
        rospy.loginfo('Waiting for service gen_grasp_preshape.')
        rospy.wait_for_service('gen_grasp_preshape')
        rospy.loginfo('Calling service gen_grasp_preshape.')
        try:
            preshape_proxy = rospy.ServiceProxy('gen_grasp_preshape', GraspPreshape)
            preshape_request = GraspPreshapeRequest()
            if not self.use_sim:
                preshape_request.obj = self.object_segment_response.obj
            else:
                preshape_request.obj = self.object_segment_blensor_response.obj
                preshape_request.sample = self.sampling_grasp

            self.heu_preshape_response = preshape_proxy(preshape_request) 
            #print self.preshape_response
        except rospy.ServiceException, e:
            rospy.loginfo('Service gen_grasp_preshape call failed: %s'%e)
        rospy.loginfo('Service gen_grasp_preshape is executed.')

    def active_plan_preshape_client(self, planner_name):
        rospy.loginfo('Waiting for service grasp_active_plan.')
        rospy.wait_for_service('grasp_active_plan')
        rospy.loginfo('Calling service grasp_active_plan.')
        try:
            grasp_al_proxy = rospy.ServiceProxy('grasp_active_plan', GraspActiveLearn)
            grasp_al_request = GraspActiveLearnRequest()
            grasp_al_request.planner_name = planner_name
            self.grasp_al_response = grasp_al_proxy(grasp_al_request) 
            self.create_partial_mesh(self.grasp_al_response.sparse_voxel_grid, 
                                        self.grasp_al_response.voxel_size,
                                        self.grasp_al_response.voxel_grid_dim)
        except rospy.ServiceException, e:
            rospy.loginfo('Service grasp_active_plan call failed: %s'%e)
        rospy.loginfo('Service grasp_active_plan is executed.')
        return self.grasp_al_response.inf_suc_prob

    def voxel_plan_preshape_client(self, prior_name, grasp_type):
        rospy.loginfo('Waiting for service grasp_voxel_infer.')
        rospy.wait_for_service('grasp_voxel_infer')
        rospy.loginfo('Calling service grasp_voxel_infer.')
        try:
            grasp_voxel_proxy = rospy.ServiceProxy('grasp_voxel_infer', GraspVoxelInfer)
            grasp_voxel_request = GraspVoxelInferRequest()
            grasp_voxel_request.seg_obj = self.object_segment_response.obj
            grasp_voxel_request.prior_name = prior_name
            grasp_voxel_request.grasp_type = grasp_type
            if prior_name == 'Constraint':
                self.gen_grasp_preshape_client()
                heu_num = len(self.heu_preshape_response.allegro_joint_state)
                print 'heu is_top_grasp:', self.heu_preshape_response.is_top_grasp
                if grasp_type == 'unknown':
                    heu_rand_idx = np.random.choice(heu_num, 1)[0]
                    grasp_voxel_request.init_hand_config.hand_joint_state = \
                            self.heu_preshape_response.allegro_joint_state[heu_rand_idx]
                    grasp_voxel_request.init_hand_config.palm_pose = \
                            self.heu_preshape_response.palm_goal_pose_world[heu_rand_idx]
                elif grasp_type == 'overhead':
                    for i in xrange(heu_num):
                        if self.heu_preshape_response.is_top_grasp[i]:
                            grasp_voxel_request.init_hand_config.hand_joint_state = \
                                    self.heu_preshape_response.allegro_joint_state[i]
                            grasp_voxel_request.init_hand_config.palm_pose = \
                                    self.heu_preshape_response.palm_goal_pose_world[i]
                            break
                elif grasp_type == 'side':
                    side_heu_indices = []
                    for i in xrange(heu_num):
                        if not self.heu_preshape_response.is_top_grasp[i]:
                            side_heu_indices.append(i)
                    rand_idx = np.random.choice(heu_num - 1, 1)[0]
                    heu_rand_idx = side_heu_indices[rand_idx]
                    grasp_voxel_request.init_hand_config.hand_joint_state = \
                            self.heu_preshape_response.allegro_joint_state[heu_rand_idx]
                    grasp_voxel_request.init_hand_config.palm_pose = \
                            self.heu_preshape_response.palm_goal_pose_world[heu_rand_idx]
                else:
                    rospy.logerr('Wrong grasp type for voxel constraint inference!')
            self.grasp_voxel_response = grasp_voxel_proxy(grasp_voxel_request) 
            self.create_partial_mesh(self.grasp_voxel_response.sparse_voxel_grid, 
                                        self.grasp_voxel_response.voxel_size,
                                        self.grasp_voxel_response.voxel_grid_dim)
        except rospy.ServiceException, e:
            rospy.loginfo('Service grasp_voxel_infer call failed: %s'%e)
        rospy.loginfo('Service grasp_voxel_infer is executed.')

    def create_partial_mesh(self, sparse_voxel_grid, voxel_size, voxel_grid_dim, 
                            mesh_path=pkg_path + '/obj_meshes/partial_obj.obj'):
        # Mesh these voxels.
        sparse_voxel_grid = np.reshape(sparse_voxel_grid, 
                    [len(sparse_voxel_grid) / 3, 3])
        voxel_grid = np.zeros(tuple(voxel_grid_dim))
        voxel_grid_index = sparse_voxel_grid.astype(int)
        voxel_grid[voxel_grid_index[:, 0], voxel_grid_index[:, 1],
                    voxel_grid_index[:, 2]] = 1
        voxel_grid = np.pad(voxel_grid, ((1,1),(1,1),(1,1)), mode='constant')
        vertices, triangles = mcubes.marching_cubes(voxel_grid, 0)
        vertices = vertices * voxel_size

        # Center mesh.
        vertices[:,0] -= voxel_size * (17.)
        vertices[:,1] -= voxel_size * (17.)
        vertices[:,2] -= voxel_size * (17.)
        
        mcubes.export_obj(vertices, triangles, mesh_path)

    def update_detection_grasp_preshape_client(self, update_preshape):
        rospy.loginfo('Waiting for service update_detection_grasp_preshape.')
        rospy.wait_for_service('update_detection_grasp_preshape')
        rospy.loginfo('Calling service update_detection_grasp_preshape.')
        try:
            update_preshape_proxy = rospy.ServiceProxy('update_detection_grasp_preshape', 
                                                UpdateInfPreshape)
            update_preshape_request = UpdateInfPreshapeRequest()
            update_preshape_request.exp_palm_poses = update_preshape 
            update_preshape_response = update_preshape_proxy(update_preshape_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service update_detection_grasp_preshape call failed: %s'%e)
        rospy.loginfo('Service update_detection_grasp_preshape is executed.')

    def control_allegro_config_client(self, go_home=False, close_hand=False):

        rospy.loginfo('Waiting for service control_allegro_config.')
        rospy.wait_for_service('control_allegro_config')
        rospy.loginfo('Calling service control_allegro_config.')
        try:
            control_proxy = rospy.ServiceProxy('control_allegro_config', AllegroConfig)
            control_request = AllegroConfigRequest()
            if go_home:
                control_request.go_home = True
            elif close_hand:
                control_request.close_hand = True
            else:
                if self.active:
                    control_request.allegro_target_joint_state = \
                            self.grasp_al_response.full_inf_config.hand_joint_state 
                else:
                    control_request.allegro_target_joint_state = \
                            self.grasp_voxel_response.full_inf_config.hand_joint_state 
            self.control_response = control_proxy(control_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service control_allegro_config call failed: %s'%e)
        rospy.loginfo('Service control_allegro_config is executed %s.'%str(self.control_response))


    def listen_mount_pose(self):
        # palm_goal_tf_name = palm_type + '_' + str(grasp_preshape_idx)
        palm_goal_tf_name = 'grasp_palm_pose'
        self.mount_desired_world = None

        self.listener.waitForTransform('palm_link', 'allegro_mount', 
                                        rospy.Time(), rospy.Duration(4.0))
        try:
            self.listener.waitForTransform('palm_link', 'allegro_mount', 
                                            rospy.Time.now(), rospy.Duration(4.0))
            (trans_m_to_p, rot_m_to_p) = self.listener.lookupTransform(
                                            'palm_link', 'allegro_mount', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Could not find transformation for mount_desired_pose!')
            return

        mount_desired_pose = PoseStamped()
        mount_desired_pose.header.frame_id = palm_goal_tf_name 
        mount_desired_pose.pose.position.x = trans_m_to_p[0]
        mount_desired_pose.pose.position.y = trans_m_to_p[1]
        mount_desired_pose.pose.position.z = trans_m_to_p[2]
        mount_desired_pose.pose.orientation.x = rot_m_to_p[0]
        mount_desired_pose.pose.orientation.y = rot_m_to_p[1]
        mount_desired_pose.pose.orientation.z = rot_m_to_p[2]
        mount_desired_pose.pose.orientation.w = rot_m_to_p[3]

        self.listener.waitForTransform('world', mount_desired_pose.header.frame_id, 
                                        rospy.Time(), rospy.Duration(4.0))
        try:
            self.listener.waitForTransform('world', mount_desired_pose.header.frame_id, 
                                            rospy.Time.now(), rospy.Duration(4.0))
            self.mount_desired_world = self.listener.transformPose('world', mount_desired_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Could not find transformation for mount_desired_world!')
            return

        #Make the pose hight above a threshold to avoid collision
        #if self.mount_desired_world.pose.position.z < 0.05: 
        #    rospy.loginfo('Increase height.')
        #    #raw_input('Increase height')
        #    self.mount_desired_world.pose.position.z = 0.05

    def listen_palm_obj_pose(self):
        '''
            Get the palm and object pose for data recording.
        '''
        #if grasp_preshape_idx == len(self.preshape_response.allegro_joint_state) - 1:
        #    self.object_world_seg_pose = None

        #palm_goal_tf_name = palm_type + '_' + str(grasp_preshape_idx)
        palm_goal_tf_name = 'grasp_palm_pose'
        object_tf_name = 'object_pose'

        self.palm_desired_world = None
        self.object_world_seg_pose = None

        # self.listener.waitForTransform('world', palm_goal_tf_name, 
        #                                 rospy.Time(), rospy.Duration(4.0))
        try:
            # self.listener.waitForTransform('world', palm_goal_tf_name, 
            #                                 rospy.Time.now(), rospy.Duration(4.0))
            # (trans_p_to_w, rot_p_to_w) = self.listener.lookupTransform(
            #                                 'world', palm_goal_tf_name, rospy.Time.now())
            self.listener.waitForTransform('world', palm_goal_tf_name, 
                                            rospy.Time(), rospy.Duration(4.0))
            (trans_p_to_w, rot_p_to_w) = self.listener.lookupTransform(
                                            'world', palm_goal_tf_name, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Could not find transformation for palm_desired_world!')
            return

        palm_desired_world = PoseStamped()
        palm_desired_world.header.frame_id = 'world'
        palm_desired_world.pose.position.x = trans_p_to_w[0]
        palm_desired_world.pose.position.y = trans_p_to_w[1]
        palm_desired_world.pose.position.z = trans_p_to_w[2]
        palm_desired_world.pose.orientation.x = rot_p_to_w[0]
        palm_desired_world.pose.orientation.y = rot_p_to_w[1]
        palm_desired_world.pose.orientation.z = rot_p_to_w[2]
        palm_desired_world.pose.orientation.w = rot_p_to_w[3]
        self.palm_desired_world = palm_desired_world

        self.listener.waitForTransform('world', object_tf_name, 
                                        rospy.Time(), rospy.Duration(4.0))
        try:
            # self.listener.waitForTransform('world', object_tf_name, 
            #                                 rospy.Time.now(), rospy.Duration(4.0))
            # (trans_o_to_w, rot_o_to_w) = self.listener.lookupTransform('world', 
            #                                         object_tf_name, rospy.Time.now())
            self.listener.waitForTransform('world', object_tf_name, 
                                            rospy.Time(), rospy.Duration(4.0))
            (trans_o_to_w, rot_o_to_w) = self.listener.lookupTransform('world', 
                                                    object_tf_name, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Could not find transformation for object_world_seg_pose!')
            return

        object_world_seg_pose = PoseStamped()
        object_world_seg_pose.header.frame_id = 'world'
        object_world_seg_pose.pose.position.x = trans_o_to_w[0]
        object_world_seg_pose.pose.position.y = trans_o_to_w[1]
        object_world_seg_pose.pose.position.z = trans_o_to_w[2]
        object_world_seg_pose.pose.orientation.x = rot_o_to_w[0]
        object_world_seg_pose.pose.orientation.y = rot_o_to_w[1]
        object_world_seg_pose.pose.orientation.z = rot_o_to_w[2]
        object_world_seg_pose.pose.orientation.w = rot_o_to_w[3]
        self.object_world_seg_pose = object_world_seg_pose


    def listen_true_palm_pose(self):
        '''
        Listen to get the true palm pose in camera and world frame. 
        This is necesssary because 
        '''
        # self.listener.waitForTransform('world', 'palm_link', 
        #                                 rospy.Time(), rospy.Duration(4.0))
        try:
            # self.listener.waitForTransform('world', 'palm_link', 
            #                                 rospy.Time.now(), rospy.Duration(4.0))
            # (trans_p_to_w, rot_p_to_w) = self.listener.lookupTransform('world', 
            #                                         'palm_link', rospy.Time.now())
            self.listener.waitForTransform('world', 'palm_link', 
                                            rospy.Time(), rospy.Duration(4.0))
            (trans_p_to_w, rot_p_to_w) = self.listener.lookupTransform('world', 
                                                    'palm_link', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Could not find transformation for true_palm_pose_world!')
            return None

        true_palm_pose_world = PoseStamped()
        true_palm_pose_world.header.frame_id = 'world'
        true_palm_pose_world.pose.position.x = trans_p_to_w[0]
        true_palm_pose_world.pose.position.y = trans_p_to_w[1]
        true_palm_pose_world.pose.position.z = trans_p_to_w[2]
        true_palm_pose_world.pose.orientation.x = rot_p_to_w[0]
        true_palm_pose_world.pose.orientation.y = rot_p_to_w[1]
        true_palm_pose_world.pose.orientation.z = rot_p_to_w[2]
        true_palm_pose_world.pose.orientation.w = rot_p_to_w[3]

        return true_palm_pose_world 

    def arm_moveit_planner_client(self, go_home=False, go_zero=False):
        rospy.loginfo('Waiting for service moveit_cartesian_pose_planner.')
        rospy.wait_for_service('moveit_cartesian_pose_planner')
        rospy.loginfo('Calling service moveit_cartesian_pose_planner.')
        try:
            planning_proxy = rospy.ServiceProxy('moveit_cartesian_pose_planner', PalmGoalPoseWorld)
            planning_request = PalmGoalPoseWorldRequest()
            if go_home:
                planning_request.go_home = True
            elif go_zero:
                planning_request.go_zero = True
            else:
                planning_request.palm_goal_pose_world = self.mount_desired_world.pose
            self.planning_response = planning_proxy(planning_request) 
            #print self.planning_response
        except rospy.ServiceException, e:
            rospy.loginfo('Service moveit_cartesian_pose_planner call failed: %s'%e)
        rospy.loginfo('Service moveit_cartesian_pose_planner is executed %s.'%str(self.planning_response.success))
        return self.planning_response.success


    def execute_arm_plan(self, send_cmd_manually=False):
        rospy.loginfo('Executing moveit arm plan...')
        if not self.planning_response.success:
            rospy.loginfo('Does not have a plan to execute!')
            return False
        plan_traj = self.planning_response.plan_traj

        if self.robot_traj_manager is None:
            self.robot_traj_manager = robot_traj_interface.robotTrajInterface(init_node=False)
        
        if self.smooth_plan_traj:
            #Smooth trajectory.
            smooth_success, smooth_traj = \
                    self.robot_traj_manager.get_smooth_traj(plan_traj)
            rospy.loginfo('Trajectory smoothing success: %s.'%str(smooth_success))
            if smooth_success:
                plan_traj = smooth_traj

        if send_cmd_manually:
            # plot_traj.plot_j_acc_profile(plan_traj, 7)

            # send to robot:
            send_cmd = raw_input('send to robot? (y/n)')
            if(send_cmd == 'y'):
                self.robot_traj_manager.send_jtraj(plan_traj)
                #raw_input('Hit any key to keep going after the robot control is done!')
                #TO DO: make sure the robot finishing executing the trajectory before returning.
                return True
            return False
        else:
            # send to robot:
            self.robot_traj_manager.send_jtraj(plan_traj)
            return True


    def hand_grasp_control(self, grasp_type='prec'):
        if grasp_type == 'prec':
            joint_idx = [1, 2, 5, 6, 9, 10, 14, 15]
        elif grasp_type == 'power':
            joint_idx = [1, 2, 3, 5, 6, 7, 9, 10, 11, 14, 15]
        else:
            rospy.logerr('Wrong grasp type for grasp controller!')
            return
        self.hand_client.grasp_object(joint_idx)
        cur_js = self.hand_client.get_joint_state() 
        increase_stiffness_times = 3
        for i in xrange(increase_stiffness_times):
            rospy.sleep(0.2)
            _, cur_js = self.hand_client.increase_stiffness(joint_idx, cur_js)
            
    def clean_moveit_scene_client(self):
        rospy.loginfo('Waiting for service clean_moveit_scene.')
        rospy.wait_for_service('clean_moveit_scene')
        rospy.loginfo('Calling service clean_moveit_scene.')
        try:
            clean_scene_proxy = rospy.ServiceProxy('clean_moveit_scene', ManageMoveitScene)
            clean_scene_request = ManageMoveitSceneRequest()
            clean_scene_request.clean_scene = True
            self.clean_scene_response = clean_scene_proxy(clean_scene_request) 
            print self.clean_scene_response
        except rospy.ServiceException, e:
            rospy.loginfo('Service clean_moveit_scene call failed: %s'%e)
        rospy.loginfo('Service clean_moveit_scene is executed %s.' %str(self.clean_scene_response))

    def lift_moveit_planner_client(self, height_to_lift=0.15, pos_noise=False):
        rospy.loginfo('Waiting for service moveit_cartesian_pose_planner to lift.')
        rospy.wait_for_service('moveit_cartesian_pose_planner')
        rospy.loginfo('Calling service moveit_cartesian_pose_planner to lift.')
        try:
            planning_proxy = rospy.ServiceProxy('moveit_cartesian_pose_planner', PalmGoalPoseWorld)
            planning_request = PalmGoalPoseWorldRequest()
            planning_request.palm_goal_pose_world = copy.deepcopy(self.mount_desired_world.pose)
            planning_request.palm_goal_pose_world.position.z += height_to_lift
            if pos_noise:
                rand_x = np.random.uniform(-0.1, 0.1)
                rand_y = np.random.uniform(-0.1, 0.1)
                planning_request.palm_goal_pose_world.position.x += rand_x
                planning_request.palm_goal_pose_world.position.y += rand_y

            self.planning_response = planning_proxy(planning_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service moveit_cartesian_pose_planner call to lift failed: %s'%e)
        rospy.loginfo('Service moveit_cartesian_pose_planner to lift is executed %s.'%str(self.planning_response.success))
        return self.planning_response.success

    def lift_moveit_noise(self, lift_noise_times=3):
        if self.lift_moveit_planner_client():
            return True
        for i in xrange(lift_noise_times):
            raw_input('Start to lift with noise')
            if self.lift_moveit_planner_client(pos_noise=True):
                raw_input('Lift with noise?')
                return True
        return False

    def lift_task_vel_planner_client(self, height_to_lift=0.15):
        rospy.loginfo('Waiting for service straight_line_planner to lift.')
        rospy.wait_for_service('straight_line_planner')
        rospy.loginfo('Calling service straight_line_planner to lift.')
        try:
            planning_proxy = rospy.ServiceProxy('straight_line_planner', StraightLinePlan)
            planning_request = StraightLinePlanRequest()
            planning_request.lift_height = height_to_lift
            self.planning_response = planning_proxy(planning_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service straight_line_planner call to lift failed: %s'%e)
        rospy.loginfo('Service straight_line_planner to lift is executed %s.'
                      %str(self.planning_response.success))
        return self.planning_response.success


    def move_arm_home(self):
        rospy.loginfo('Move the arm to go home.')
        self.arm_moveit_planner_client(go_home=True)
        self.execute_arm_plan()


    def move_arm_zero(self):
        rospy.loginfo('Move the arm to go zero.')
        self.arm_moveit_planner_client(go_zero=True)
        self.execute_arm_plan()


    def record_grasp_visual_data_client(self, save_visual_data_request):
        rospy.loginfo('Waiting for service save_visual_data.')
        rospy.wait_for_service('save_visual_data')
        rospy.loginfo('Calling service save_visual_data.')
        try:
            save_visual_data_proxy = rospy.ServiceProxy('save_visual_data', SaveVisualData)
            self.save_visual_data_response = save_visual_data_proxy(save_visual_data_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service save_visual_data call failed: %s'%e)
        rospy.loginfo('Service save_visual_data is executed %s.'
                       %self.save_visual_data_response.save_visual_data_success)


    def bag_tactile_visual_data(self, grasp_phase, operation):
        cur_object_id, cur_grasp_id = self.update_obj_grasp_id()
        if self.bag_tactile:
            self.bag_grasp_tactile_data_client(grasp_phase, operation,
                                               cur_object_id, cur_grasp_id)
        self.bag_grasp_visual_data_client(grasp_phase, operation,
                                           cur_object_id, cur_grasp_id)

    def bag_grasp_tactile_data_client(self, grasp_phase, operation, 
                                      cur_object_id, cur_grasp_id):
        rospy.loginfo('Waiting for service bag_tactile_data.')
        rospy.wait_for_service('bag_tactile_data')
        rospy.loginfo('Calling service bag_tactile_data.')
        try:
            bag_tactile_data_request = GraspDataBaggingRequest()
            
            bag_tactile_data_request.object_id = cur_object_id
            bag_tactile_data_request.object_name = self.object_name
            bag_tactile_data_request.grasp_id = cur_grasp_id
            # bag_tactile_data_request.grasp_type = 'unknown'
            # bag_tactile_data_request.grasp_control_type = 'prec'

            bag_tactile_data_request.grasp_phase = grasp_phase
            bag_tactile_data_request.operation = operation

            bag_tactile_data_proxy = rospy.ServiceProxy('bag_tactile_data', GraspDataBagging)
            self.bag_tactile_data_response = bag_tactile_data_proxy(bag_tactile_data_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service bag_tactile_data call failed: %s'%e)
        rospy.loginfo('Service bag_tactile_data is executed %s.'
                       %self.bag_tactile_data_response.bag_data_success)

    def bag_grasp_visual_data_client(self, grasp_phase, operation,
                                     cur_object_id, cur_grasp_id):
        rospy.loginfo('Waiting for service bag_visual_data.')
        rospy.wait_for_service('bag_visual_data')
        rospy.loginfo('Calling service bag_visual_data.')
        try:
            bag_visual_data_request = GraspDataBaggingRequest()
            
            bag_visual_data_request.object_id = cur_object_id
            bag_visual_data_request.object_name = self.object_name
            bag_visual_data_request.grasp_id = cur_grasp_id
            # bag_visual_data_request.grasp_type = 'unknown'
            # bag_visual_data_request.grasp_control_type = 'prec'

            bag_visual_data_request.grasp_phase = grasp_phase
            bag_visual_data_request.operation = operation

            bag_visual_data_proxy = rospy.ServiceProxy('bag_visual_data', GraspDataBagging)
            self.bag_visual_data_response = bag_visual_data_proxy(bag_visual_data_request) 
        except rospy.ServiceException, e:
            rospy.loginfo('Service bag_visual_data call failed: %s'%e)
        rospy.loginfo('Service bag_visual_data is executed %s.'
                       %self.bag_visual_data_response.bag_data_success)

    def record_voxel_data_client(self, prior_name):
        rospy.loginfo('Waiting for service record_grasp_data.')
        rospy.wait_for_service('record_grasp_data')
        rospy.loginfo('Calling service record_grasp_data.')
        self.save_grasp_visual_data()
        try:
            record_grasp_data_proxy = rospy.ServiceProxy('record_grasp_data', RealGraspData)
            record_grasp_data_request = RealGraspDataRequest()
            grasp_label = None
            while grasp_label is None:
                grasp_label_str = raw_input('### Is this grasp successful? Please input y (success), n (failure),' + \
                        ' p (no plan). \n')
                #print 'grasp_label_str:', grasp_label_str
                if grasp_label_str == 'y':
                    grasp_label = 1
                elif grasp_label_str == 'n':
                    grasp_label = 0
                elif grasp_label_str == 'p':
                    grasp_label = -1
                else:
                    rospy.logerr('Wrong grasp label input!')
            record_grasp_data_request.grasp_success_label = grasp_label 

            #Update the grasp id and object id if the current grasp is not ignored.
            self.update_obj_grasp_id_db(self.object_name)

            record_grasp_data_request.object_id = self.cur_object_id
            record_grasp_data_request.object_name = self.object_name
            record_grasp_data_request.grasp_id = self.cur_grasp_id
            record_grasp_data_request.time_stamp = time.time()

            record_grasp_data_request.object_world_seg_pose = self.object_world_seg_pose
            record_grasp_data_request.preshape_palm_world_pose = \
                    self.grasp_voxel_response.full_inf_config.palm_pose
            record_grasp_data_request.preshape_allegro_joint_state = \
                    self.grasp_voxel_response.full_inf_config.hand_joint_state

            if grasp_label_str != 'p':
                record_grasp_data_request.true_preshape_joint_state = self.true_preshape_hand_js
                record_grasp_data_request.true_preshape_palm_world_pose = self.true_palm_pose_world
                record_grasp_data_request.close_shape_allegro_joint_state = self.close_hand_js  
                record_grasp_data_request.close_shape_palm_world_pose = self.close_palm_pose_world 
                record_grasp_data_request.lift_shape_allegro_joint_state = self.lift_hand_js  
                record_grasp_data_request.lift_shape_palm_world_pose = self.lift_palm_pose_world 
            # record_grasp_data_request.top_grasp = self.preshape_response.is_top_grasp[grasp_preshape_idx] 

            record_grasp_data_request.sparse_voxel_grid = self.grasp_voxel_response.sparse_voxel_grid
            record_grasp_data_request.object_size = self.grasp_voxel_response.object_size

            record_grasp_data_request.init_config_array = self.grasp_voxel_response.init_config_array
            record_grasp_data_request.inf_config_array = self.grasp_voxel_response.inf_config_array

            record_grasp_data_request.init_val = self.grasp_voxel_response.init_val
            record_grasp_data_request.init_suc_prob = self.grasp_voxel_response.init_suc_prob
            record_grasp_data_request.init_log_prior = self.grasp_voxel_response.init_log_prior

            record_grasp_data_request.inf_val = self.grasp_voxel_response.inf_val
            record_grasp_data_request.inf_suc_prob = self.grasp_voxel_response.inf_suc_prob
            record_grasp_data_request.inf_log_prior = self.grasp_voxel_response.inf_log_prior

            record_grasp_data_request.prior_name = prior_name 

            self.record_grasp_data_response = record_grasp_data_proxy(record_grasp_data_request) 
            rospy.loginfo('****' + str(self.record_grasp_data_response))
        except rospy.ServiceException, e:
            rospy.loginfo('Service record_grasp_data call failed: %s'%e)
        rospy.loginfo('Service record_grasp_data is executed %s.'%self.record_grasp_data_response.save_h5_success)

        # self.record_grasp_data_request = record_grasp_data_request
        if grasp_label_str != 'p':
            self.save_lift_visual_data()
        

    def record_active_data_client(self):
        '''
            Record the active learning grasp data.
        '''
        # TODO: get the voxel grid and object sizes from the active learner 
        # and save these.
        rospy.loginfo('Waiting for service record_grasp_data.')
        rospy.wait_for_service('record_grasp_data')
        rospy.loginfo('Calling service record_grasp_data.')
        self.save_grasp_visual_data()
        try:
            record_grasp_data_proxy = rospy.ServiceProxy('record_grasp_data', RealGraspData)
            record_grasp_data_request = RealGraspDataRequest()

            grasp_label = None
            while grasp_label is None:
                grasp_label_str = raw_input('### Is this grasp successful? Please input y (success), n (failure),' + \
                        ' p (no plan). \n')
                #print 'grasp_label_str:', grasp_label_str
                if grasp_label_str == 'y':
                    grasp_label = 1
                elif grasp_label_str == 'n':
                    grasp_label = 0
                elif grasp_label_str == 'p':
                    grasp_label = -1
                else:
                    rospy.logerr('Wrong grasp label input!')
            record_grasp_data_request.grasp_success_label = grasp_label 

            #Update the grasp id and object id if the current grasp is not ignored.
            self.update_obj_grasp_id_db(self.object_name)

            record_grasp_data_request.object_id = self.cur_object_id
            record_grasp_data_request.object_name = self.object_name
            record_grasp_data_request.grasp_id = self.cur_grasp_id
            record_grasp_data_request.time_stamp = time.time()

            #TODO: save the preshape and final grasp images.

            record_grasp_data_request.object_world_seg_pose = self.object_world_seg_pose
            # record_grasp_data_request.preshape_palm_world_pose = \
            #         self.preshape_response.palm_goal_pose_world[grasp_preshape_idx]
            # record_grasp_data_request.preshape_allegro_joint_state = \
            #         self.preshape_response.allegro_joint_state[grasp_preshape_idx] 
            record_grasp_data_request.preshape_palm_world_pose = \
                    self.grasp_al_response.full_inf_config.palm_pose
            record_grasp_data_request.preshape_allegro_joint_state = \
                    self.grasp_al_response.full_inf_config.hand_joint_state

            record_grasp_data_request.true_preshape_joint_state = self.true_preshape_hand_js
            record_grasp_data_request.true_preshape_palm_world_pose = self.true_palm_pose_world
            record_grasp_data_request.close_shape_allegro_joint_state = self.close_hand_js  
            record_grasp_data_request.close_shape_palm_world_pose = self.close_palm_pose_world 
            record_grasp_data_request.lift_shape_allegro_joint_state = self.lift_hand_js  
            record_grasp_data_request.lift_shape_palm_world_pose = self.lift_palm_pose_world 
            # record_grasp_data_request.top_grasp = self.preshape_response.is_top_grasp[grasp_preshape_idx] 

            record_grasp_data_request.sparse_voxel_grid = self.grasp_al_response.sparse_voxel_grid
            record_grasp_data_request.object_size = self.grasp_al_response.object_size

            record_grasp_data_request.init_config_array = self.grasp_al_response.init_config_array
            record_grasp_data_request.inf_config_array = self.grasp_al_response.inf_config_array

            record_grasp_data_request.init_ik_config_array = self.grasp_al_response.init_ik_config_array
            record_grasp_data_request.inf_ik_config_array = self.grasp_al_response.inf_ik_config_array

            record_grasp_data_request.init_val = self.grasp_al_response.init_val
            record_grasp_data_request.init_suc_prob = self.grasp_al_response.init_suc_prob
            record_grasp_data_request.init_log_prior = self.grasp_al_response.init_log_prior
            record_grasp_data_request.init_uct = self.grasp_al_response.init_uct

            record_grasp_data_request.inf_val = self.grasp_al_response.inf_val
            record_grasp_data_request.inf_suc_prob = self.grasp_al_response.inf_suc_prob
            record_grasp_data_request.inf_log_prior = self.grasp_al_response.inf_log_prior
            record_grasp_data_request.inf_uct = self.grasp_al_response.inf_uct
            record_grasp_data_request.reward = self.grasp_al_response.reward
            record_grasp_data_request.action = self.grasp_al_response.action

            self.record_grasp_data_response = record_grasp_data_proxy(record_grasp_data_request) 
            rospy.loginfo('****' + str(self.record_grasp_data_response))
        except rospy.ServiceException, e:
            rospy.loginfo('Service record_grasp_data call failed: %s'%e)
        rospy.loginfo('Service record_grasp_data is executed %s.'%self.record_grasp_data_response.save_h5_success)

        self.record_grasp_data_request = record_grasp_data_request
        self.save_lift_visual_data()
        

    def record_active_np_data_client(self):
        '''
            Record the active learning grasp data without motion plans.
        '''
        rospy.loginfo('Waiting for service record_grasp_data.')
        rospy.wait_for_service('record_grasp_data')
        rospy.loginfo('Calling service record_grasp_data.')
        self.save_grasp_visual_data()
        try:
            record_grasp_data_proxy = rospy.ServiceProxy('record_grasp_data', RealGraspData)
            record_grasp_data_request = RealGraspDataRequest()

            grasp_label = -1
            record_grasp_data_request.grasp_success_label = grasp_label 

            #Update the grasp id and object id if the current grasp is not ignored.
            self.update_obj_grasp_id_db(self.object_name)

            record_grasp_data_request.object_id = self.cur_object_id
            record_grasp_data_request.object_name = self.object_name
            record_grasp_data_request.grasp_id = self.cur_grasp_id
            record_grasp_data_request.time_stamp = time.time()

            record_grasp_data_request.object_world_seg_pose = self.object_world_seg_pose
            record_grasp_data_request.preshape_palm_world_pose = \
                    self.grasp_al_response.full_inf_config.palm_pose
            record_grasp_data_request.preshape_allegro_joint_state = \
                    self.grasp_al_response.full_inf_config.hand_joint_state

            record_grasp_data_request.sparse_voxel_grid = self.grasp_al_response.sparse_voxel_grid
            record_grasp_data_request.object_size = self.grasp_al_response.object_size

            record_grasp_data_request.init_config_array = self.grasp_al_response.init_config_array
            record_grasp_data_request.inf_config_array = self.grasp_al_response.inf_config_array

            record_grasp_data_request.init_ik_config_array = self.grasp_al_response.init_ik_config_array
            record_grasp_data_request.inf_ik_config_array = self.grasp_al_response.inf_ik_config_array

            record_grasp_data_request.init_val = self.grasp_al_response.init_val
            record_grasp_data_request.init_suc_prob = self.grasp_al_response.init_suc_prob
            record_grasp_data_request.init_log_prior = self.grasp_al_response.init_log_prior
            record_grasp_data_request.init_uct = self.grasp_al_response.init_uct

            record_grasp_data_request.inf_val = self.grasp_al_response.inf_val
            record_grasp_data_request.inf_suc_prob = self.grasp_al_response.inf_suc_prob
            record_grasp_data_request.inf_log_prior = self.grasp_al_response.inf_log_prior
            record_grasp_data_request.inf_uct = self.grasp_al_response.inf_uct
            record_grasp_data_request.reward = self.grasp_al_response.reward
            record_grasp_data_request.action = self.grasp_al_response.action

            self.record_grasp_data_response = record_grasp_data_proxy(record_grasp_data_request) 
            rospy.loginfo('****' + str(self.record_grasp_data_response))
        except rospy.ServiceException, e:
            rospy.loginfo('Service record_grasp_data call failed: %s'%e)
        rospy.loginfo('Service record_grasp_data is executed %s.'%self.record_grasp_data_response.save_h5_success)

        self.record_grasp_data_request = record_grasp_data_request


    def save_grasp_visual_data(self):
        if self.grasp_visual_data_response is not None:
            cur_object_id, cur_grasp_id = self.update_obj_grasp_id()
            self.save_grasp_visual_data_request = SaveVisualDataRequest()
            # save_visual_data_request.real_kinect2 = not self.use_sim
            self.save_grasp_visual_data_request.scene_cloud = self.grasp_visual_data_response.scene_cloud 

            self.save_grasp_visual_data_request.scene_depth_img = self.grasp_visual_data_response.scene_depth_img
            self.save_grasp_visual_data_request.scene_rgb_img = self.grasp_visual_data_response.scene_rgb_img

            self.save_grasp_visual_data_request.scene_sd_depth_img = self.grasp_visual_data_response.scene_sd_depth_img
            self.save_grasp_visual_data_request.scene_sd_rgb_img = self.grasp_visual_data_response.scene_sd_rgb_img

            self.save_grasp_visual_data_request.scene_sd_cloud = self.grasp_visual_data_response.scene_sd_cloud 
            self.save_grasp_visual_data_request.scene_cloud_save_path = self.save_visual_data_pre_path + \
                    'pcd/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(cur_grasp_id) + '.pcd' 

            self.save_grasp_visual_data_request.rgb_image_save_path = self.save_visual_data_pre_path + \
                    'rgb_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(cur_grasp_id) + '.png' 
            self.save_grasp_visual_data_request.depth_image_save_path = self.save_visual_data_pre_path + \
                    'depth_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(cur_grasp_id) + '.png' 
            self.save_grasp_visual_data_request.sd_rgb_image_save_path = self.save_visual_data_pre_path + \
                    'sd_rgb_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(cur_grasp_id) + '.png' 
            self.save_grasp_visual_data_request.sd_depth_image_save_path = self.save_visual_data_pre_path + \
                    'sd_depth_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(cur_grasp_id) + '.png' 

            self.save_grasp_visual_data_request.scene_sd_cloud_save_path = self.save_visual_data_pre_path + \
                    'sd_pcd/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(cur_grasp_id) + '.pcd' 
            self.save_grasp_visual_data_request.sd_cloud_depth_image_save_path = self.save_visual_data_pre_path + \
                    'sd_cloud_depth_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(cur_grasp_id) + '.png' 
            self.save_grasp_visual_data_request.sd_cloud_rgb_image_save_path = self.save_visual_data_pre_path + \
                    'sd_cloud_rgb_image/' + 'object_' + str(cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(cur_grasp_id) + '.png' 

            self.record_grasp_visual_data_client(self.save_grasp_visual_data_request)
        else:
            self.grasp_visual_data_response = None


    def save_lift_visual_data(self):
        if self.lift_visual_data_response is not None:
            save_visual_data_request = SaveVisualDataRequest()
            save_visual_data_request.real_kinect2 = not self.use_sim
            save_visual_data_request.scene_cloud = self.lift_visual_data_response.scene_cloud 

            save_visual_data_request.scene_depth_img = self.lift_visual_data_response.scene_depth_img
            save_visual_data_request.scene_rgb_img = self.lift_visual_data_response.scene_rgb_img

            save_visual_data_request.scene_sd_depth_img = self.lift_visual_data_response.scene_sd_depth_img
            save_visual_data_request.scene_sd_rgb_img = self.lift_visual_data_response.scene_sd_rgb_img

            save_visual_data_request.scene_sd_cloud = self.lift_visual_data_response.scene_sd_cloud 
            save_visual_data_request.scene_cloud_save_path = self.save_visual_data_pre_path + \
                    'pcd/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.pcd' 

            save_visual_data_request.rgb_image_save_path = self.save_visual_data_pre_path + \
                    'rgb_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.png' 
            save_visual_data_request.depth_image_save_path = self.save_visual_data_pre_path + \
                    'depth_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.png' 
            save_visual_data_request.sd_rgb_image_save_path = self.save_visual_data_pre_path + \
                    'sd_rgb_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.png' 
            save_visual_data_request.sd_depth_image_save_path = self.save_visual_data_pre_path + \
                    'sd_depth_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.png' 

            save_visual_data_request.scene_sd_cloud_save_path = self.save_visual_data_pre_path + \
                    'sd_pcd/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.pcd' 
            save_visual_data_request.sd_cloud_depth_image_save_path = self.save_visual_data_pre_path + \
                    'sd_cloud_depth_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.png' 
            save_visual_data_request.sd_cloud_rgb_image_save_path = self.save_visual_data_pre_path + \
                    'sd_cloud_rgb_image/' + 'object_' + str(self.cur_object_id) + '_' + str(self.object_name) + \
                    '_grasp_' + str(self.cur_grasp_id) + '_lift.png' 

            self.record_grasp_visual_data_client(save_visual_data_request)
        else:
            self.lift_visual_data_response = None

    def segment_and_get_vis_data(self):
        object_found = self.segment_object_client()
        if not object_found:
            return False
        self.grasp_visual_data_response = self.get_visual_data_client()
        return True

    def grasp_and_lift_object_steps(self):
        #create moveit scene -> move hand to preshape (To do) -> move arm to preshape
        #close hand to grasp -> clean object from moveit -> lift object 

        #Listen mount poses of experiment grasp preshapes to 
        #find arm plans.
        self.listen_mount_pose() 
        #Listen experiment grasp preshape palm poses
        #in object and world frame for data recording.
        self.listen_palm_obj_pose()

        self.create_moveit_scene_client(self.object_world_seg_pose)

        self.control_allegro_config_client()

        moveit_found_plan = self.arm_moveit_planner_client()
        if not moveit_found_plan:
            return False
        if not self.execute_arm_plan(send_cmd_manually=True):
            return False

        self.true_palm_pose_world = self.listen_true_palm_pose()
        self.true_preshape_hand_js = self.true_hand_joint_state

        self.bag_tactile_visual_data(grasp_phase='preshape', operation='start')
        # rospy.sleep(1.)
        rospy.sleep(0.5)
        self.bag_tactile_visual_data(grasp_phase='preshape', operation='stop')
        # raw_input('Grasp?')
        self.bag_tactile_visual_data(grasp_phase='grasp', operation='start')
        self.hand_grasp_control()
        # rospy.sleep(1.)
        rospy.sleep(0.5)
        self.bag_tactile_visual_data(grasp_phase='grasp', operation='stop')

        self.close_palm_pose_world = self.listen_true_palm_pose()
        self.close_hand_js = self.true_hand_joint_state 

        #Remove the object from the hand for collision checking.
        # self.clean_moveit_scene_client()
        #lift = raw_input('Lift or not (y/n)?')
        lift = 'y'
        if lift == 'y':
            task_vel_lift_succes = self.lift_task_vel_planner_client()
            if not task_vel_lift_succes:
                rospy.loginfo('Task velocity straight line planner fails to find a valid plan' \
                               ' for lifting. Switch to the moveit planner.')
                # self.lift_moveit_planner_client()
                self.lift_moveit_noise()
            # self.execute_arm_plan(send_cmd_manually=True)
            self.execute_arm_plan(send_cmd_manually=False)
            self.bag_tactile_visual_data(grasp_phase='lift', operation='start')
            rospy.loginfo('Wait for a few seconds before getting the lifting visual data.')
            rospy.sleep(1.)
            self.bag_tactile_visual_data(grasp_phase='lift', operation='stop')
            self.lift_palm_pose_world = self.listen_true_palm_pose()
            self.lift_hand_js = self.true_hand_joint_state 
            self.lift_visual_data_response = self.get_visual_data_client()
        return True 

    def place_object_steps(self):
        #self.place_arm_movement_client()
        self.place_control_allegro_client()
        if self.use_sim:
            self.create_moveit_scene_client()
        self.move_arm_home_client()


