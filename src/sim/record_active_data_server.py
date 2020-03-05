#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
from grasp_pipeline.srv import *
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState, CameraInfo
import tf
import numpy as np
import h5py


class RecordActiveData():
    def __init__(self):
        rospy.init_node('record_grasp_data_server')
        self.num_grasps_per_object = rospy.get_param('~num_grasps_per_object', 10)
        self.data_recording_path = rospy.get_param('~data_recording_path', '/mnt/tars_data/multi_finger_grasp_data/')
        self.grasp_file_name = self.data_recording_path + 'grasp_data.h5'
        self.suc_grasp_file_name = self.data_recording_path + 'suc_grasp_data.h5'
        self.failure_grasp_file_name = self.data_recording_path + 'failure_grasp_data.h5'
        self.initialize_active_files()


    def initialize_active_files(self):
        self.initialize_data_file(self.grasp_file_name)
        self.initialize_data_file(self.suc_grasp_file_name)
        self.initialize_data_file(self.failure_grasp_file_name)
        

    def initialize_data_file(self, file_name):
        #a: Read/write if exists, create otherwise (default)
        grasp_file = h5py.File(file_name, 'a')
        hand_joint_state_name = ['index_joint_0','index_joint_1','index_joint_2', 'index_joint_3',
                   'middle_joint_0','middle_joint_1','middle_joint_2', 'middle_joint_3',
                   'ring_joint_0','ring_joint_1','ring_joint_2', 'ring_joint_3',
                   'thumb_joint_0','thumb_joint_1','thumb_joint_2', 'thumb_joint_3']
        hand_js_name_key = 'hand_joint_state_name' 
        if hand_js_name_key not in grasp_file:
            grasp_file.create_dataset(hand_js_name_key, data=hand_joint_state_name)

        max_object_id_key = 'max_object_id'
        if max_object_id_key not in grasp_file:
            grasp_file.create_dataset(max_object_id_key, data=-1)
        cur_object_name_key = 'cur_object_name'
        if cur_object_name_key not in grasp_file:
            grasp_file.create_dataset(cur_object_name_key, data='empty')

        total_grasps_num_key = 'total_grasps_num'
        if total_grasps_num_key not in grasp_file:
            grasp_file.create_dataset(total_grasps_num_key, data=0)
        suc_grasps_num_key = 'suc_grasps_num'
        if suc_grasps_num_key not in grasp_file:
            grasp_file.create_dataset(suc_grasps_num_key, data=0)

        grasp_file.close()
    

    def handle_record_grasp_data(self, req):
        self.record_grasp_data(req)
        grasp_file = h5py.File(self.grasp_file_name, 'r+')
        object_id = grasp_file['max_object_id'][()]
        grasp_file.close()
        if req.grasp_success_label == 1:
            self.record_grasp_data(req, self.suc_grasp_file_name, object_id)
        elif req.grasp_success_label == 0:
            self.record_grasp_data(req, self.failure_grasp_file_name, object_id)
        else:
            rospy.logerr('Wrong grasp label for data recording!')

        response = SimGraspDataResponse()
        response.object_id = object_id
        response.save_h5_success = True
        grasp_file.close()
        return response


    def record_grasp_data(self, req, file_name=None, object_id=None):
        if file_name is None:
            #'r+': Read/write, file must exist
            grasp_file = h5py.File(self.grasp_file_name, 'r+')
            if req.grasp_id == 0:
                grasp_file['max_object_id'][()] +=1
                grasp_file['cur_object_name'][()] = req.object_name

            # [()] is the way to get the scalar value from h5 file.
            #object_id = grasp_file['max_object_id'][()] + 1 
            object_id = grasp_file['max_object_id'][()] 
        else:
            grasp_file = h5py.File(file_name, 'r+')

        object_grasp_id = 'object_' + str(object_id) + '_grasp_' + str(req.grasp_id)
        
        global_grasp_id = grasp_file['total_grasps_num'][()]
        obj_grasp_id_key = 'grasp_' + str(global_grasp_id) + '_obj_grasp_id'
        if obj_grasp_id_key not in grasp_file:
            grasp_file.create_dataset(obj_grasp_id_key, data=object_grasp_id)

        grasp_object_name_key = 'object_' + str(object_id) + '_name'
        if grasp_object_name_key not in grasp_file:
            grasp_file.create_dataset(grasp_object_name_key, data=req.object_name)
    
        grasp_time_stamp_key = object_grasp_id + '_time_stamp'
        if grasp_time_stamp_key not in grasp_file:
            grasp_file.create_dataset(grasp_time_stamp_key, data=req.time_stamp)

        voxel_grid_key = object_grasp_id + '_sparse_voxel_grid'
        if voxel_grid_key not in grasp_file:
            voxel_grid = np.reshape(req.sparse_voxel_grid, 
                        [len(req.sparse_voxel_grid) / 3, 3])
            grasp_file.create_dataset(voxel_grid_key, data=voxel_grid)

        object_size_key = object_grasp_id + '_object_size'
        if object_size_key not in grasp_file:
            grasp_file.create_dataset(object_size_key, data=req.object_size)

        init_ik_config_array_key = object_grasp_id + '_init_ik_config_array'
        if init_ik_config_array_key not in grasp_file:
            grasp_file.create_dataset(init_ik_config_array_key, data=req.init_ik_config_array)

        init_config_array_key = object_grasp_id + '_init_config_array'
        if init_config_array_key not in grasp_file:
            grasp_file.create_dataset(init_config_array_key, data=req.init_config_array)

        init_val_key = object_grasp_id + '_init_val' 
        if init_val_key not in grasp_file:
            grasp_file.create_dataset(init_val_key, data=req.init_val)

        init_suc_prob_key = object_grasp_id + '_init_suc_prob' 
        if init_suc_prob_key not in grasp_file:
            grasp_file.create_dataset(init_suc_prob_key, data=req.init_suc_prob)

        init_log_prior_key = object_grasp_id + '_init_log_prior' 
        if init_log_prior_key not in grasp_file:
            grasp_file.create_dataset(init_log_prior_key, data=req.init_log_prior)

        init_uct_key = object_grasp_id + '_init_uct' 
        if init_uct_key not in grasp_file:
            grasp_file.create_dataset(init_uct_key, data=req.init_uct)

        inf_ik_config_array_key = object_grasp_id + '_inf_ik_config_array'
        if inf_ik_config_array_key not in grasp_file:
            grasp_file.create_dataset(inf_ik_config_array_key, data=req.inf_ik_config_array)

        inf_config_array_key = object_grasp_id + '_inf_config_array'
        if inf_config_array_key not in grasp_file:
            grasp_file.create_dataset(inf_config_array_key, data=req.inf_config_array)

        inf_val_key = object_grasp_id + '_inf_val' 
        if inf_val_key not in grasp_file:
            grasp_file.create_dataset(inf_val_key, data=req.inf_val)

        inf_suc_prob_key = object_grasp_id + '_inf_suc_prob' 
        if inf_suc_prob_key not in grasp_file:
            grasp_file.create_dataset(inf_suc_prob_key, data=req.inf_suc_prob)

        inf_log_prior_key = object_grasp_id + '_inf_log_prior' 
        if inf_log_prior_key not in grasp_file:
            grasp_file.create_dataset(inf_log_prior_key, data=req.inf_log_prior)

        inf_uct_key = object_grasp_id + '_inf_uct' 
        if inf_uct_key not in grasp_file:
            grasp_file.create_dataset(inf_uct_key, data=req.inf_uct)

        reward_key = object_grasp_id + '_reward' 
        if reward_key not in grasp_file:
            grasp_file.create_dataset(reward_key, data=req.reward)

        action_key = object_grasp_id + '_action' 
        if action_key not in grasp_file:
            grasp_file.create_dataset(action_key, data=req.action)

        preshape_palm_world_pose_list = [req.preshape_palm_world_pose.pose.position.x, req.preshape_palm_world_pose.pose.position.y,
               req.preshape_palm_world_pose.pose.position.z, req.preshape_palm_world_pose.pose.orientation.x, 
               req.preshape_palm_world_pose.pose.orientation.y, req.preshape_palm_world_pose.pose.orientation.z, 
               req.preshape_palm_world_pose.pose.orientation.w]
        palm_world_pose_key = object_grasp_id + '_preshape_palm_world_pose'
        if palm_world_pose_key not in grasp_file:
            grasp_file.create_dataset(palm_world_pose_key, 
                    data=preshape_palm_world_pose_list)

        true_preshape_palm_world_pose_list = [req.true_preshape_palm_world_pose.pose.position.x, req.true_preshape_palm_world_pose.pose.position.y,
               req.true_preshape_palm_world_pose.pose.position.z, req.true_preshape_palm_world_pose.pose.orientation.x, 
               req.true_preshape_palm_world_pose.pose.orientation.y, req.true_preshape_palm_world_pose.pose.orientation.z, 
               req.true_preshape_palm_world_pose.pose.orientation.w]
        palm_world_pose_key = object_grasp_id + '_true_preshape_palm_world_pose'
        if palm_world_pose_key not in grasp_file:
            grasp_file.create_dataset(palm_world_pose_key, 
                    data=true_preshape_palm_world_pose_list)

        close_shape_palm_world_pose_list = [req.close_shape_palm_world_pose.pose.position.x, req.close_shape_palm_world_pose.pose.position.y,
               req.close_shape_palm_world_pose.pose.position.z, req.close_shape_palm_world_pose.pose.orientation.x, 
               req.close_shape_palm_world_pose.pose.orientation.y, req.close_shape_palm_world_pose.pose.orientation.z, 
               req.close_shape_palm_world_pose.pose.orientation.w]
        palm_world_pose_key = object_grasp_id + '_close_shape_palm_world_pose'
        if palm_world_pose_key not in grasp_file:
            grasp_file.create_dataset(palm_world_pose_key, 
                    data=close_shape_palm_world_pose_list)

        lift_shape_palm_world_pose_list = [req.lift_shape_palm_world_pose.pose.position.x, req.lift_shape_palm_world_pose.pose.position.y,
               req.lift_shape_palm_world_pose.pose.position.z, req.lift_shape_palm_world_pose.pose.orientation.x, 
               req.lift_shape_palm_world_pose.pose.orientation.y, req.lift_shape_palm_world_pose.pose.orientation.z, 
               req.lift_shape_palm_world_pose.pose.orientation.w]
        palm_world_pose_key = object_grasp_id + '_lift_shape_palm_world_pose'
        if palm_world_pose_key not in grasp_file:
            grasp_file.create_dataset(palm_world_pose_key, 
                    data=lift_shape_palm_world_pose_list)

        object_world_seg_pose_list = [req.object_world_seg_pose.pose.position.x, req.object_world_seg_pose.pose.position.y,
               req.object_world_seg_pose.pose.position.z, req.object_world_seg_pose.pose.orientation.x,
               req.object_world_seg_pose.pose.orientation.y, req.object_world_seg_pose.pose.orientation.z, 
               req.object_world_seg_pose.pose.orientation.w]
        object_world_seg_pose_key = object_grasp_id + '_object_world_seg_pose'
        if object_world_seg_pose_key not in grasp_file:
            grasp_file.create_dataset(object_world_seg_pose_key, 
                    data=object_world_seg_pose_list)

        object_world_sim_pose_list = [req.object_world_sim_pose.pose.position.x, req.object_world_sim_pose.pose.position.y,
               req.object_world_sim_pose.pose.position.z, req.object_world_sim_pose.pose.orientation.x,
               req.object_world_sim_pose.pose.orientation.y, req.object_world_sim_pose.pose.orientation.z, 
               req.object_world_sim_pose.pose.orientation.w]
        object_world_sim_pose_key = object_grasp_id + '_object_world_sim_pose'
        if object_world_sim_pose_key not in grasp_file:
            grasp_file.create_dataset(object_world_sim_pose_key, 
                    data=object_world_sim_pose_list)

        preshape_js_position_key = object_grasp_id + '_preshape_joint_state_position'
        if preshape_js_position_key not in grasp_file:
            grasp_file.create_dataset(preshape_js_position_key, 
                    data=req.preshape_allegro_joint_state.position)

        true_preshape_js_position_key = object_grasp_id + '_true_preshape_js_position'
        if true_preshape_js_position_key not in grasp_file:
            grasp_file.create_dataset(true_preshape_js_position_key, 
                    data=req.true_preshape_joint_state.position)

        close_js_position_key = object_grasp_id + '_close_shape_joint_state_position'
        if close_js_position_key not in grasp_file:
            grasp_file.create_dataset(close_js_position_key, 
                    data=req.close_shape_allegro_joint_state.position)

        lift_js_position_key = object_grasp_id + '_lift_shape_joint_state_position'
        if lift_js_position_key not in grasp_file:
            grasp_file.create_dataset(lift_js_position_key, 
                    data=req.lift_shape_allegro_joint_state.position)

        grasp_label_key = object_grasp_id + '_grasp_label' 
        if (grasp_label_key not in grasp_file) and \
                (len(req.inf_config_array) != 0):
            grasp_file.create_dataset(grasp_label_key, 
                    data=req.grasp_success_label)
            if req.grasp_success_label == 1:
                grasp_file['suc_grasps_num'][()] += 1
            grasp_file['total_grasps_num'][()] += 1

        top_grasp_key = object_grasp_id + '_top_grasp'
        if top_grasp_key not in grasp_file:
            grasp_file.create_dataset(top_grasp_key,
                    data=req.top_grasp)


    def create_record_data_server(self):
        rospy.Service('record_grasp_data', SimGraspData, self.handle_record_grasp_data)
        rospy.loginfo('Service record_grasp_data:')
        rospy.loginfo('Ready to record grasp data.')


if __name__ == '__main__':
    record_active_data = RecordActiveData()
    record_active_data.create_record_data_server()
    rospy.spin()


