#!/usr/bin/env python

import os
import rospy
import time
from grasp_sim_client import GraspClient 


if __name__ == '__main__':
    dc_client = GraspClient()
    # dataset_name = 'BigBird'
    # dataset_name = 'GraspDatabase'
    dataset_name = 'YCB'
    dataset_dir = '/mnt/tars_data/sim_dataset/' + dataset_name \
                    + '/' + dataset_name + '_mesh'
    object_mesh_dirs = os.listdir(dataset_dir)
    # object_mesh_dirs = ['006_mustard_bottle']
    # object_mesh_dirs = object_mesh_dirs[:10]
    # Reverse object mesh directory list
    # object_mesh_dirs = object_mesh_dirs[::-1]
    # print object_mesh_dirs

    #object_mesh_dirs = ['pringles_bbq'] 

    # Bigbird objects that don't work for gazebo+dart.
    bigbird_objects_exclude = {'coca_cola_glass_bottle', 'softsoap_clear',
                       'softsoap_green', 'softsoap_purple', 'windex',
                       'palmolive_orange', 'palmolive_green', 
                       #'progresso_new_england_clam_chowder', 
                       'listerine_green', 'hersheys_bar', 'mahatma_rice', 
                       'paper_plate'}
    ycb_objects_exclude = {'006_mustard_bottle', '012_strawberry', 
                            '058_golf_ball', '062_dice', '063-b_marbles', 
                            '070-a_colored_wood_blocks', '070-b_colored_wood_blocks'}
    #exclude_object = 'windex'
    skip_object = True
    # grasp_failure_retry_times = dc_client.num_grasps_per_object
    prior_names = ['MDN', 'GMM', 'Constraint'] 
    # prior_names = ['Constraint'] 
    prior_num = len(prior_names)
    # grasp_type = 'unknown'
    # grasp_type = 'side'
    grasp_type = 'overhead'

    for i, object_name in enumerate(object_mesh_dirs):
        rospy.loginfo('Object: %s' %object_name)

        dc_client.get_last_object_id_name()
        # Resume from the last object.
        if dc_client.last_object_name == 'empty' or \
                object_name == dc_client.last_object_name:
            skip_object = False
        if skip_object:
            continue

        if dataset_name == 'BigBird' and object_name in bigbird_objects_exclude:
            continue

        if dataset_name == 'YCB' and object_name in ycb_objects_exclude:
            continue

        #if object_name == exclude_object:
        #    skip_object = False
        #if skip_object:
        #    continue
        #if object_name == exclude_object:
        #    continue

        if dataset_name == 'BigBird':
            obj_mesh_path = dataset_dir + '/' + object_name + \
                            '/textured_meshes/optimized_tsdf_textured_mesh.stl'
            #obj_mesh_path = dataset_dir + '/' + object_name + \
            #                '/meshes/poisson_proc'
        elif dataset_name == 'GraspDatabase' or dataset_name == 'YCB':
            obj_mesh_path = dataset_dir + '/' + object_name + \
                            '/' + object_name + '_proc.stl' 

        object_pose_array = [0., 0., 0., 0., -0.8, 0.59]
        object_pose_stamped = dc_client.get_pose_stamped_from_array(object_pose_array) 
        dc_client.update_gazebo_object_client(object_name, object_pose_array, object_model_name=object_name)

        grasp_id = 0
        grasp_plan_failures_num = 0
        while grasp_id < dc_client.num_grasps_per_object:
            start_time = time.time()
            rospy.loginfo('Grasp_id: %s' %str(grasp_id))

            # It is a better way to directly stop grasps collection for the current object
            # when the number of failure grasps is above the retry times.
            # if grasp_plan_failures_num >= grasp_failure_retry_times:
            #     rospy.loginfo('Reached the maximum retry times due to planning failures!')
            #     break

            # Use the tsdf mesh for moveit, since the poission mesh doesn't work for moveit.
            #dc_client.set_up_object_name(object_name=object_name, object_mesh_path=object_poisson_mesh_path + '.stl')
            dc_client.set_up_object_name(object_name=object_name, object_mesh_path=obj_mesh_path)
            dc_client.set_up_grasp_id(grasp_id)

            object_pose_stamped = dc_client.gen_exp_object_pose() 
            dc_client.move_gazebo_object_client(object_model_name=object_name, 
                                                object_pose_stamped=object_pose_stamped)
            
            object_found = dc_client.segment_and_get_vis_data()
            if not object_found:
                # grasp_plan_failures_num += 3 
                # grasp_plan_failures_num += 1 
                # raw_input('Object not found!')
                continue

            # for pid, prior_name in enumerate(prior_names):
            for pid in xrange(prior_num):
                dc_client.set_up_grasp_id(grasp_id)
                prior_name = prior_names[grasp_id % prior_num] 
                print 'grasp_id: ', grasp_id
                print 'prior_name: ', prior_name
                if pid > 0:
                    dc_client.move_gazebo_object_client(object_model_name=object_name, 
                                                        object_pose_stamped=object_pose_stamped)
                dc_client.voxel_plan_preshape_client(prior_name, grasp_type)
                grasp_arm_plan = dc_client.grasp_and_lift_object_steps(object_pose_stamped)
                if not grasp_arm_plan:
                    rospy.logerr('Can not find moveit plan to grasp.\n')
                    grasp_plan_failures_num += 1
                    dc_client.record_voxel_np_data_client(prior_name)
                else:
                    dc_client.record_voxel_data_client(prior_name)

                dc_client.move_robot_home(move_arm=grasp_arm_plan)

                #TODO: Check batch id and update the active learning model 
                # if a whole new batch is generated
                dc_client.get_total_grasps_num()
                print '************************************'
                print dc_client.total_grasps_num
                # raw_input('Wait')

                grasp_id += 1

            # raw_input('Wait')
 
            elapsed_time = time.time() - start_time
            rospy.loginfo('Time: %s' %str(elapsed_time))

        rospy.loginfo('All grasps are finished for object: %s' %object_name) 

