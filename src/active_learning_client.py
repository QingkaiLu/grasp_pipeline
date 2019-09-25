#!/usr/bin/env python

import os
import rospy
import time
from grasp_client import GraspClient 


if __name__ == '__main__':
    dc_client = GraspClient()
    #dataset_dir = '/mnt/tars_data/sim_dataset/BigBird/BigBird_mesh'
    dataset_name = 'BigBird'
    #dataset_name = 'GraspDatabase'
    dataset_dir = '/mnt/tars_data/sim_dataset/' + dataset_name \
                    + '/' + dataset_name + '_mesh'
    object_mesh_dirs = os.listdir(dataset_dir)
    # Reverse object mesh directory list
    # object_mesh_dirs = object_mesh_dirs[::-1]
    # dc_client.last_object_name = 'empty'
    # print object_mesh_dirs

    #object_mesh_dirs = ['pringles_bbq'] 

    # Bigbird objects that don't work for gazebo+dart.
    bigbird_objects_exclude = {'coca_cola_glass_bottle', 'softsoap_clear',
                       'softsoap_green', 'softsoap_purple', 'windex',
                       'palmolive_orange', 'palmolive_green', 
                       #'progresso_new_england_clam_chowder', 
                       'listerine_green', 'hersheys_bar', 'mahatma_rice', 
                       'paper_plate'}
    #exclude_object = 'windex'
    skip_object = True
    grasp_failure_retry_times = dc_client.num_grasps_per_object
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
        elif dataset_name == 'GraspDatabase':
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
            if grasp_plan_failures_num >= grasp_failure_retry_times:
                rospy.loginfo('Reached the maximum retry times due to planning failures!')
                break

            # Use the tsdf mesh for moveit, since the poission mesh doesn't work for moveit.
            #dc_client.set_up_object_name(object_name=object_name, object_mesh_path=object_poisson_mesh_path + '.stl')
            dc_client.set_up_object_name(object_name=object_name, object_mesh_path=obj_mesh_path)
            dc_client.set_up_grasp_id(grasp_id)

            # object_pose_stamped = dc_client.gen_object_pose() 
            object_pose_stamped = dc_client.gen_exp_object_pose()
            dc_client.move_gazebo_object_client(object_model_name=object_name, 
                                                object_pose_stamped=object_pose_stamped)
            
            #Notice: if one object x doesn't have any grasp with valid plans, the next object y
            #is going to have the same object_id with x.
            object_found = dc_client.segment_and_generate_preshape()
            if not object_found:
                # grasp_plan_failures_num += 3 
                grasp_plan_failures_num += 1 
                # raw_input('Object not found!')
                continue

            # for i in xrange(len(dc_client.preshape_response.allegro_joint_state)):
            #     dc_client.set_up_grasp_id(grasp_id)
            #     if i > 0:
            #         dc_client.move_gazebo_object_client(object_model_name=object_name, 
            #                                             object_pose_stamped=object_pose_stamped)
            #     grasp_arm_plan = dc_client.grasp_and_lift_object_steps(object_pose_stamped, i)
            #     if not grasp_arm_plan:
            #         rospy.logerr('Can not find moveit plan to grasp. Ignore this grasp.\n')
            #         grasp_plan_failures_num += 1
            #         # Changing the grasp_id to num_grasps_per_object is tricky.
            #         # For example, it can be tricky to find the pcd file for the valid grasp with grasp_id
            #         # as num_grasps_per_object. 
            #         #if grasp_plan_failures_num >= grasp_failure_retry_times:
            #         #    rospy.loginfo('Reached the maximum retry times due to planning failures!')
            #         #    grasp_id = dc_client.num_grasps_per_object

            #         dc_client.place_object_steps(move_arm=grasp_arm_plan)
            #         continue
            #     
            #     dc_client.record_grasp_data_client(i)
            #     grasp_id += 1

            #     dc_client.place_object_steps(move_arm=grasp_arm_plan)

            dc_client.set_up_grasp_id(grasp_id)
            grasp_arm_plan = dc_client.grasp_and_lift_object_steps(object_pose_stamped)
            if not grasp_arm_plan:
                rospy.logerr('Can not find moveit plan to grasp.\n')
                grasp_plan_failures_num += 1
                # dc_client.record_data_client_no_plan()
                dc_client.place_object_steps(move_arm=grasp_arm_plan)
                continue
            dc_client.record_grasp_data_client()

            dc_client.place_object_steps(move_arm=grasp_arm_plan)

            #TODO: Check batch id and update the active learning model 
            # if a whole new batch is generated
            dc_client.get_total_grasps_num()
            print '************************************'
            print dc_client.total_grasps_num
            if(dc_client.total_grasps_num != 0 and 
                    dc_client.total_grasps_num % dc_client.queries_num_per_batch == 0): 
                dc_client.active_model_update_client(dc_client.total_grasps_num / 
                                            dc_client.queries_num_per_batch - 1)
            dc_client.active_data_update_client(grasp_arm_plan)
            # raw_input('Wait')

            grasp_id += 1

            # raw_input('Wait')
 
            elapsed_time = time.time() - start_time
            rospy.loginfo('Time: %s' %str(elapsed_time))

        rospy.loginfo('All grasps are finished for object: %s' %object_name) 

