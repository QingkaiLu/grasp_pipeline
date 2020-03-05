#!/usr/bin/env python

import os
import rospy
import time
from grasp_real_client import GraspClient 


if __name__ == '__main__':
    #Segment -> generate preshape -> listen pose ->
    #create moveit scene -> move hand to preshape (To do) -> move arm to preshape
    #close hand to grasp (To do) -> clean object from moveit -> lift object 
    #-> record data -> hand go home -> arm go home

    dc_client = GraspClient()

    dc_client.move_arm_zero()

    # dc_client.control_allegro_config_client(go_home=True)
    # raw_input('Wait')
    # dc_client.hand_grasp_control()

    move_home_cmd = raw_input('Move robot to the home position for planning or not? (y or n).')
    dc_client.move_robot_home = move_home_cmd == 'y'
    if dc_client.move_robot_home:
        dc_client.move_arm_home()
        rospy.loginfo('Hand go home.')
        dc_client.control_allegro_config_client(go_home=True)
    
    prior_names = ['MDN', 'GMM', 'Constraint'] 
    prior_num = len(prior_names)
    # grasp_type = 'unknown'
    grasp_type = 'side'
    # grasp_type = 'overhead'

    motion_plan_max_tries = 5

    while True:
        object_name = raw_input('### Input the new object name or input stop to quit: \n')
        if object_name == 'stop':
            rospy.loginfo('Quit grasp data collection.')
            break

        # while True:
        #     object_name_confirmation = raw_input('### Are you sure ' + object_name + ' is the correct object name? ' + \
        #             'Please input y to continue or n to re-input object name. \n')
        #     if object_name_confirmation == 'y':
        #         break
        #    object_name = raw_input('### Input the new object name or input stop to quit: \n')

        dc_client.set_object_name(object_name)

        for pid in xrange(prior_num):
            dc_client.get_cur_object_id()
            dc_client.get_cur_grasp_id()
            rospy.loginfo('object id: %s, object name: %s, grasp id: %s, prior id: %s\n', 
                dc_client.cur_object_id, object_name, dc_client.cur_grasp_id, pid)

            if pid < ((dc_client.cur_grasp_id + 1) % prior_num):
                print pid
                continue

            raw_input('Press any key to start grasp.')

            # print 'cur_object_id: ', dc_client.cur_object_id
            # print 'cur_grasp_id: ', dc_client.cur_grasp_id
            prior_name = prior_names[(dc_client.cur_grasp_id + 1) % prior_num] 
            
            grasp_arm_plan = False
            for t in xrange(motion_plan_max_tries): 
                dc_client.segment_and_get_vis_data()
                dc_client.voxel_plan_preshape_client(prior_name, grasp_type)
                raw_input('Wait')
                # continue

                grasp_arm_plan = dc_client.grasp_and_lift_object_steps()
                if grasp_arm_plan:
                    break
                else:
                    rospy.logerr('Can not find moveit plan to grasp after %s attepmts\n' %str(t+1))

            if not grasp_arm_plan:
                rospy.logerr('Can not find moveit plan to grasp. \n')
                raw_input('Press any key to continue to record the grasp without arm plan.')
            else:
                rospy.loginfo('Motion plan found! Record data!')

                # record_grasp = raw_input('Do you want to record this grasp? Please input: y (yes) or n (no).')
                # if record_grasp == 'n':
                #     continue

            dc_client.record_voxel_data_client(prior_name)

            #Move arm and hand home after grasping and lifting.
            raw_input('Ready to move arm home. Please release the object and take grasp photos.')
            if grasp_arm_plan:
                dc_client.move_arm_home()
                rospy.loginfo('Hand go home.')
                dc_client.control_allegro_config_client(go_home=True)

