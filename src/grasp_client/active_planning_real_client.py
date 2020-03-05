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

    # dc_client.move_arm_zero()

    # dc_client.control_allegro_config_client(go_home=True)
    # raw_input('Wait')
    # dc_client.hand_grasp_control()

    move_home_cmd = raw_input('Move robot to the home position for planning or not? (y or n).')
    dc_client.move_robot_home = move_home_cmd == 'y'
    if dc_client.move_robot_home:
        dc_client.move_arm_home()
        rospy.loginfo('Hand go home.')
        dc_client.control_allegro_config_client(go_home=True)
    
    planner_names = ['active_planner', 'more_data_spv_planner', 
                    'less_data_spv_planner']
    # planner_names = ['active_planner'] 
    planners_num = len(planner_names)

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

        for pid, planner_name in enumerate(planner_names):
            dc_client.get_cur_object_id()
            dc_client.get_cur_grasp_id()
            rospy.loginfo('object id: %s, object name: %s, grasp id: %s, planner id: %s\n', 
                dc_client.cur_object_id, object_name, dc_client.cur_grasp_id, pid)

            if pid < ((dc_client.cur_grasp_id + 1) % planners_num):
                print pid
                continue

            raw_input('Press any key to start grasp.')

            # print 'cur_object_id: ', dc_client.cur_object_id
            # print 'cur_grasp_id: ', dc_client.cur_grasp_id
            
            grasp_arm_plan = False
            for t in xrange(motion_plan_max_tries): 
                dc_client.segment_and_get_vis_data()
                dc_client.active_plan_preshape_client(planner_name)
                # skip = raw_input('Execute? y/n.')
                # if skip != 'y':
                #     continue

                grasp_arm_plan = dc_client.grasp_and_lift_object_steps()
                if grasp_arm_plan:
                    break
                else:
                    rospy.logerr('Can not find moveit plan to grasp after %s attepmts\n' %str(t+1))

            if not grasp_arm_plan:
                rospy.logerr('Can not find moveit plan to grasp.\n')
                # grasp_plan_failures_num += 1
                dc_client.record_active_np_data_client()
                # dc_client.place_object_steps(move_arm=grasp_arm_plan)
                # continue
            else:
                rospy.loginfo('Motion plan found! Record data!')
                dc_client.record_active_data_client()

            # record_grasp = raw_input('Do you want to record this grasp? Please input: y (yes) or n (no).')
            # if record_grasp == 'n':
            #     continue

            #Move arm and hand home after grasping and lifting.
            raw_input('Ready to move arm home. Please release the object and take grasp photos.')
            if grasp_arm_plan:
                dc_client.move_arm_home()
                rospy.loginfo('Hand go home.')
                dc_client.control_allegro_config_client(go_home=True)

