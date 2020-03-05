#!/usr/bin/env python


import rosbag
import rospy
from biotac_sensors.msg import *
from grasp_pipeline.srv import *


class BagTactileData():
    '''
    ROS service to bag tactile data.
    '''
    
    def __init__(self):
        rospy.init_node('bag_tactile_data_server')
        self.data_recording_path = rospy.get_param('~data_recording_path', 
                                                    '/dataspace/data_kai/multi_finger_exp_data/')
        self.biotac_topic = rospy.get_param('~biotac_topic', 'biotac_pub')
        rospy.Subscriber(self.biotac_topic, BioTacHand, self.get_biotac_data)
        self.bag = None
        self.bag_tactile = False

    def get_biotac_data(self, biotac_data):
        if self.bag_tactile:
            self.bag.write(self.biotac_topic, biotac_data)

    def create_bag_tactile_server(self):
        rospy.Service('bag_tactile_data', GraspDataBagging,
                      self.handle_bag_tactile_data)
        rospy.loginfo('Service bag_tactile_data:')
        rospy.loginfo('Ready to bag tactile data.')

    def handle_bag_tactile_data(self, req):
        response = GraspDataBaggingResponse()
        if req.operation == 'start':
            # file_name = 'object_' + str(req.object_id) + '_' + req.object_name + \
            #         '_grasp_' + str(req.grasp_id) + '_' + req.grasp_type + '_' + \
            #         req.grasp_control_type + '_' + req.grasp_phase + '_tactile.bag' 
            file_name = 'object_' + str(req.object_id) + '_' + req.object_name + \
                    '_grasp_' + str(req.grasp_id) + '_' + req.grasp_phase + '_tactile.bag' 
            bag_file_name = self.data_recording_path + 'grasp_data/' + file_name
            self.bag = rosbag.Bag(bag_file_name, 'w')
            self.bag_tactile = True
            rospy.loginfo('Start to bag tactile data: %s.'%file_name)
        elif req.operation == 'stop':
            self.bag_tactile = False
            self.bag.close()
            self.bag = None
            rospy.loginfo('Stoped to bag tactile data.')
        response.bag_data_success = True
        return response
        
if __name__ == '__main__':
    bag_tactile_data = BagTactileData()
    bag_tactile_data.create_bag_tactile_server() 
    rospy.spin()
