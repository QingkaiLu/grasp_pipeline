#!/usr/bin/env python


import rosbag
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from grasp_pipeline.srv import *


class BagVisualData():
    '''
    ROS service to bag visual data.
    '''
    
    def __init__(self):
        rospy.init_node('bag_visual_data_server')
        self.data_recording_path = rospy.get_param('~data_recording_path', 
                                                    '/dataspace/data_kai/multi_finger_exp_data/')

        self.hd_cam_info_topic = rospy.get_param('~hd_cam_info_topic', '/kinect2/hd/camera_info')
        rospy.Subscriber(self.hd_cam_info_topic, CameraInfo, self.get_hd_camera_info)
        self.hd_color_rect_topic = rospy.get_param('~hd_color_rect_topic', '/kinect2/hd/image_color_rect')
        rospy.Subscriber(self.hd_color_rect_topic, Image, self.get_hd_color_rect_img)
        self.hd_depth_rect_topic = rospy.get_param('~hd_depth_rect_topic', '/kinect2/hd/image_depth_rect')
        rospy.Subscriber(self.hd_depth_rect_topic, Image, self.get_hd_depth_rect_img)
        self.hd_pcd_topic = rospy.get_param('~hd_pcd_topic', '/kinect2/hd/points')
        rospy.Subscriber(self.hd_pcd_topic, PointCloud2, self.get_hd_pcd)

        self.sd_cam_info_topic = rospy.get_param('~sd_cam_info_topic', '/kinect2/sd/camera_info')
        rospy.Subscriber(self.sd_cam_info_topic, CameraInfo, self.get_sd_camera_info)
        self.sd_color_rect_topic = rospy.get_param('~sd_color_rect_topic', '/kinect2/sd/image_color_rect')
        rospy.Subscriber(self.sd_color_rect_topic, Image, self.get_sd_color_rect_img)
        self.sd_depth_rect_topic = rospy.get_param('~sd_depth_rect_topic', '/kinect2/sd/image_depth_rect')
        rospy.Subscriber(self.sd_depth_rect_topic, Image, self.get_sd_depth_rect_img)
        self.sd_pcd_topic = rospy.get_param('~sd_pcd_topic', '/kinect2/sd/points')
        rospy.Subscriber(self.sd_pcd_topic, PointCloud2, self.get_sd_pcd)

        self.bag = None
        self.bag_visual = False
        self.bag_writting_finish = True
        #self.bag_write_mutex = True

        self.hd_cam_info = None
        self.hd_color_rect_img = None
        self.hd_depth_rect_img = None
        self.hd_pcd = None
        self.sd_cam_info = None
        self.sd_color_rect_img = None
        self.sd_depth_rect_img = None
        self.sd_pcd = None


    #def get_hd_camera_info(self, hd_cam_info):
    #    if self.bag_visual:
    #        self.bag.write(self.hd_cam_info_topic, hd_cam_info)

    #def get_hd_color_rect_img(self, hd_color_rect_img):
    #    if self.bag_visual:
    #        self.bag.write(self.hd_color_rect_topic, hd_color_rect_img)

    #def get_hd_depth_rect_img(self, hd_depth_rect_img):
    #    if self.bag_visual:
    #        self.bag.write(self.hd_depth_rect_topic, hd_depth_rect_img)

    #def get_hd_pcd(self, hd_pcd):
    #    if self.bag_visual:
    #        self.bag.write(self.hd_pcd_topic, hd_pcd)

    #def get_sd_camera_info(self, sd_cam_info):
    #    if self.bag_visual:
    #        self.bag.write(self.sd_cam_info_topic, sd_cam_info)

    #def get_sd_color_rect_img(self, sd_color_rect_img):
    #    if self.bag_visual:
    #        self.bag.write(self.sd_color_rect_topic, sd_color_rect_img)

    #def get_sd_depth_rect_img(self, sd_depth_rect_img):
    #    if self.bag_visual:
    #        self.bag.write(self.sd_depth_rect_topic, sd_depth_rect_img)

    #def get_sd_pcd(self, sd_pcd):
    #    if self.bag_visual:
    #        self.bag.write(self.sd_pcd_topic, sd_pcd)

    def get_hd_camera_info(self, hd_cam_info):
        self.hd_cam_info = hd_cam_info

    def get_hd_color_rect_img(self, hd_color_rect_img):
        self.hd_color_rect_img = hd_color_rect_img

    def get_hd_depth_rect_img(self, hd_depth_rect_img):
        self.hd_depth_rect_img = hd_depth_rect_img

    def get_hd_pcd(self, hd_pcd):
        self.hd_pcd = hd_pcd

    def get_sd_camera_info(self, sd_cam_info):
        self.sd_cam_info = sd_cam_info

    def get_sd_color_rect_img(self, sd_color_rect_img):
        self.sd_color_rect_img = sd_color_rect_img

    def get_sd_depth_rect_img(self, sd_depth_rect_img):
        self.sd_depth_rect_img = sd_depth_rect_img

    def get_sd_pcd(self, sd_pcd):
        self.sd_pcd = sd_pcd

    def create_bag_visual_server(self):
        rospy.Service('bag_visual_data', GraspDataBagging,
                      self.handle_bag_visual_data)
        rospy.loginfo('Service bag_visual_data:')
        rospy.loginfo('Ready to bag visual data.')

    def write_bag_data(self):
        if self.bag_visual:
            self.bag_writting_finish = False
            if self.hd_cam_info is not None:
                self.bag.write(self.hd_cam_info_topic, self.hd_cam_info)
                self.hd_cam_info = None

            if self.hd_color_rect_img is not None:
                self.bag.write(self.hd_color_rect_topic, self.hd_color_rect_img)
                self.hd_color_rect_img = None

            if self.hd_depth_rect_img is not None:
                self.bag.write(self.hd_depth_rect_topic, self.hd_depth_rect_img)
                self.hd_depth_rect_img = None

            if self.hd_pcd is not None:
                self.bag.write(self.hd_pcd_topic, self.hd_pcd)
                self.hd_pcd = None

            if self.sd_cam_info is not None:
                self.bag.write(self.sd_cam_info_topic, self.sd_cam_info)
                self.sd_cam_info = None

            if self.sd_color_rect_img is not None:
                self.bag.write(self.sd_color_rect_topic, self.sd_color_rect_img)
                self.sd_color_rect_img = None

            if self.sd_depth_rect_img is not None:
                self.bag.write(self.sd_depth_rect_topic, self.sd_depth_rect_img)
                self.sd_depth_rect_img = None

            if self.sd_pcd is not None:
                self.bag.write(self.sd_pcd_topic, self.sd_pcd)
                self.sd_pcd = None

            self.bag_writting_finish = True

    def handle_bag_visual_data(self, req):
        response = GraspDataBaggingResponse()
        if req.operation == 'start':
            # file_name = 'object_' + str(req.object_id) + '_' + req.object_name + \
            #         '_grasp_' + str(req.grasp_id) + '_' + req.grasp_type + '_' + \
            #         req.grasp_control_type + '_' + req.grasp_phase + '_visual.bag' 
            file_name = 'object_' + str(req.object_id) + '_' + req.object_name + \
                    '_grasp_' + str(req.grasp_id) + '_' + req.grasp_phase + '_visual.bag' 
            bag_file_name = self.data_recording_path + 'grasp_data/' + file_name
            self.bag = rosbag.Bag(bag_file_name, 'w')
            self.bag_visual = True
            rospy.loginfo('Start to bag visual data: %s.'%file_name)
        elif req.operation == 'stop':
            self.bag_visual = False
            while(not self.bag_writting_finish):
                pass
            self.bag.close()
            self.bag = None
            rospy.loginfo('Stoped to bag visual data.')
        response.bag_data_success = True
        return response
        
if __name__ == '__main__':
    bag_visual_data = BagVisualData()
    bag_visual_data.create_bag_visual_server() 
    #rospy.spin()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        bag_visual_data.write_bag_data()
        rate.sleep()

