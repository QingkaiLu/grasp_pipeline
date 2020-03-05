#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import rospy

from grasp_pipeline.srv import *
from point_cloud_segmentation.srv import *
import cv2
from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
    rospy.init_node('preshape_test_client')
    seg_proxy = rospy.ServiceProxy('/object_segmenter', SegmentGraspObject)
    data_recording_proxy = rospy.ServiceProxy('/save_visual_data', GraspDataRecording)

    rospy.loginfo('Requesting segmentation')
    try:
        seg_result = seg_proxy(SegmentGraspObjectRequest())
    except rospy.ServiceException as e:
        rospy.logwarn("Segmentation service did not process request: %s"%str(e))
        exit()

    # bridge = CvBridge()
    # depth_img = bridge.imgmsg_to_cv2(seg_result.scene_depth_img)
    # cv2.imshow('depth_img',depth_img)
    # cv2.waitKey()

    base_path = '/home/kai'

    data_recording_req = GraspDataRecordingRequest()
    data_recording_req.scene_cloud = seg_result.scene_cloud
    data_recording_req.scene_cloud_save_path = base_path+'/test_pcd.pcd'
    data_recording_req.rgb_image_save_path =  base_path+'/test_rgb.ppm'
    data_recording_req.depth_image_save_path =  base_path+'/test_depth.ppm'
    data_recording_req.scene_depth_img = seg_result.scene_depth_img
    try:
        data_recording_response = data_recording_proxy(data_recording_req)
    except rospy.ServiceException as e:
        rospy.logwarn("Data Recording service did not process request: %s"%str(e))
        exit()

    rospy.loginfo('Data recording response: ' + str(data_recording_response.save_visual_data_success))

    read_depth_img = cv2.imread(data_recording_req.depth_image_save_path)
    cv2.imshow('Read depth image', read_depth_img)
    bridge = CvBridge()
    depth_img = bridge.imgmsg_to_cv2(seg_result.scene_depth_img)
    cv2.imshow('saved depth_img',depth_img)
    cv2.waitKey()
