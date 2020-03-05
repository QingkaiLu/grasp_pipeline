#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from grasp_pipeline.srv import *
from point_cloud_segmentation.srv import *

if __name__ == '__main__':
    rospy.init_node('preshape_test_client')
    seg_proxy = rospy.ServiceProxy('/object_segmenter', SegmentGraspObject)
    preshape_proxy = rospy.ServiceProxy('/gen_grasp_preshape', GraspPreshape)

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

    rospy.loginfo('Requesting preshape generation')
    # Convert seg_result into preshape request
    preshape_req = GraspPreshapeRequest()
    preshape_req.obj = seg_result.obj

    try:
        preshape_response = preshape_proxy(preshape_req)
    except rospy.ServiceException as e:
        rospy.logwarn("Preshape service did not process request: %s"%str(e))
        exit()

    rospy.loginfo('Preshape response: ' + str(preshape_response))
