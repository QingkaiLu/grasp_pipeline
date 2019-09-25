#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import ModelStates


def broadcast_kinect_gazebo_tf(kinect_pose):
    tf_br = tf.TransformBroadcaster()
    tf_br.sendTransform((kinect_pose.position.x,
                        kinect_pose.position.y,
                        kinect_pose.position.z),
                        (kinect_pose.orientation.x,
                        kinect_pose.orientation.y,
                        kinect_pose.orientation.z,
                        kinect_pose.orientation.w),
                        rospy.Time.now(), 'kinect_gazebo', 'world')


def broadcast_kinect_pointcloud_tf():
    tf_br = tf.TransformBroadcaster()
    tf_br.sendTransform((0, 0, 0),
                        (0.5, -0.5, 0.5, -0.5),
                        rospy.Time.now(), 'kinect_pointcloud', 'kinect_gazebo')


if __name__ == '__main__':
    rospy.init_node('kinect_gazebo_tf_br')
    gazebo_model_states = \
        rospy.wait_for_message('/gazebo/model_states', ModelStates)
    for i, ms_name in enumerate(gazebo_model_states.name):
        if ms_name == 'kinect_ros':
            kinect_pose = gazebo_model_states.pose[i]
            break
    while not rospy.is_shutdown():
        broadcast_kinect_gazebo_tf(kinect_pose)
        broadcast_kinect_pointcloud_tf()
        # rospy.sleep(1)
