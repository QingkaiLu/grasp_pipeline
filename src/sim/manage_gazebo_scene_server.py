#!/usr/bin/env python
import os
import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from grasp_pipeline.srv import *

class ManageSceneInGazebo:
    def __init__(self):
        rospy.init_node('manage_gazebo_scene_node')
        self.save_urdf_path = rospy.get_param('~save_urdf_path', '')
        self.object_meshes_path = rospy.get_param('~object_meshes_path', '')
        self.cur_urdf_path = None
        self.moved_away_objects_num = 0
        self.move_away_obj_x_dist = 10.
        self.move_away_obj_y_loc = 1000.
        self.prev_obj_model_name = None


    def move_object(self, object_model_name, object_pose_quart_array):
        """
            Move object to a certain pose (Quaternion for orientation).
        """
        move_obj_cmd = "rosservice call /gazebo/set_model_state " + "'{model_state: { model_name: " + object_model_name + \
                            ", pose: { position: { x: " + str(object_pose_quart_array[4]) + ", y: " + str(object_pose_quart_array[5]) + \
                            ", z: "+ str(object_pose_quart_array[6]) + " }, orientation: {x: " + str(object_pose_quart_array[0]) + \
                            ", y: " + str(object_pose_quart_array[1]) + ", z: " + str(object_pose_quart_array[2]) + ", w: " + \
                            str(object_pose_quart_array[3]) + " } }, reference_frame: world } }'"
        print 'move_obj_cmd:', move_obj_cmd
        os.system(move_obj_cmd)
        # sleep for 0.5 second to make sure the object is updated for the gazebo camera
        rospy.sleep(0.5)


    def move_away_prev_object(self):
        """
            Move the previous object to somewhere further away. This is to get around the issue that
            dart would crash after deleting objects.
        """
        if self.prev_obj_model_name is None:
            return
        object_away_x_loc = self.move_away_obj_x_dist * self.moved_away_objects_num
        object_pose_quart_array = [0., 0., 0., 1., object_away_x_loc, self.move_away_obj_y_loc, 0]
        self.move_object(self.prev_obj_model_name, object_pose_quart_array)
        self.moved_away_objects_num += 1
        

    def delete_object(self, object_model_name):
        delete_obj_cmd = 'rosservice call gazebo/delete_model ' + object_model_name 
        print 'delete_obj_cmd:', delete_obj_cmd
        os.system(delete_obj_cmd)


    def delete_prev_object(self):
        """
            Delete the previous object model.
        """
        if self.prev_obj_model_name is None:
            return
        self.delete_object(self.prev_obj_model_name)


    def spawn_object(self, object_name, obj_model_name, object_pose_array):
        '''
            Spawn the generated object urdf with the given object model name.
        '''
        self.cur_urdf_path = self.save_urdf_path + '/' + object_name + '.urdf'
        spawn_object_cmd = "rosrun gazebo_ros spawn_model -file " + self.cur_urdf_path + " -urdf -x " + str(object_pose_array[3]) +\
                " -y "+ str(object_pose_array[4]) + " -z " + str(object_pose_array[5]) + " -model " + obj_model_name 
        print 'spawn_object_cmd:', spawn_object_cmd
        os.system(spawn_object_cmd)
        self.prev_obj_model_name = obj_model_name
    

    def handle_update_gazebo_object(self, req):
        '''
            Gazebo scene management server handler.
            Generate the urdf for the given object mesh, spawn the urdf and delete the previous object.
            parameter: req.object_pose = [r, p, y, x, y, z]
        '''
        #self.move_away_prev_object()
        self.delete_prev_object()
        #mesh_pose_array = [0., 0., 0., 0., 0., 0.]
        #self.generate_urdf(req.object_name, mesh_pose_array)
        self.spawn_object(req.object_name, req.object_model_name, req.object_pose_array)

        response = UpdateObjectGazeboResponse()
        response.success = True
        return response

    def update_gazebo_object_server(self):
        rospy.Service('update_gazebo_object', UpdateObjectGazebo, self.handle_update_gazebo_object)
        rospy.loginfo('Service update_gazebo_object:')
        rospy.loginfo('Ready to update the object in gazebo scene.')

    def handle_move_gazebo_object(self, req):
        '''
            Move object to a new pose.
        '''
        object_pose_quart_array = [req.object_pose_stamped.pose.orientation.x, req.object_pose_stamped.pose.orientation.y,
                                    req.object_pose_stamped.pose.orientation.z, req.object_pose_stamped.pose.orientation.w,
                                    req.object_pose_stamped.pose.position.x, req.object_pose_stamped.pose.position.y,
                                    req.object_pose_stamped.pose.position.z]
        self.move_object(req.object_model_name, object_pose_quart_array)
        response = MoveObjectGazeboResponse()
        response.success = True
        return response

    def move_gazebo_object_server(self):
        rospy.Service('move_gazebo_object', MoveObjectGazebo, self.handle_move_gazebo_object)
        rospy.loginfo('Service move_gazebo_object:')
        rospy.loginfo('Ready to move the object in gazebo scene.')



if __name__=='__main__':
    #manage_gazebo_scene = ManageGazeboScene(save_urdf_path)
    ##object_name = '3m_high_tack_spray_adhesive' 
    #object_name = 'campbells_soup_at_hand_creamy_tomato'
    #object_pose = [0.] * 6
    #obj_model_name = object_name + '_1'
    #manage_gazebo_scene.generate_urdf(mesh_path, object_name, object_pose)
    #manage_gazebo_scene.spawn_object(obj_model_name)
    #manage_gazebo_scene.move_away_prev_object()
    #obj_model_name = object_name + '_2'
    #manage_gazebo_scene.spawn_object(obj_model_name)
    #manage_gazebo_scene.move_away_prev_object()
    #obj_model_name = object_name + '_3'
    #manage_gazebo_scene.spawn_object(obj_model_name)
    manage_gazebo_scene = ManageSceneInGazebo()
    manage_gazebo_scene.update_gazebo_object_server()
    manage_gazebo_scene.move_gazebo_object_server()
    rospy.spin()



