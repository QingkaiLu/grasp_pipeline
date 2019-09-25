#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
#from grasp_pipeline import srv
from grasp_pipeline.srv import *
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, \
        PointStamped, Vector3Stamped
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs import point_cloud2
import tf
import numpy as np
import random
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import copy

def sqr_dist(x, y):
    '''
    Get the squared distance between two vectors
    '''
    xx = x-y
    xx = np.multiply(xx,xx)
    xx = np.sum(xx)
    return xx

def find_nearest_neighbor(sample_point, cloud):
    '''
    Find the nearest neigbor between sample_point and the points in cloud.
    Uses the squared Euclidean distance.
    '''
    min_dist = 10000
    min_pt = None
    for idx, pt in enumerate(cloud):
        pt_dist = sqr_dist(np.array(pt), sample_point)
        if pt_dist < min_dist:
            min_pt = np.array(pt[:])
            min_dist = pt_dist
            min_idx = idx;
    return min_pt, min_idx, min_dist

def publish_points(grasp_pose_pub, points_stamped, 
        color=(1., 0., 0.)):
    #grasp_pose_pub=rospy.Publisher(publisher_name, MarkerArray, queue_size=1)
    markerArray = MarkerArray()
    for i, pnt in enumerate(points_stamped):
        marker = Marker()
        marker.header.frame_id = pnt.header.frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        marker.pose.position.x = pnt.point.x
        marker.pose.position.y = pnt.point.y
        marker.pose.position.z = pnt.point.z
        marker.id = i
        markerArray.markers.append(marker)
    grasp_pose_pub.publish(markerArray)

class BoundingBoxFace():
    '''
    Simple class to store properties of the face of 3D bounding box
    '''
    def __init__(self, center, orientation_a, orientation_b, height, width,
                 is_top=False):
        self.center = center
        self.orientation_a = orientation_a
        self.orientation_b = orientation_b
        self.height = height
        self.width = width
        self.is_top = is_top


class GenGraspPreshape():
    def __init__(self):
        rospy.init_node('gen_grasp_preshape_server')
        # Read parameters from server
        self.use_sim = rospy.get_param('~use_sim', False)
        self.use_bb_orientation = rospy.get_param('~use_bb_orientation',True)
        #self.hand_sample_dist = rospy.get_param('~hand_sample_dist', 0.03)
        # Due to gravity, the hand will be lower than the goal pose for top grasps in simulation. 
        # the top grasp need a further distance from the object.
        self.hand_sample_dist_top = rospy.get_param('~hand_sample_dist_top', 0.06)
        self.hand_sample_dist_side = rospy.get_param('~hand_sample_dist_side', 0.03)
        # NOTE: Set to 0.0 to turn off sampling
        self.hand_dist_var = rospy.get_param('~hand_sample_dist_var', 0.001)
        self.palm_position_var = rospy.get_param('~palm_position_sample_var', 0.001)
        self.palm_ort_var = rospy.get_param('~palm_ort_sample_var', 0.001)
        self.roll_angle_sample_var = rospy.get_param('~/hand_roll_angle_sample_var', 0.001)
        self.setup_joint_angle_limits()
        self.listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()
        self.service_is_called = False
        self.object_pose = PoseStamped()
        self.palm_goal_pose_world = [] 
        # The minimum distance from the object top to the table so that the hand can reach the grasp 
        # preshape without colliding the table.
        self.min_object_top_dist_to_table = rospy.get_param('~min_object_top_dist_to_table', 0.03)
        self.samples_num_per_preshape = 50
        self.exp_palm_pose = None
        self.grasp_pose_pub = rospy.Publisher('/publish_box_points', MarkerArray, queue_size=1)


    def setup_joint_angle_limits(self):
        '''
        Initializes a number of constants determing the joint limits for allegro
        TODO: Automate this by using a URDF file and allow hand to be specified at launch
        '''
        self.index_joint_0_lower = -0.59
        self.index_joint_0_upper = 0.57
        self.middle_joint_0_lower = -0.59
        self.middle_joint_0_upper = 0.57
        self.ring_joint_0_lower = -0.59
        self.ring_joint_0_upper = 0.57

        self.index_joint_1_lower = -0.296
        self.index_joint_1_upper = 0.71
        self.middle_joint_1_lower = -0.296
        self.middle_joint_1_upper = 0.71
        self.ring_joint_1_lower = -0.296
        self.ring_joint_1_upper = 0.71

        self.thumb_joint_0_lower = 0.363
        self.thumb_joint_0_upper = 1.55
        self.thumb_joint_1_lower = -0.205
        self.thumb_joint_1_upper = 1.263

        self.index_joint_0_middle = (self.index_joint_0_lower + self.index_joint_0_upper) * 0.5
        self.middle_joint_0_middle = (self.middle_joint_0_lower + self.middle_joint_0_upper) * 0.5
        self.ring_joint_0_middle = (self.ring_joint_0_lower + self.ring_joint_0_upper) * 0.5
        self.index_joint_1_middle = (self.index_joint_1_lower + self.index_joint_1_upper) * 0.5
        self.middle_joint_1_middle = (self.middle_joint_1_lower + self.middle_joint_1_upper) * 0.5
        self.ring_joint_1_middle = (self.ring_joint_1_lower + self.ring_joint_1_upper) * 0.5
        self.thumb_joint_0_middle = (self.thumb_joint_0_lower + self.thumb_joint_0_upper) * 0.5
        self.thumb_joint_1_middle = (self.thumb_joint_1_lower + self.thumb_joint_1_upper) * 0.5

        self.index_joint_0_range = self.index_joint_0_upper - self.index_joint_0_lower
        self.middle_joint_0_range = self.middle_joint_0_upper - self.middle_joint_0_lower
        self.ring_joint_0_range = self.ring_joint_0_upper - self.ring_joint_0_lower
        self.index_joint_1_range = self.index_joint_1_upper - self.index_joint_1_lower
        self.middle_joint_1_range = self.middle_joint_1_upper - self.middle_joint_1_lower
        self.ring_joint_1_range = self.ring_joint_1_upper - self.ring_joint_1_lower
        self.thumb_joint_0_range = self.thumb_joint_0_upper - self.thumb_joint_0_lower
        self.thumb_joint_1_range = self.thumb_joint_1_upper - self.thumb_joint_1_lower

        self.first_joint_lower_limit = 0.25
        self.first_joint_upper_limit = 0.25
        self.second_joint_lower_limit = 0.5
        self.second_joint_upper_limit = 0. #-0.1

        self.thumb_1st_joint_lower_limit = -0.5
        self.thumb_1st_joint_upper_limit = 0.5
        self.thumb_2nd_joint_lower_limit = 0.25
        self.thumb_2nd_joint_upper_limit = 0.25

        self.index_joint_0_sample_lower = self.index_joint_0_middle - self.first_joint_lower_limit * self.index_joint_0_range
        self.index_joint_0_sample_upper = self.index_joint_0_middle + self.first_joint_upper_limit * self.index_joint_0_range
        self.middle_joint_0_sample_lower = self.middle_joint_0_middle - self.first_joint_lower_limit * self.middle_joint_0_range
        self.middle_joint_0_sample_upper = self.middle_joint_0_middle + self.first_joint_upper_limit * self.middle_joint_0_range
        self.ring_joint_0_sample_lower = self.ring_joint_0_middle - self.first_joint_lower_limit * self.ring_joint_0_range
        self.ring_joint_0_sample_upper = self.ring_joint_0_middle + self.first_joint_upper_limit * self.ring_joint_0_range

        self.index_joint_1_sample_lower = self.index_joint_1_middle - self.second_joint_lower_limit * self.index_joint_1_range
        self.index_joint_1_sample_upper = self.index_joint_1_middle + self.second_joint_upper_limit * self.index_joint_1_range
        self.middle_joint_1_sample_lower = self.middle_joint_1_middle - self.second_joint_lower_limit * self.middle_joint_1_range
        self.middle_joint_1_sample_upper = self.middle_joint_1_middle + self.second_joint_upper_limit * self.middle_joint_1_range
        self.ring_joint_1_sample_lower = self.ring_joint_1_middle - self.second_joint_lower_limit * self.ring_joint_1_range
        self.ring_joint_1_sample_upper = self.ring_joint_1_middle + self.second_joint_upper_limit * self.ring_joint_1_range

        self.thumb_joint_0_sample_lower = self.thumb_joint_0_middle - self.thumb_1st_joint_lower_limit * self.thumb_joint_0_range
        self.thumb_joint_0_sample_upper = self.thumb_joint_0_middle + self.thumb_1st_joint_upper_limit * self.thumb_joint_0_range
        self.thumb_joint_1_sample_lower = self.thumb_joint_1_middle - self.thumb_2nd_joint_lower_limit * self.thumb_joint_1_range
        self.thumb_joint_1_sample_upper = self.thumb_joint_1_middle + self.thumb_2nd_joint_upper_limit * self.thumb_joint_1_range

    def handle_gen_grasp_preshape(self, req):
        if req.sample:
            return self.sample_grasp_preshape(req)
        else:
            return self.gen_grasp_preshape(req)

    def gen_grasp_preshape(self, req):
        '''
        Grasp preshape service callback for generating a preshape from real perceptual
        data.
        '''
        response = GraspPreshapeResponse()

        self.object_pose.header.frame_id = req.obj.cloud.header.frame_id
        self.object_pose.pose = req.obj.pose
        self.service_is_called = True

        bb_center_points, bb_faces = self.get_bb_faces(req.obj) 
        self.palm_goal_pose_world = []
        for i in xrange(len(bb_faces)):
            # Get desired palm pose given the point from the bounding box
            palm_pose_world = self.find_palm_pose(bb_center_points[i], req.obj, bb_faces[i])
            self.palm_goal_pose_world.append(palm_pose_world)
            response.palm_goal_pose_world.append(palm_pose_world)
            # Sample remaining joint values
            allegro_js = self.gen_allegro_random_preshape_js()
            response.allegro_joint_state.append(allegro_js) 
            response.is_top_grasp.append(bb_faces[i].is_top)

        response.object_pose = self.object_pose

        return response

    def sample_grasp_preshape(self, req):
        '''
        Grasp preshape service callback for sampling grasp preshapes.
        '''
        response = GraspPreshapeResponse()
    
        bb_center_points, bb_faces = self.get_bb_faces(req.obj) 
        self.palm_goal_pose_world = []
        for i in xrange(len(bb_faces)):
            # Get desired palm pose given the point from the bounding box
            palm_pose_world = self.find_palm_pose(bb_center_points[i], req.obj, bb_faces[i])
            self.palm_goal_pose_world.append(palm_pose_world)

            self.set_palm_rand_pose_limits(palm_pose_world)
            for j in xrange(self.samples_num_per_preshape):
                sample_palm_pose = self.sample_palm_rand_pose(palm_pose_world.header.frame_id)
                response.palm_goal_pose_world.append(sample_palm_pose)
                # Sample remaining joint values
                allegro_js = self.gen_allegro_random_preshape_js()
                response.allegro_joint_state.append(allegro_js) 
                response.is_top_grasp.append(bb_faces[i].is_top)

        self.object_pose.header.frame_id = req.obj.cloud.header.frame_id
        self.object_pose.pose = req.obj.pose
        response.object_pose = self.object_pose

        self.service_is_called = True
        return response


    def broadcast_palm_and_obj(self):
        #if self.service_is_called and not self.use_sim:
        if self.service_is_called:
            # Publish the object tf
            self.tf_br.sendTransform((self.object_pose.pose.position.x, self.object_pose.pose.position.y, 
                    self.object_pose.pose.position.z),
                    (self.object_pose.pose.orientation.x, self.object_pose.pose.orientation.y, 
                    self.object_pose.pose.orientation.z, self.object_pose.pose.orientation.w),
                    rospy.Time.now(), 'object_pose', self.object_pose.header.frame_id)

            # Publish the palm goal tf
            for i, palm_pose_world in enumerate(self.palm_goal_pose_world):
                self.tf_br.sendTransform((palm_pose_world.pose.position.x, palm_pose_world.pose.position.y, 
                        palm_pose_world.pose.position.z),
                        (palm_pose_world.pose.orientation.x, palm_pose_world.pose.orientation.y, 
                        palm_pose_world.pose.orientation.z, palm_pose_world.pose.orientation.w),
                        rospy.Time.now(), 'heu_' + str(i), palm_pose_world.header.frame_id)

            # Broadcast experiment grasp poses
            if self.exp_palm_pose is not None:
                for i, exp_pose in enumerate(self.exp_palm_pose):
                    self.tf_br.sendTransform((exp_pose.pose.position.x, exp_pose.pose.position.y, 
                            exp_pose.pose.position.z),
                            (exp_pose.pose.orientation.x, exp_pose.pose.orientation.y, 
                            exp_pose.pose.orientation.z, exp_pose.pose.orientation.w),
                            rospy.Time.now(), 'exp_' + str(i), exp_pose.header.frame_id)


    def get_bb_faces(self, grasp_object):
        '''
        Computes and return the centers of 3 bounding box sides: top face, two side faces 
        closer to the camera.
        '''
        homo_matrix_world_frame = self.listener.fromTranslationRotation(
            (.0, .0, .0),
            (grasp_object.pose.orientation.x, grasp_object.pose.orientation.y,
             grasp_object.pose.orientation.z, grasp_object.pose.orientation.w)
        )
        x_axis_world_frame = homo_matrix_world_frame[:3, 0]
        y_axis_world_frame = homo_matrix_world_frame[:3, 1]
        z_axis_world_frame = homo_matrix_world_frame[:3, 2]
        bb_center_world_frame = np.array([grasp_object.pose.position.x,
                                           grasp_object.pose.position.y,
                                           grasp_object.pose.position.z])
        # Append all faces of the bounding box except the back face.
        half_width = 0.5 * grasp_object.width
        half_height = 0.5 * grasp_object.height
        half_depth = 0.5 * grasp_object.depth

        faces_world_frame = [BoundingBoxFace(bb_center_world_frame +
                                              half_width * x_axis_world_frame,
                                              y_axis_world_frame,
                                              z_axis_world_frame,
                                              grasp_object.height,
                                              grasp_object.depth),
                              BoundingBoxFace(bb_center_world_frame -
                                              half_width * x_axis_world_frame,
                                              y_axis_world_frame,
                                              z_axis_world_frame,
                                              grasp_object.height,
                                              grasp_object.depth),
                              BoundingBoxFace(bb_center_world_frame +
                                              half_height * y_axis_world_frame,
                                              x_axis_world_frame,
                                              z_axis_world_frame,
                                              grasp_object.width,
                                              grasp_object.depth),
                              BoundingBoxFace(bb_center_world_frame -
                                              half_height * y_axis_world_frame,
                                              x_axis_world_frame,
                                              z_axis_world_frame,
                                              grasp_object.width,
                                              grasp_object.depth),
                              BoundingBoxFace(bb_center_world_frame +
                                              half_depth * z_axis_world_frame,
                                              x_axis_world_frame,
                                              y_axis_world_frame,
                                              grasp_object.width,
                                              grasp_object.height),
                              BoundingBoxFace(bb_center_world_frame -
                                              half_depth * z_axis_world_frame,
                                              x_axis_world_frame,
                                              y_axis_world_frame,
                                              grasp_object.width,
                                              grasp_object.height)]

        faces_world_frame = sorted(faces_world_frame, key=lambda x: x.center[2])
        # Assign the top face
        faces_world_frame[-1].is_top = True
        min_z_world = faces_world_frame[0].center[2]
        max_z_world = faces_world_frame[-1].center[2]
        # Delete the bottom face
        del faces_world_frame[0]

        # Sort along x axis and delete the face furthest from camera.
        faces_world_frame = sorted(faces_world_frame, key=lambda x: x.center[0])
        faces_world_frame = faces_world_frame[:-1]
        # Sort along y axis and delete the face furthest from the robot. 
        faces_world_frame = sorted(faces_world_frame, key=lambda x: x.center[1])
        faces_world_frame = faces_world_frame[1:]
        face_centers_world_frame = []
        center_stamped_world = PointStamped()
        center_stamped_world.header.frame_id = grasp_object.header.frame_id
        for i, face in enumerate(faces_world_frame):
            center_stamped_world.point.x = face.center[0]
            center_stamped_world.point.y = face.center[1]
            center_stamped_world.point.z = face.center[2]
            face_centers_world_frame.append(copy.deepcopy(center_stamped_world))

        publish_points(self.grasp_pose_pub, face_centers_world_frame)

        # If the object is too short, only select top grasps.
        obj_height = max_z_world - min_z_world
        rospy.loginfo('##########################')
        rospy.loginfo('Obj_height: %s' %obj_height)
        if obj_height < self.min_object_top_dist_to_table:
            rospy.loginfo('Object is short, only use top grasps!')
            return [face_centers_world_frame[0]], [faces_world_frame[0]]

        return face_centers_world_frame, faces_world_frame

    def sample_from_bounding_box(self, grasp_object):
        '''
        Computes the centers of the bounding box sides and returns a sample point close
        to one of these.
        '''
        face_centers_world_frame, faces_world_frame = self.get_bb_faces(grasp_object)
        rand_idx = np.random.randint(len(face_centers_world_frame))
        rospy.loginfo('Selected random index = ' + str(rand_idx))

        return face_centers_world_frame[rand_idx], faces_world_frame[rand_idx]

    def find_palm_pose(self, object_point_stamped, obj, bb_face):
        '''
        Determine the desired palm pose given a point close to the object and the
        object point cloud. Finds the point on the point cloud closest to the desired
        point and generates a palm pose given the point's surface normal.
        '''
        object_point = np.array([object_point_stamped.point.x, object_point_stamped.point.y, object_point_stamped.point.z])
        if not bb_face.is_top:
            # Find nearest neighbor
            points_xyz = point_cloud2.read_points(obj.cloud,field_names=['x','y','z'])
            closest_pt, min_idx, min_dist = find_nearest_neighbor(object_point, points_xyz)
            rospy.loginfo('Found closest point ' + str(closest_pt) + ' to obj_pt = ' +
                          str(object_point) + ' at dist = ' + str(min_dist))

            # Get associated normal
            point_normals = point_cloud2.read_points(obj.normals, field_names=['normal_x',
                                                                           'normal_y',
                                                                           'normal_z'])
            for i, normal in enumerate(point_normals):
                if i == min_idx:
                    obj_normal = np.array(normal[:])
                    break
            rospy.loginfo('Associated normal n = ' + str(obj_normal))
            hand_dist = self.hand_sample_dist_side
            vec_center_to_face = object_point - np.array([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z])
            if np.dot(obj_normal, vec_center_to_face) < 0.:
                obj_normal = -obj_normal
        else:
            closest_pt = object_point
            obj_normal = closest_pt - np.array([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z])
            obj_normal /= np.linalg.norm(obj_normal)
            hand_dist = self.hand_sample_dist_top

        # Add Gaussian noise to the palm position
        if self.hand_dist_var > 0.:
            hand_dist += np.random.normal(0., self.hand_dist_var)
        palm_position = closest_pt + hand_dist*obj_normal
        if self.palm_position_var > 0.:
            palm_position += np.random.normal(0., self.palm_position_var, 3)

        palm_pose = PoseStamped()
        palm_pose.header.frame_id = obj.cloud.header.frame_id
        palm_pose.header.stamp = rospy.Time.now()
        palm_pose.pose.position.x = palm_position[0]
        palm_pose.pose.position.y = palm_position[1]
        palm_pose.pose.position.z = palm_position[2]
        rospy.loginfo('Chosen palm position is ' + str(palm_pose.pose.position))

        # Sample from a Gaussian around the bounding box face orientation
        # by an interpolation between two orientations with a random ratio
        #rand_interpolation_two_ort_vec = np.random.normal(0.0, self.roll_angle_sample_var)
        #rand_interpolation_two_ort_vec = np.random.uniform(0.5, 1.0)
        #rand_interpolation_two_ort_vec = np.random.uniform(0., 0.2)

        # Get orientation vector from bounding box face
        if bb_face.is_top:
            rospy.loginfo('***Top.')
            # Determine palm orientation from 4 possible bounding box sides
            # Negative sign is decided by observing the pca axis direction
            # of the segmentaion object cloud.
            bb_orientation_vec_a = -np.array(bb_face.orientation_a)
            bb_orientation_vec_b = -np.array(bb_face.orientation_b)

            ## Flip a coin between a and b
            #if np.random.randint(2):
            #    bb_orientation_vec = bb_orientation_vec_a[:] + \
            #            rand_interpolation_two_ort_vec * bb_orientation_vec_b[:]  
            #else:
            #    bb_orientation_vec = bb_orientation_vec_b[:] + \
            #            rand_interpolation_two_ort_vec * bb_orientation_vec_a[:]  

            bb_orientation_vec = bb_orientation_vec_a[:] + bb_orientation_vec_b[:]  

            # Thumb always point to x to make sure it's more likely to be reachable.
            if bb_orientation_vec[0] < 0:
                bb_orientation_vec = -bb_orientation_vec

        else:
            rospy.loginfo('***Not top.')
            # TODO: Remove the orientation that runs into the table
            # Determine palm orientation from 4 possible bounding box sides
            bb_orientation_vec_a = -np.array(bb_face.orientation_a)
            bb_orientation_vec_b = -np.array(bb_face.orientation_b)

            # Pick the vertical vector with larger z value to be the palm link y axis (thumb) 
            # in order to remove the orientation that runs into the table
            if abs(bb_orientation_vec_a[2]) > abs(bb_orientation_vec_b[2]):
                #bb_orientation_vec = bb_orientation_vec_a[:] + \
                #        rand_interpolation_two_ort_vec * bb_orientation_vec_b[:]  
                bb_orientation_vec = bb_orientation_vec_a[:]
            else:
                #bb_orientation_vec = bb_orientation_vec_b[:] + \
                #        rand_interpolation_two_ort_vec * bb_orientation_vec_a[:]  
                bb_orientation_vec = bb_orientation_vec_b[:]

            # Thumb always point up to avoid collision with the table.
            if bb_orientation_vec[2] < 0:
                bb_orientation_vec = -bb_orientation_vec

        if self.palm_ort_var > 0.:
           bb_orientation_vec += np.random.normal(0., self.palm_ort_var, 3)
        bb_orientation_vec /= np.linalg.norm(bb_orientation_vec)
       
        q = self.sample_palm_orientation(obj_normal, bb_orientation_vec)
        palm_pose.pose.orientation.x = q[0]
        palm_pose.pose.orientation.y = q[1]
        palm_pose.pose.orientation.z = q[2]
        palm_pose.pose.orientation.w = q[3]
        return palm_pose

    def sample_palm_orientation(self, obj_normal, bb_orientation_vec):
        '''
        Sample the hand wrist roll. Currently it is uniform.
        For the palm frame, x: palm normal, y: thumb, z: middle finger.
        '''
        if self.use_bb_orientation:
            y_rand = bb_orientation_vec
        else:
            roll = np.random.uniform(-np.pi, np.pi)
            y_rand = np.array([0.0, np.sin(roll), np.cos(roll)])

        # Project orientation vector into the tangent space of the normal
        # NOTE: assumes normal is a unit vector
        x_axis = -obj_normal
        y_onto_x = y_rand.dot(x_axis)*x_axis
        y_axis = y_rand - y_onto_x
        # Normalize to unit length
        y_axis /= np.linalg.norm(y_axis)
        # Find the third orthonormal component and build the rotation matrix
        z_axis = np.cross(x_axis, y_axis)
        z_axis /= np.linalg.norm(z_axis)
        rot_matrix = np.matrix([x_axis, y_axis, z_axis]).T

        # Compute quaternion from rpy
        trans_matrix = np.matrix(np.zeros((4,4)))
        trans_matrix[:3,:3] = rot_matrix
        trans_matrix[3,3] = 1.
        quaternion = tf.transformations.quaternion_from_matrix(trans_matrix)
        rospy.loginfo('Grasp orientation = ' +  str(quaternion))
        return quaternion

    def create_preshape_server(self):
        '''
        Create the appropriate simulated or real service callback based on parameter
        setting
        '''
        self.preshape_service = rospy.Service('gen_grasp_preshape', GraspPreshape,
                                              self.handle_gen_grasp_preshape)
        rospy.loginfo('Service gen_grasp_preshape:')
        rospy.loginfo('Ready to generate the grasp preshape.')
        #rospy.spin()

    def update_detection_grasp_server(self):
        '''
        Update grasp preshape list including detection (sampling, inference) 
        to publish the palm_pose tf of both detection preshape and segmentation preshape.
        '''
        self.preshape_service = rospy.Service('update_detection_grasp_preshape', UpdateInfPreshape,
                                              self.handle_update_detection_preshape)
        rospy.loginfo('Service update_detection_grasp_server:')
        rospy.loginfo('Ready to update the detection grasp preshape.')

    def handle_update_detection_preshape(self, req):
        self.exp_palm_pose = req.exp_palm_poses
        response = UpdateInfPreshapeResponse()
        response.success = True
        return response

    def gen_allegro_random_preshape_js(self):
        '''
        Generate a random preshape for the hand joints
        '''
        hand_joint_state = JointState()
        hand_joint_state.name = ['index_joint_0','index_joint_1','index_joint_2', 'index_joint_3',
                   'middle_joint_0','middle_joint_1','middle_joint_2', 'middle_joint_3',
                   'ring_joint_0','ring_joint_1','ring_joint_2', 'ring_joint_3',
                   'thumb_joint_0','thumb_joint_1','thumb_joint_2', 'thumb_joint_3']
        js_position = np.zeros(16)
        js_position[0] = np.random.uniform(self.index_joint_0_sample_lower, self.index_joint_0_sample_upper)
        js_position[1] = np.random.uniform(self.index_joint_1_sample_lower, self.index_joint_1_sample_upper)
        js_position[4] = np.random.uniform(self.middle_joint_0_sample_lower, self.middle_joint_0_sample_upper)
        js_position[5] = np.random.uniform(self.middle_joint_1_sample_lower, self.middle_joint_1_sample_upper)
        js_position[8] = np.random.uniform(self.ring_joint_0_sample_lower, self.ring_joint_0_sample_upper)
        js_position[9] = np.random.uniform(self.ring_joint_1_sample_lower, self.ring_joint_1_sample_upper)
        js_position[12] = np.random.uniform(self.thumb_joint_0_sample_lower, self.thumb_joint_0_sample_upper)
        js_position[13] = np.random.uniform(self.thumb_joint_1_sample_lower, self.thumb_joint_1_sample_upper)
        hand_joint_state.position = js_position.tolist()
        rospy.loginfo('Random joint states of the hand preshape: %s'%str(hand_joint_state.position))
        return hand_joint_state

    def set_palm_rand_pose_limits(self, palm_preshape_pose):
        '''
        Set the palm pose sample range for sampling grasp detection.
        '''
        palm_preshape_quaternion = (palm_preshape_pose.pose.orientation.x, palm_preshape_pose.pose.orientation.y,
                            palm_preshape_pose.pose.orientation.z, palm_preshape_pose.pose.orientation.w) 
        palm_preshape_euler = tf.transformations.euler_from_quaternion(palm_preshape_quaternion) 

        preshape_palm_pose_config = [palm_preshape_pose.pose.position.x, palm_preshape_pose.pose.position.y,
                palm_preshape_pose.pose.position.z, palm_preshape_euler[0], palm_preshape_euler[1], 
                palm_preshape_euler[2]]
        preshape_palm_pose_config = np.array(preshape_palm_pose_config)

        pos_range = 0.05
        ort_range = 0.05 * np.pi
        lower_limit_range = -np.array([pos_range, pos_range, pos_range, ort_range, ort_range, ort_range])
        upper_limit_range = np.array([pos_range, pos_range, pos_range, ort_range, ort_range, ort_range])
        self.palm_pose_lower_limit = preshape_palm_pose_config + lower_limit_range
        self.palm_pose_upper_limit = preshape_palm_pose_config + upper_limit_range

    def sample_palm_rand_pose(self, frame_id):
        '''
        Get a random palm pose by sampling around the preshape palm pose
        for sampling grasp detection. 
        '''
        sample_palm_pose_array = np.random.uniform(self.palm_pose_lower_limit, self.palm_pose_upper_limit)
        sample_palm_pose = PoseStamped()
        sample_palm_pose.header.frame_id = frame_id
        sample_palm_pose.pose.position.x, sample_palm_pose.pose.position.y, \
                sample_palm_pose.pose.position.z = sample_palm_pose_array[:3] 
    
        palm_euler = sample_palm_pose_array[3:] 
        palm_quaternion = tf.transformations.quaternion_from_euler(palm_euler[0], palm_euler[1], palm_euler[2])
        sample_palm_pose.pose.orientation.x, sample_palm_pose.pose.orientation.y, \
                sample_palm_pose.pose.orientation.z, sample_palm_pose.pose.orientation.w = palm_quaternion 
        
        return sample_palm_pose


if __name__ == '__main__':
    gen_grasp_preshape = GenGraspPreshape()
    gen_grasp_preshape.create_preshape_server()
    # gen_grasp_preshape.update_detection_grasp_server()
    # rate = rospy.Rate(100)
    # while not rospy.is_shutdown():
    #     gen_grasp_preshape.broadcast_palm_and_obj()
    #     rate.sleep()
    rospy.spin()
