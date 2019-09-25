#!/usr/bin/env python

import roslib; roslib.load_manifest('grasp_pipeline')
import rospy
from grasp_pipeline.srv import *
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
#from std_msgs.msg import Char
from sensor_msgs.msg import JointState
import moveit_msgs.msg
import moveit_commander
#import geometry_msgs.msg
#from geometry_msgs.msg import PoseStamped,Pose
import numpy as np
import copy

class CartesianPoseMoveitPlanner:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_goal_pose_planner_node')
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('kuka_arm')

    def go_home(self):
        #print 'go home'
        self.group.clear_pose_targets()
        self.group.set_joint_value_target(np.zeros(7))
        plan_home = self.group.plan()
        return plan_home

    def go_goal(self, pose):
        #print 'go goal'
        self.group.clear_pose_targets()
        self.group.set_pose_target(pose)
        plan_goal = self.group.plan()
        return plan_goal

    def lift_up_way_points(self, cur_pose, height=0.3):
        print 'Lift up with way points.'
        #self.group.clear_pose_targets()
        #self.group.set_pose_target(pose)
        way_points = []
        #start with the current pose
        #cur_pose = self.group.get_current_pose().pose
        way_points.append(copy.deepcopy(cur_pose))
        lift_steps = 20
        height_step = height / float(lift_steps)
        for i in xrange(lift_steps):
            cur_pose.position.z += height_step 
            way_points.append(copy.deepcopy(cur_pose)) 
        print way_points
        (plan_way_points, fraction) = self.group.compute_cartesian_path(
                way_points,   # waypoints to follow
                0.01,        # eef_step
                0.0)         # jump_threshold
        print plan_way_points
        return plan_way_points

    def handle_pose_goal_planner(self, req):
        plan = None
        if req.go_home:
            plan = self.go_home()
        elif req.lift_way_points:
            plan = self.lift_up_way_points(req.palm_goal_pose_world, req.lift_height)
        else:
            plan = self.go_goal(req.palm_goal_pose_world)
        #print plan
        response = PalmGoalPoseWorldResponse()
        response.success = False
        if len(plan.joint_trajectory.points) > 0:
            response.success = True
            response.plan_traj = plan.joint_trajectory
        return response

    def create_moveit_planner_server(self):
        rospy.Service('moveit_cartesian_pose_planner', PalmGoalPoseWorld, self.handle_pose_goal_planner)
        rospy.loginfo('Service moveit_cartesian_pose_planner:')
        rospy.loginfo('Reference frame: %s' %self.group.get_planning_frame())
        rospy.loginfo('End-effector frame: %s' %self.group.get_end_effector_link())
        rospy.loginfo('Robot Groups: %s' %self.robot.get_group_names())
        rospy.loginfo('Ready to start to plan for given palm goal poses.')

    def handle_arm_movement(self, req):
        self.group.go(wait=True)
        response = MoveArmResponse()
        response.success = True
        return response

    def create_arm_movement_server(self):
        rospy.Service('arm_movement', MoveArm, self.handle_arm_movement)
        rospy.loginfo('Service moveit_cartesian_pose_planner:')
        rospy.loginfo('Ready to start to execute movement plan on robot arm.')

if __name__ == '__main__':
    planner = CartesianPoseMoveitPlanner()
    planner.create_moveit_planner_server()
    planner.create_arm_movement_server()
    rospy.spin()

