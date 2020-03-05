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
from trac_ik_python.trac_ik import IK


class CartesianPoseMoveitPlanner:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_goal_pose_planner_node')
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('kuka_arm')
        #Reference link: https://answers.ros.org/question/255792/change-planner-in-moveit-python/
        #self.group.set_planner_id('RRTstarkConfigDefault')
        self.group.set_planner_id('RRTConnectkConfigDefault')
        self.group.set_planning_time(10)
        self.group.set_num_planning_attempts(3)
        #self.group.set_planning_time(15)
        #self.group.set_num_planning_attempts(1)

        self.zero_joint_states = np.zeros(7) 
        self.left_home_pose = False
        if self.left_home_pose:
            self.home_joint_states = np.array([0.0001086467455024831, 0.17398914694786072, -0.00015721925592515618, 
                                                -1.0467143058776855, 0.0006054198020137846, 
                                                -0.00030679398332722485, 3.3859387258416973e-06])
        else:
            self.home_joint_states = np.array([0.0001086467455024831, -0.17398914694786072, 0.00015721925592515618, 
                                                1.0467143058776855, 0.0006054198020137846, 
                                                -0.00030679398332722485, 3.3859387258416973e-06])
        self.ik_solver = IK('world', 'allegro_mount')
        self.seed_state = [0.0] * self.ik_solver.number_of_joints

        
    def go_home(self):
        print 'go home'
        self.group.clear_pose_targets()
        self.group.set_joint_value_target(self.home_joint_states)
        plan_home = self.group.plan()
        return plan_home

    def go_zero(self):
        print 'go zero'
        self.group.clear_pose_targets()
        self.group.set_joint_value_target(self.zero_joint_states)
        plan_home = self.group.plan()
        return plan_home

    def go_goal(self, pose):
        print 'go goal'
        self.group.clear_pose_targets()
        self.group.set_pose_target(pose)
        plan_goal = self.group.plan()
        return plan_goal

    def go_goal_trac_ik(self, pose):
        print 'go goal'
        self.group.clear_pose_targets()
        ik_js = self.ik_solver.get_ik(self.seed_state, pose.position.x, pose.position.y, pose.position.z,
                                        pose.orientation.x, pose.orientation.y, pose.orientation.z, 
                                        pose.orientation.w)
        if ik_js is None:
            rospy.logerr('No IK solution for motion planning!')
            return None
        self.group.set_joint_value_target(np.array(ik_js))
        plan_goal = self.group.plan()
        return plan_goal

    def handle_pose_goal_planner(self, req):
        plan = None
        if req.go_home:
            plan = self.go_home()
        elif req.go_zero:
            plan = self.go_zero()
        else:
            # plan = self.go_goal(req.palm_goal_pose_world)
            plan = self.go_goal_trac_ik(req.palm_goal_pose_world)
        #print plan
        response = PalmGoalPoseWorldResponse()
        response.success = False
        if plan is None:
            return response
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
    # planner.create_arm_movement_server()
    rospy.spin()

