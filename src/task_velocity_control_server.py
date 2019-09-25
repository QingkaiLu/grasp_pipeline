#!/usr/bin/env python


from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from grasp_pipeline.srv import *
import copy


class TaskVelocityControl():
    '''
    Task velocity control.
    '''

    def __init__(self):
        rospy.init_node('task_velocity_control_server')
        self.robot = URDF.from_parameter_server()
        base_link = 'lbr4_base_link'
        end_link = 'palm_link'
        self.kdl_kin = KDLKinematics(self.robot, base_link, end_link)
        self.dq = 10**-5 * np.ones(7)
        self.display_traj = None
        lbr4_joint_states_topic = '/lbr4/joint_states'
        rospy.Subscriber(lbr4_joint_states_topic,
                         JointState, self.get_lbr4_joint_state)
        self.lbr4_joint_state = None

    def get_lbr4_joint_state(self, lbr4_js):
        self.lbr4_joint_state = lbr4_js

    def compute_manipulability_score(self, J):
        return np.sqrt(np.linalg.det(np.matmul(J, J.transpose())))

    def compute_redundancy_manipulability_resolution(self, q_cur, q_vel, J):
        m_score = self.compute_manipulability_score(J)
        J_prime = self.kdl_kin.jacobian(q_cur + self.dq)
        m_score_prime = self.compute_manipulability_score(J_prime)
        q_vel_null = (m_score_prime - m_score) / self.dq
        return q_vel_null

    def null_space_projection(self, q_cur, q_vel, J, J_pinv):
        identity = np.identity(7)
        q_vel_null = \
            self.compute_redundancy_manipulability_resolution(q_cur, q_vel, J)
        q_vel_constraint = np.array(np.matmul((
            identity - np.matmul(J_pinv, J)), q_vel_null))[0]
        q_vel_proj = q_vel + q_vel_constraint
        return q_vel_proj

    def damped_pinv(self, A, rho=0.017):
        AA_T = np.dot(A, A.T)
        damping = np.eye(A.shape[0]) * rho**2
        inv = np.linalg.inv(AA_T + damping)
        d_pinv = np.dot(A.T, inv)
        return d_pinv

    def straight_line_planner(self, q_init, dist=0.15):
        pose_init = self.kdl_kin.forward(q_init)
        z_init = pose_init[2, 3]
        q_cur = np.copy(q_init)
        pose_cur = np.copy(pose_init)
        pos_vel = np.zeros(6)
        pos_vel[2] = 0.05  # 0.01
        delta_t = 0.1  # 0.002
        joint_pos_list = [q_cur]
        joint_vel_list = [np.zeros(7)]
        valid_plan = True
        while pose_cur[2, 3] - z_init < dist:
            J = self.kdl_kin.jacobian(q_cur)
            J_pinv = self.damped_pinv(J)
            # Convert numpy matrix to numpy array
            # and reduce the dimension
            q_vel = np.matmul(J_pinv, pos_vel)
            # Convert numpy matrix to numpy array
            # and reduce the dimension
            q_vel = np.array(q_vel)[0]
            q_vel = self.null_space_projection(q_cur, q_vel, J, J_pinv)
            delta_q = q_vel * delta_t
            q_cur = np.copy(q_cur) + delta_q
            pose_cur = self.kdl_kin.forward(q_cur)
            # Check if joint states are in joint limits
            valid_plan = np.all([q_cur >= self.kdl_kin.joint_limits_lower,
                                q_cur <= self.kdl_kin.joint_limits_upper])
            if not valid_plan:
                rospy.loginfo('Check lower joint limits %s.'
                              %str(q_cur >= self.kdl_kin.joint_limits_lower))
                rospy.loginfo('Check upper joint limits %s.'
                              %str(q_cur <= self.kdl_kin.joint_limits_upper))
                rospy.loginfo('Joints out of limits!')
                break
            joint_pos_list.append(q_cur)
            joint_vel_list.append(q_vel)
        joint_traj = JointTrajectory()
        joint_traj.header.frame_id = '/world'
        joint_traj.joint_names = self.kdl_kin.get_joint_names()
        for i, joint_pos in enumerate(joint_pos_list):
            joint_traj_point = JointTrajectoryPoint()
            joint_traj_point.positions = joint_pos
            joint_traj_point.velocities = joint_vel_list[i]
            joint_traj.points.append(joint_traj_point)
        return valid_plan, joint_traj

    def gen_straight_line_plan(self, req):
        response = StraightLinePlanResponse()
        cur_js = self.lbr4_joint_state.position
        valid_plan, joint_traj = \
            self.straight_line_planner(cur_js, req.lift_height)
        response.success = valid_plan
        response.plan_traj = copy.copy(joint_traj)
        self.create_display_traj(joint_traj)
        rospy.loginfo('The straight plan generated is %s'%str(valid_plan))
        return response

    def create_sl_planner_server(self):
        '''
        Create straight line planner service.
        '''
        rospy.Service('straight_line_planner', StraightLinePlan,
                      self.gen_straight_line_plan)
        rospy.loginfo('Service straight_line_planner:')
        rospy.loginfo('Ready to plan straight line paths.')

    def create_display_traj(self, joint_traj):
        self.display_traj = DisplayTrajectory()
        self.display_traj.model_id = 'lbr4'
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = joint_traj
        self.display_traj.trajectory.append(robot_traj)
        self.display_traj.trajectory_start.\
            joint_state.header.frame_id = '/world'
        self.display_traj.trajectory_start.joint_state.name = \
            self.kdl_kin.get_joint_names()
        self.display_traj.trajectory_start.joint_state.position = \
            joint_traj.points[0].positions
        self.display_traj.trajectory_start.joint_state.velocity = \
            joint_traj.points[0].velocities


if __name__ == '__main__':
    task_velocity_control = TaskVelocityControl()
    task_velocity_control.create_sl_planner_server()
    plan_pub = rospy.Publisher('straight_line_plan',
                               DisplayTrajectory, queue_size=1)
    run_rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if task_velocity_control.display_traj is not None:
            plan_pub.publish(task_velocity_control.display_traj)
            run_rate.sleep()
