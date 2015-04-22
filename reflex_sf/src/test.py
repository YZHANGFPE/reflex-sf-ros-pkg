#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
    def __init__(self, motor_name):      
        self.jta = actionlib.SimpleActionClient('/f_hand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

            
    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()                      
        goal.trajectory.joint_names = ['reflex_sf_f1', 'reflex_sf_f2','reflex_sf_f3','reflex_sf_preshape']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(2)                       
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)
              

def main():
    arm = Joint('f_arm')
    
    rospy.sleep(5)
    # initial position
    arm.move_joint([0.0,0.0,0.0,2.2])
    # cylindrical grasp
    arm.move_joint([3.0,3.0,3.0,2.2])
    rospy.sleep(3)
    # initial position
    arm.move_joint([0.0,0.0,0.0,2.2])
    # open position for pinch grasp
    arm.move_joint([0.0,0.0,0.0,0.0])
    # pinch grasp
    arm.move_joint([3.0,3.0,0.0,0.0])
    rospy.sleep(3)
    arm.move_joint([0.0,0.0,0.0,0.0])
    arm.move_joint([0.0,0.0,0.0,2.2])

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()