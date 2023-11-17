#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import  PoseWithCovarianceStamped

def pose_estimate():
    ipub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
    initpose = PoseWithCovarianceStamped()

    initpose.header.frame_id = "map"

    initpose.pose.pose.position.x = -0.6899999380111694
    initpose.pose.pose.position.y = 0.08999999612569809
    initpose.pose.pose.position.z = 0.0  
    initpose.pose.pose.orientation.x = 0.0
    initpose.pose.pose.orientation.w = 0.9995511094798347
    initpose.pose.pose.orientation.y = 0.0
    initpose.pose.pose.orientation.z = -0.029959631800665718
    initpose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

    rate = rospy.Rate(1)

    for _ in range(3):
        ipub.publish(initpose)
        rate.sleep()

    rospy.loginfo(initpose)
    rospy.loginfo("Pose estimate gönderildi.")
    
      


def nav_goal(x, y):

    pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)

    goal = MoveBaseActionGoal()

    goal.goal.target_pose.header.frame_id = "map"

    goal.goal.target_pose.pose.position.x = x # -1.9004530906677246
    goal.goal.target_pose.pose.position.y = y # 1.2669683694839478
    goal.goal.target_pose.pose.position.z = 0.0
    
    goal.goal.target_pose.pose.orientation.x = 0.0
    goal.goal.target_pose.pose.orientation.y = 0.0
    goal.goal.target_pose.pose.orientation.z = 0.866324240397043
    goal.goal.target_pose.pose.orientation.w = 0.49948204222022485
    rate = rospy.Rate(1)
    

    for _ in range(3):
        pub.publish(goal)
        rate.sleep()


    rospy.loginfo(goal)
    rospy.loginfo("goal gönderildi.")
    rate.sleep()


if __name__ == '__main__':


    rospy.init_node("move_base_start2goal_node")
    rospy.loginfo("node basladi.")

    x = float(input("x : "))
    y = float(input("y : "))
    
    pose_estimate()
    rospy.sleep(2) 
    nav_goal(x, y)


