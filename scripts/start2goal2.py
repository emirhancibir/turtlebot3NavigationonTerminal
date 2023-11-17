#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib

class MoveBaseController:
    def __init__(self):
        rospy.init_node("move_base_start2goal_node")
        rospy.loginfo("Node başlatildi.")
        self.rate = rospy.Rate(1)
        self.ipub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)

    def pose_estimate(self):
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

        for _ in range(3):
            self.ipub.publish(initpose)
            self.rate.sleep()

        rospy.loginfo(initpose)
        rospy.loginfo("Pose estimate gönderildi.")

    def nav_goal(self, x, y): #can use this one or movebase_client
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.pose.position.x = x
        goal.goal.target_pose.pose.position.y = y
        goal.goal.target_pose.pose.position.z = 0.0
        goal.goal.target_pose.pose.orientation.x = 0.0
        goal.goal.target_pose.pose.orientation.y = 0.0
        goal.goal.target_pose.pose.orientation.z = 0.866324240397043
        goal.goal.target_pose.pose.orientation.w = 0.49948204222022485

        for _ in range(3):
            self.pub.publish(goal)
            self.rate.sleep()

        rospy.loginfo(goal)
        rospy.loginfo("Goal gönderildi.")
        self.rate.sleep()

    def movebase_client(self, goal_x, goal_y): 

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = -0.5504909014748598 # rastgele aci degerleri
        goal.target_pose.pose.orientation.w = 0.8348411629725718
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()

if __name__ == '__main__':
    controller = MoveBaseController()

    x = float(input("x : "))
    y = float(input("y : "))
    
    controller.pose_estimate()
    rospy.sleep(2) 
    # controller.nav_goal(x, y)
    res = controller.movebase_client(x, y)
    if res:
        rospy.loginfo("Goal reached!")

