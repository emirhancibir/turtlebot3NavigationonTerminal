#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import  PoseWithCovarianceStamped

def movebase_client(goal_x, goal_y):

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
    goal.target_pose.pose.orientation.z = -0.5504909014748598
    goal.target_pose.pose.orientation.w = 0.8348411629725718
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')

        
        
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


        rate.sleep()
        
        goal_x = 1.57 #float(input("x : "))
        goal_y = - 0.78 #float(input("y : "))

        
        
        result = movebase_client(goal_x, goal_y)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

       
        




       









