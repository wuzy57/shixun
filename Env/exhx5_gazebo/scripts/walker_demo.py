#!/usr/bin/env python

import rospy
from exhx5_gazebo.exhx5 import Exhx5


if __name__=="__main__":
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating Exhx5 Client")
    exhx5=Exhx5()
    rospy.sleep(1)
 
    rospy.loginfo("Exhx5 Walker Demo Starting")


    exhx5.set_walk_velocity(0.2,0,0)
    rospy.sleep(3)
    exhx5.set_walk_velocity(1,0,0)
    rospy.sleep(3)
    exhx5.set_walk_velocity(0,1,0)
    rospy.sleep(3)
    exhx5.set_walk_velocity(0,-1,0)
    rospy.sleep(3)
    exhx5.set_walk_velocity(-1,0,0)
    rospy.sleep(3)
    exhx5.set_walk_velocity(1,1,0)
    rospy.sleep(5)
    exhx5.set_walk_velocity(0,0,0)

    exhx5.set_angles({"j_pan":1})
    rospy.sleep(1)
    exhx5.set_angles({"j_pan":-1})
    rospy.sleep(1) 
    
    rospy.loginfo("Exhx5 Walker Demo Finished")
