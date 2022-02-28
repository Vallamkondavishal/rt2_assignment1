#! /usr/bin/env python -tt

## @package rt2_assignment1
#
#  \file user_interface.py
#  \brief This file implements the behaviour that allows the robot to reach a goal.
#
#  \author Vishal Vallamkonda
#  \version 1.0
#  \date 09/02/2022
#  \details
#  
#
#  Services: <BR>
#    /user_interface
#
#
#  Description: <BR>
#    This node implements an user interface 

import rospy
import time
from rt2_assignment1.srv import Command

## \brief Brief function description
#
# This function is the main function
#

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Please wait, the robot is going to stop when the position will be reached")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
