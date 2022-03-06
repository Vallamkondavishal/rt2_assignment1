"""

.. module:: go_to_point
     :platform: Unix
     :synopsis: Python module for piloting the robot to the target
   
     :moduleauthor:: Vishal Vallamkonda <vvishal243@yahoo.com>

ROS node for driving a robot to a specific point

Subscribes to:
    /odom topic where the simulator publishes the robot position
    
Publishes to:
    /cmd_vel the desired robot position
    
Service :
    /go_to_point to start the robot motion.

This node define the movememnt of the holonomic robot using the go_to_point algorithm
    
"""

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
#from rt2_assignment1.srv import Position
import math
import numpy as np
import actionlib
import actionlib.msg
import rt2_assignment1.msg

# robot state variables
position_ = Point()
"""Point: actual robot position
"""

pose_=Pose()
yaw_ = 0
position_ = 0
state_ = 0
## publisher
pub_ = None


# parameters for control
desired_position_= Point()
desired_position_.z=0
success = False
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
## Maximum angular speed
ub_a = 0.6
lb_a = -0.5
## Maximum linear speed
ub_d = 0.6



#action server
act_s=None





def clbk_odom(msg):
    """
    This Callback is for the subscriber to topic odom. It keeps track of the robot pose.

    Args:
        msg(Odometry): the message received
    Returns: none
    """
    global position_
    global pose_
    global yaw_
    
    # position	
    position_ = msg.pose.pose.position
    #yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]



def change_state(state):
    """
    Function to specify the state value
    
    Update the current global state
    
    Args: 
        state (int):  new state
    
    """ 

    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


## The normalize_angle function.
#
#  It performs normalazation of an angle .
def normalize_angle(angle):
    """
    Function for normalizing the angle between -pi and pi.

    Args:
        angle(Float): the input angle
    Returns:angle(Float):
        the normalized angle.
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    """
    
    Orient the robot in a desired way
    Args:  des_yaw (float):  desired yaw
        next_state (int): next state to set
        
    """
    global yaw_, pub, yaw_precision_2_, state_
    des_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)





def go_straight_ahead(des_pos):
    """
    Function for going straight ahead to the target.

    Args:
        des_pos(Point): the desiderd pose
    Returns: none
    """
    global yaw_, pub, yaw_precision_, state_
    des_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = des_yaw - yaw_
    err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def fix_final_yaw(des_yaw):
    """
    Function for fixing the final angle of the robot, to the final position

    Args:
        des_yaw(float): the desiderd yaw
    Returns: none
    """
    err_yaw = normalize_angle(des_yaw - yaw_)
    #rospy.loginfo(err_yaw)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)

      
def done():
    """
    Calling this function, the robot will stop

    Args: none
    Returns: none
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    success = True
    act_s.set_succeeded()
 
 
   
def go_to_point(goal):
    """
    It is the action callback. The script will remain inside this function, due to the while loop.
    Here it will wait for new goal, ask the robot to reach them and delete goal if required. The state will also be updated

    Args:
        goal(rt2_assignment1.msg.go_to_pointActionGoal): goal of the robot
    Returns: none
    """
    global state_, desired_position_, act_s, success
    
#   desider position and orientation acquired    
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    des_yaw = goal.target_pose.pose.position.z
    change_state(0)
    while True:
        #checking if the client is requested to cancel the goal
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub_.publish(twist_msg)
            act_s.set_preempted()
            success=False 
            break
        elif state_ == 0:
            fix_yaw(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            fix_final_yaw(des_yaw)
        elif state_ == 3:
            done()
            break
    return True



## Documentation for the main function.
#
#  More details.
#
# @param None
def main():
    """
    Main function to manage 
    the robot behaviour
    Including :
	- the initialization of the *go_to_point*  node
	- the publisher for the "\cmd_vel" topic
	- the subscriber to the "\odom" topic
	- the action server *\go_to_point*
    """
    global pub_, active_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer('/go_to_point', rt2_assignment1.msg.GotopointAction, go_to_point, auto_start=False)#creation of the action server
         # generally auto_start needs to go to false, or it could sometime start too early; we have to manually start
         # planning is the action binded to the callback of the action
    
    #vel_s = rospy.Subscriber('/set_vel', SetVel, clbk_set_vel)
    act_s.start()
    
    
    rospy.spin()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
