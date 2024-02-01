#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import sin, cos, pi
from line_detection_package.msg import line_follower

#initialize the ros node and publisher
rospy.init_node('line_controller', anonymous=False)
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

#initialize the twist message for controlling the robot movement
move_cmd = Twist()

#set the publishing rate
r = rospy.Rate(20)
rospy.sleep(2)

#initialize variables related to to line detection.
line_isDetected = False
line_orientation = "none"
line_error_rad = 0.0
goal_angle = 0.0

ctrl_c_pressed = False
#line detection info from msg
def get_line_info(msg):
    global line_isDetected, line_orientation, line_error_rad
    line_isDetected = msg.line_detected
    line_orientation = msg.orientation
    line_error_rad = msg.error_rad

#calculate the goal angle based on line direction
def g_angle():
    if line_isDetected is True:
        if line_orientation == "Left":
            goal_angle = line_error_rad
        if line_orientation == "Right":
            goal_angle = line_error_rad * (-1)
    else:
        goal_angle = 0.0
    return goal_angle

#control linear and angular velocities using trigonometrical or classic kp controller
def velocity_control(goal_rot):
    
    #### trigonometrical controller####
    
    dl=0.035 #0.042 #0.0422222#0.042
   
    
    V=0.08 #V=0.27 #0.22#0.35#0.27 
    error = goal_rot 
    linear_vel=V*cos(error) #linear velocity
    angular_vel=(V/dl)*sin(error)  #angular velocity
    
    '''
    ####classic kp controller, needs tuning#####
    
    linear_vel=0.03    #Kp_linear = 0.5  linear_vel=0.5 #linear velocity
  
    Kp_angular = 7.3,     #Kp_angular = 0.8 , 1.9 
    
    angular_vel = Kp_angular * (error) #angular velocity

    '''
    print("goal_rotation",goal_rot)
    
    
    return (linear_vel, angular_vel)

rospy.Subscriber('/line_follower/detector', line_follower, get_line_info)

# main execution loop
try:
    while not rospy.is_shutdown():
        goal_angle = g_angle()
        print (goal_angle)
        # print sub_line.orientation
        goal_rotation =  goal_angle
        
        (lin_vel, ang_vel) = velocity_control(goal_rotation)
        move_cmd.linear.x = lin_vel ##controlled linear velocity
        move_cmd.angular.z = ang_vel ##controlled angular velocity
        print (move_cmd)
        cmd_vel.publish(move_cmd)
        
        r.sleep()

except KeyboardInterrupt:
    # This block will be executed when Ctrl+C is pressed
    ctrl_c_pressed = True
    rospy.loginfo("Ctrl+C pressed. Stopping the robot...")

finally:
    # This block will be executed on normal shutdown or if Ctrl+C is pressed
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    cmd_vel.publish(move_cmd)

    if ctrl_c_pressed:
        # Use rospy.Rate for sleep duration
        rate = rospy.Rate(10)  # 1 Hz, adjust as needed
        for _ in range(3):
            try:
                cmd_vel.publish(move_cmd)  # Publish zero velocity repeatedly
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                # Break out of the loop if ROS shutdown request occurs
                break
