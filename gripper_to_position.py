#!/usr/bin/env python

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from time import sleep

cp = -100   # current position, set as some null value to begin
rPR_c = -100     # requested position, set as some null value to begin

def printStatus(status):
    
    cp = status.gPR

    if cp == rPR_c:
        #rospy.loginfo(cp)
        rospy.signal_shutdown("Run to position COMPLETE")

def gripper_to_pos(position):

    global rPR_c
    rPR_c = position    # set requested position global vairable as value of position param

    pub = rospy.Publisher('/gripper/output', outputMsg.Robotiq2FGripper_robot_output, queue_size=3)

    command = outputMsg.Robotiq2FGripper_robot_output();

    while not rospy.is_shutdown():

        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 100
        command.rPR = rPR_c
        
        pub.publish(command)

        rospy.Subscriber("/gripper/input", inputMsg.Robotiq2FGripper_robot_input, printStatus)
