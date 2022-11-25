#!/usr/bin/env python3

import rospy
from control_msgs.msg import GripperCommand
from arduino_Thor.msg import joints_steps

valor=0

def publisher():

    jointStep = rospy.Publisher('joints_steps',joints_steps, queue_size=10)
    pub = rospy.Publisher('gripper',GripperCommand, queue_size=10)

    rate =rospy.Rate(1)

    msg=GripperCommand()
    joint = joints_steps()

    while not rospy.is_shutdown():
        msg.position = valor
        joint.joint_1 = 0
        joint.joint_2 = 0
        joint.joint_3 = 0
        joint.joint_4 = 0
        joint.joint_5 = 0
        joint.joint_6 = 0


        pub.publish(msg)
        jointStep.publish(joint)
        rospy.loginfo(valor)

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Thor_arduino')
    publisher()
