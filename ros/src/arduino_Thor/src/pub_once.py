#!/usr/bin/env python3

import rospy
from control_msgs.msg import GripperCommand
from arduino_Thor.msg import joints_steps


valor=55   #80 y 55


rospy.init_node('Thor_arduino')
#jointStep = rospy.Publisher('joints_steps',joints_steps, queue_size=10)
pub = rospy.Publisher('gripper_Thor1',GripperCommand, queue_size=10)

joint = joints_steps()
msg = GripperCommand()
msg.position = valor

joint.joint_1 = 0
joint.joint_2 = 0
joint.joint_3 = 0
joint.joint_4 = 0
joint.joint_5 = 0
joint.joint_6 = 0


rate =rospy.Rate(1)
ctrl_c = False

while not ctrl_c:
    connections=pub.get_num_connections()

    if connections > 0:
        #pub.publish(msg)
        pub.publish(msg)
        ctrl_c= True
    else:
        rate.sleep()
