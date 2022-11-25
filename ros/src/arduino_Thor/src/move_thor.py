#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python',anonymous=True)
robot= moveit_commander.RobotCommander

arm_group= moveit_commander.MoveGroupCommander('Thor1')

success=False

pose_target= geometry_msgs.msg.Pose()

pose_target.position.x= 0.4645
pose_target.position.y= -0.139
pose_target.position.z= 0.02

yaw=0
pitch=0
#1.57-alpha
while not success:

    #while pitch!=1.57:
    quaternion=quaternion_from_euler( 0 ,pitch, 1.57-yaw)
    pose_target.orientation.w= quaternion[3]
    pose_target.orientation.x= quaternion[0]
    pose_target.orientation.y= quaternion[1]
    pose_target.orientation.z= quaternion[2]

    arm_group.set_pose_target(pose_target)
    plan=arm_group.go()
    success=plan
    print(plan)
    #if success:
    #    break
    pitch=pitch+0.10
    #roll=roll+0.01

#arm_group.execute(plan)

rospy.sleep(5)
moveit_commander.roscpp_shutdown()

def get_transformation(source_frame,target_frame, duration=2):
    tf_buffer=tf2_ros.Buffer(rospy.Duration(duration))
    tf_listener=tf2_ros.TransformListener(tf_buffer)

    try:
        transformation=tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logger('no se logro encontrar la tranformada de %s a $s' % source_frame, target_frame)
