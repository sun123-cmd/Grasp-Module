#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R


def move_to_goal(x, y, z, rotation_matrix):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_goal', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "ur3_manipulator"  # 修改为您的planning group名称
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # 旋转矩阵转换为四元数
    r = R.from_matrix(rotation_matrix)
    quat = r.as_quat()  # [x, y, z, w]

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)
    move_group.set_planning_time(30.0)
    # 进行路径规划并执行
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if plan:
        print("规划并执行成功。")
    else:
        print("规划执行失败。")

    # 结果反馈
    current_pose = move_group.get_current_pose().pose
    print("当前机械臂末端执行器姿态：", current_pose)

if __name__ == "__main__":
    try:

        input_str = input("请输入目标位置的(x, y, z)坐标和旋转矩阵(a,b,c,d,e,f,g,h,i)，使用空格分隔：")
        input_list = [float(i) for i in input_str.split()]
        x, y, z = input_list[:3]
        rotation_matrix = np.array(input_list[3:]).reshape(3, 3)
        move_to_goal(x, y, z, rotation_matrix)
    except rospy.ROSInterruptException:
        pass
    except ValueError as e:
        print("输入错误：", e)
