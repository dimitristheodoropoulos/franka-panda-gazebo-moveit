#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import copy

def panda_gripper_control():
    # 1. Αρχικοποίηση ROS Node και MoveIt! Commander
    print("======== Starting Panda Gripper Control Node ========")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_gripper_control_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # 2. Ορισμός Planning Groups
    arm_group_name = "panda_arm"
    arm_group = moveit_commander.MoveGroupCommander(arm_group_name)
    arm_group.set_planning_time(10)
    arm_group.set_num_planning_attempts(10)

    gripper_group_name = "hand"
    gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
    gripper_group.set_planning_time(10)
    gripper_group.set_num_planning_attempts(50) # ΕΔΩ: Αύξηση προσπαθειών για τον gripper

    print("Joints in gripper group: %s" % gripper_group.get_active_joints())
    print("Joints in arm group: %s" % arm_group.get_active_joints())

    # ---- Κίνηση 1: Άνοιγμα Gripper ----
    rospy.loginfo("Opening the gripper...")
    current_gripper_joint_values = gripper_group.get_current_joint_values()

    try:
        finger_joint_index = gripper_group.get_active_joints().index("finger_joint")
        # ΕΔΩ: Αλλαγή στόχου σε 0.008m, το οποίο επιβεβαιώθηκε ως collision-free
        current_gripper_joint_values[finger_joint_index] = 0.008
    except ValueError:
        rospy.logerr("finger_joint not found in gripper group active joints. Check your SRDF.")
        return

    gripper_group.set_joint_value_target(current_gripper_joint_values)
    plan_success, plan_trajectory, planning_time, error_code = gripper_group.plan()

    if plan_success:
        gripper_group.execute(plan_trajectory, wait=True)
        rospy.loginfo("Gripper successfully opened!")
    else:
        rospy.logwarn("Failed to plan gripper open motion. Error code: %s" % str(error_code.val))

    gripper_group.stop()
    gripper_group.clear_pose_targets()
    rospy.sleep(1)

    # ---- Κίνηση 2: Κίνηση Panda Arm σε συγκεκριμένη πόζα ----
    rospy.loginfo("Moving Panda arm to a specific joint pose...")
    # Αυτή η πόζα επιβεβαιώθηκε ως collision-free
    joint_values_for_ready_pose = [0.0, -0.75, 0.0, -0.5, 0.0, 0.0, 0.785]
    arm_group.set_joint_value_target(joint_values_for_ready_pose)

    plan_success, plan_trajectory, planning_time, error_code = arm_group.plan()

    if plan_success:
        arm_group.execute(plan_trajectory, wait=True)
        rospy.loginfo("Panda arm moved to specified pose!")
    else:
        rospy.logwarn("Failed to plan arm motion to specified pose. Error code: %s" % str(error_code.val))

    arm_group.stop()
    arm_group.clear_pose_targets()
    rospy.sleep(1)

    # ---- Κίνηση 3: Κλείσιμο Gripper ----
    rospy.loginfo("Closing the gripper...")
    current_gripper_joint_values = gripper_group.get_current_joint_values()
    try:
        finger_joint_index = gripper_group.get_active_joints().index("finger_joint")
        current_gripper_joint_values[finger_joint_index] = 0.001
    except ValueError:
        rospy.logerr("finger_joint not found for closing motion.")
        return

    gripper_group.set_joint_value_target(current_gripper_joint_values)
    plan_success, plan_trajectory, planning_time, error_code = gripper_group.plan()

    if plan_success:
        gripper_group.execute(plan_trajectory, wait=True)
        rospy.loginfo("Gripper successfully closed!")
    else:
        rospy.logwarn("Failed to plan gripper close motion. Error code: %s" % str(error_code.val))

    gripper_group.stop()
    gripper_group.clear_pose_targets()
    rospy.sleep(1)

    rospy.loginfo("Panda Gripper Control Demo Finished!")

    moveit_commander.roscpp_shutdown()
    print("======== Panda Gripper Control Node Shut Down ========")

if __name__ == '__main__':
    try:
        panda_gripper_control()
    except rospy.ROSInterruptException:
        pass