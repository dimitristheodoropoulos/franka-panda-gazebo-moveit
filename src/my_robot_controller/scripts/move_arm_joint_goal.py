#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

def move_arm_joint_goal():
    # Initialize MoveIt! commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm_joint_goal', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Create a publisher for displaying trajectories in RViz (optional)
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    # Get basic information
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    print("============ Robot State:")
    print(robot.get_current_state())
    print("")

    # --- Move to a specific joint configuration ---
    print("============ Planning to a joint goal")

    # Get the current joint values (optional, for debugging)
    joint_goal = move_group.get_current_joint_values()
    print("Current Joint Values:", joint_goal)

    # Define the target joint values for the Panda arm
    # These values represent a specific pose for the robot.
    # Panda has 7 joints in the arm: panda_joint1 to panda_joint7
    joint_goal[0] = 0.0      # panda_joint1
    joint_goal[1] = -pi/4    # panda_joint2 (approx -45 degrees)
    joint_goal[2] = 0.0      # panda_joint3
    joint_goal[3] = -pi/2    # panda_joint4 (approx -90 degrees)
    joint_goal[4] = 0.0      # panda_joint5
    joint_goal[5] = pi/3     # panda_joint6 (approx 60 degrees)
    joint_goal[6] = 0.0      # panda_joint7

    # Plan the trajectory
    move_group.set_joint_value_target(joint_goal)
    plan = move_group.plan() # For Noetic, plan() returns a tuple (success, trajectory, planning_time, error_code)
    
    if plan[0]: # Check if planning was successful
        print("============ Visualizing plan 1 (joint goal) (now we will execute it)")
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan[1]) # plan[1] is the trajectory
        display_trajectory_publisher.publish(display_trajectory)
        
        rospy.sleep(1) # Give RViz time to display the plan
        
        # Execute the planned trajectory
        print("============ Executing plan 1")
        move_group.execute(plan[1], wait=True) # Execute the trajectory
        print("============ Plan 1 executed!")
        
    else:
        print("============ Joint goal planning FAILED!")

    # Ensure the robot is stopped (optional)
    move_group.stop()
    # It is always good to clear your targets after planning with a joint target
    move_group.clear_pose_targets()

    rospy.sleep(2) # Give some time to observe the final state

    # When finished, shut down MoveIt! commander
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Finished moving robot.")

if __name__ == '__main__':
    try:
        move_arm_joint_goal()
    except rospy.ROSInterruptException:
        pass