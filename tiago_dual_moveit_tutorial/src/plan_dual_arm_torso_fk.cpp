/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */


/** \author Alessandro Di Fava. */

// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_dual_arm_torso_fk");

  if ( argc < 10 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun tiago_moveit_tutorial plan_dual_arm_torso_fk [left|right] torso_lift arm_1 arm_2 arm_3 arm_4 arm_5 arm_6 arm_7");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments are the target values for the given joints");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }

  std::string arm_name = argv[1];
  std::map<std::string, double> target_position;

  target_position["torso_lift_joint"] = atof(argv[2]);
  std::string target_joint = "arm_" + arm_name + "_1_joint";
  target_position[target_joint] = atof(argv[3]);
  target_joint = "arm_" + arm_name + "_2_joint";
  target_position[target_joint] = atof(argv[4]);
  target_joint = "arm_" + arm_name + "_3_joint";
  target_position[target_joint] = atof(argv[5]);
  target_joint = "arm_" + arm_name + "_4_joint";
  target_position[target_joint] = atof(argv[6]);
  target_joint = "arm_" + arm_name + "_5_joint";
  target_position[target_joint] = atof(argv[7]);
  target_joint = "arm_" + arm_name + "_6_joint";
  target_position[target_joint] = atof(argv[8]);
  target_joint = "arm_" + arm_name + "_7_joint";
  target_position[target_joint] = atof(argv[9]);

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  //select group of joints
  std::string moveit_group = "arm_" + arm_name + "_torso";
  moveit::planning_interface::MoveGroupInterface group_arm_torso(moveit_group);
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");

  torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
    if ( target_position.count(torso_arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm_torso.setPlanningTime(5.0);
  bool success = bool(group_arm_torso.plan(my_plan));

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
  if (!bool(e))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  spinner.stop();

  return EXIT_SUCCESS;
}
