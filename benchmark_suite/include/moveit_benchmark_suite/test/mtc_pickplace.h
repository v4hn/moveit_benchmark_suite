/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/
#pragma once

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <eigen_conversions/eigen_msg.h>

namespace moveit
{
namespace benchmark_suite
{
using namespace moveit::task_constructor;

class PickPlaceTask
{
public:
  PickPlaceTask(const std::string& task_name, const ros::NodeHandle& nh);
  ~PickPlaceTask() = default;
  void loadParameters();

  void init();
  void pick();
  void place();

  bool plan();

  bool execute();

private:
  ros::NodeHandle nh_;

  std::string task_name_;
  moveit::task_constructor::TaskPtr task_;

  // planners
  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> sampling_planner_;
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner_;

  // stage forward
  Stage* stage_forward_ptr_ = nullptr;

  // planning group properties
  std::string arm_group_name_;
  std::string eef_name_;
  std::string hand_group_name_;
  std::string hand_frame_;

  // object + surface
  std::vector<std::string> support_surfaces_;
  std::string object_reference_frame_;
  std::string surface_link_;
  std::string object_name_;
  std::string world_frame_;
  std::vector<double> object_dimensions_;

  // Predefined pose targets
  double hand_open_gap_;
  double hand_close_gap_;

  moveit_msgs::RobotState hand_open_pose_;
  moveit_msgs::RobotState hand_close_pose_;

  // Execution
  actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_;

  // Pick metrics
  Eigen::Isometry3d grasp_frame_transform_;
  double approach_object_min_dist_;
  double approach_object_max_dist_;
  double lift_object_min_dist_;
  double lift_object_max_dist_;

  //// Place metrics
  geometry_msgs::Pose place_pose_;
  double place_surface_offset_;
};
}  // namespace benchmark_suite
}  // namespace moveit
