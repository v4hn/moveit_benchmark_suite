/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Captain Yoshi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Captain Yoshi
   Desc:
*/
#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <scene_parser/scene_parser.h>

#include <moveit_benchmark_suite/io/yaml.h>
#include <moveit_benchmark_suite/yaml.h>
#include <moveit_benchmark_suite/planning.h>
#include <moveit_benchmark_suite/benchmark.h>
#include <moveit_benchmark_suite/io/gnuplot.h>

using namespace moveit_benchmark_suite;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "benchmark");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle pnh("~");

  // Parse world
  SceneParser parser;
  parser.loadURDFFile(pnh, "scene/bbt");
  moveit_msgs::PlanningSceneWorld world;

  parser.getCollisionObjects(world.collision_objects);

  // Parse request
  std::string request_filename;
  pnh.getParam("/request/1", request_filename);

  moveit_msgs::MotionPlanRequest request;
  auto node = YAML::LoadFile(request_filename);

  request = node.as<moveit_msgs::MotionPlanRequest>();

  // Setup robot
  auto robot = std::make_shared<Robot>("robot", "robot_description");
  robot->initialize();

  // Setup scene
  planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(robot->getModelConst());
  scene->processPlanningSceneWorldMsg(world);

  // Setup planner
  auto ompl_planner = std::make_shared<PipelinePlanner>(robot);
  ompl_planner->initialize("ompl");

  auto stomp_planner = std::make_shared<PipelinePlanner>(robot);
  stomp_planner->initialize("stomp");

  auto move_group_planner = std::make_shared<MoveGroupPlanner>(robot);

  // Setup a benchmarking request for the joint and pose motion plan requests.
  PlanningProfiler::Options options;
  options.metrics =
      PlanningProfiler::WAYPOINTS | PlanningProfiler::CORRECT | PlanningProfiler::LENGTH | PlanningProfiler::SMOOTHNESS;
  PlanningBenchmark benchmark("PipelinePlaner vs MoveGroupInterface",  // Name of benchmark
                              options,                                 // Options for internal profiler
                              5.0,                                     // Timeout allowed for ALL queries
                              10);                                     // Number of trials

  request.pipeline_id = "ompl";
  request.planner_id = "RRTConnectkConfigDefault";
  benchmark.addQuery("rrt-pp", scene, ompl_planner, request);
  benchmark.addQuery("rrt-mgi", scene, move_group_planner, request);

  request.pipeline_id = "stomp";
  request.planner_id = "STOMP";
  benchmark.addQuery("stomp-pp", scene, stomp_planner, request);
  benchmark.addQuery("stomp-mgi", scene, move_group_planner, request);

  // Use the post-query callback to visualize the data live.
  IO::GNUPlotBoxPlotPlanDataSet plot_time("time");
  IO::GNUPlotBoxPlotPlanDataSet plot_waypoint("waypoints");
  IO::GNUPlotBoxPlotPlanDataSet plot_length("length");
  IO::GNUPlotBoxPlotPlanDataSet plot_smoothness("smoothness");
  IO::GNUPlotBarGraphPlanDataSet plot_success("success");
  IO::GNUPlotBarGraphPlanDataSet plot_correct("correct");

  benchmark.setPostQueryCallback([&](PlanDataSetPtr dataset, const PlanningQuery&) {
    plot_time.dump(*dataset);
    plot_waypoint.dump(*dataset);
    plot_success.dump(*dataset);
    plot_correct.dump(*dataset);
    plot_length.dump(*dataset);
    plot_smoothness.dump(*dataset);
  });

  auto dataset = benchmark.run();

  // Dump metrics to a logfile
  OMPLPlanDataSetOutputter output("demo");
  output.dump(*dataset);

  // ros::waitForShutdown();

  return 0;
}
