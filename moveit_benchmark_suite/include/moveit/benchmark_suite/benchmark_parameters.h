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
   Desc: Load benchmark parameters
*/
#pragma once

#include <ros/ros.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/Constraints.h>

namespace moveit {
namespace benchmark_suite {

class BenchmarkParameters
{
public:
	BenchmarkParameters();

	virtual ~BenchmarkParameters();

	void readMotionPlanningBenchmark(ros::NodeHandle& nh);

	void readStartStates(ros::NodeHandle& nh);
	void readCollisionObjects(ros::NodeHandle& nh);
	void readGoalConstraints(ros::NodeHandle& nh);
	void readPathConstraints(ros::NodeHandle& nh);
	void readTrajConstraints(ros::NodeHandle& nh);

	const std::vector<moveit_msgs::RobotState>& getRobotStates() const { return robot_states_; }
	const moveit_msgs::Constraints& getGoalConstraints() const { return goal_constraints_; }
	const std::vector<moveit_msgs::CollisionObject>& getCollisionObjects() const { return collision_objects_; }

	bool constructRobotStates(XmlRpc::XmlRpcValue& params, std::vector<moveit_msgs::RobotState>& robot_states);
	bool constructCollisionObjects(XmlRpc::XmlRpcValue& params,
	                               std::vector<moveit_msgs::CollisionObject>& collision_objects);
	bool constructMesh(XmlRpc::XmlRpcValue& params, shape_msgs::Mesh& mesh);
	bool constructSolidPrimitive(XmlRpc::XmlRpcValue& params, shape_msgs::SolidPrimitive& primitive);
	bool constructBoxPrimitive(XmlRpc::XmlRpcValue& params, shape_msgs::SolidPrimitive& primitive);
	bool constructSpherePrimitive(XmlRpc::XmlRpcValue& params, shape_msgs::SolidPrimitive& primitive);
	bool constructCylinderPrimitive(XmlRpc::XmlRpcValue& params, shape_msgs::SolidPrimitive& primitive);
	bool constructConePrimitive(XmlRpc::XmlRpcValue& params, shape_msgs::SolidPrimitive& primitive);
	bool constructPose(XmlRpc::XmlRpcValue& params, geometry_msgs::Pose& pose);

	bool isValidStruct(XmlRpc::XmlRpcValue& params, const std::set<std::string>& keys, const std::string& name);

protected:
	std::vector<moveit_msgs::RobotState> robot_states_;
	std::vector<moveit_msgs::CollisionObject> collision_objects_;
	moveit_msgs::Constraints goal_constraints_;
	moveit_msgs::Constraints path_constraints_;
};
}  // namespace benchmark_suite
}  // namespace moveit