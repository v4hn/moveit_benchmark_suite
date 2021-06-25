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

#include <moveit/benchmark_suite/benchmark_parameters.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/utils/xmlrpc_casts.h>

namespace moveit {
namespace benchmark_suite {

BenchmarkParameters::BenchmarkParameters() {}

BenchmarkParameters::~BenchmarkParameters() = default;
void BenchmarkParameters::readMotionPlanningBenchmark(ros::NodeHandle& nh) {}

void BenchmarkParameters::readStartStates(ros::NodeHandle& nh) {
	XmlRpc::XmlRpcValue start_states_description;
	if (!nh.getParam("start_states", start_states_description) ||
	    !constructRobotStates(start_states_description, robot_states_)) {
		ROS_ERROR("Robot start states not found");
	}
};

void BenchmarkParameters::readCollisionObjects(ros::NodeHandle& nh){

};

void BenchmarkParameters::readGoalConstraints(ros::NodeHandle& nh) {
	XmlRpc::XmlRpcValue constraint_description;
	if (!nh.getParam("goal_constraints", constraint_description) ||
	    !kinematic_constraints::constructConstraints(constraint_description, goal_constraints_)) {
		ROS_ERROR("Goal constraints not found");
	}
};

void BenchmarkParameters::readPathConstraints(ros::NodeHandle& nh) {
	XmlRpc::XmlRpcValue constraint_description;
	if (!nh.getParam("path_constraints", constraint_description) ||
	    !kinematic_constraints::constructConstraints(constraint_description, path_constraints_)) {
		ROS_WARN("Path constraints not found");
	}
};

void BenchmarkParameters::readTrajConstraints(ros::NodeHandle& nh){};

bool BenchmarkParameters::constructRobotStates(XmlRpc::XmlRpcValue& params,
                                               std::vector<moveit_msgs::RobotState>& robot_states) {
	if (!moveit::core::isArray(params))
		return false;

	for (int i = 0; i < params.size(); ++i)  // NOLINT(modernize-loop-convert)
	{
		if (!params[i].hasMember("joint_state"))
			return false;

		if (!moveit::core::isStruct(params[i]["joint_state"], { "name", "position" }, "Parameter"))
			return false;

		if (!moveit::core::isArray(params[i]["joint_state"]["name"]) ||
		    !moveit::core::isArray(params[i]["joint_state"]["position"]))
			return false;

		if (params[i]["joint_state"]["name"].size() != params[i]["joint_state"]["position"].size())
			return false;

		moveit_msgs::RobotState rs;
		rs.joint_state.name.reserve(params[i]["joint_state"]["name"].size());
		rs.joint_state.name.reserve(params[i]["joint_state"]["position"].size());

		for (int j = 0; j < params[i]["joint_state"]["name"].size(); ++j)  // NOLINT(modernize-loop-convert)
		{
			std::string name = static_cast<std::string>(params[i]["joint_state"]["name"][j]);
			double position = moveit::core::parseDouble(params[i]["joint_state"]["position"][j]);

			rs.joint_state.name.emplace_back(name);
			rs.joint_state.position.emplace_back(position);
		}
		robot_states_.push_back(rs);
	}
	return true;
};  // namespace benchmark_suite

}  // namespace benchmark_suite
}  // namespace moveit
