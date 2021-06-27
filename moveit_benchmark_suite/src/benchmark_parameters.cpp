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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_extents.h>
#include <geometric_shapes/solid_primitive_dims.h>
using namespace moveit::core;

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

void BenchmarkParameters::readCollisionObjects(ros::NodeHandle& nh) {
	XmlRpc::XmlRpcValue collision_objects_description;
	if (!nh.getParam("collision_objects", collision_objects_description) ||
	    !constructCollisionObjects(collision_objects_description, collision_objects_)) {
		ROS_WARN("Collision objects not found");
	}
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
	if (!isArray(params))
		return false;

	for (int i = 0; i < params.size(); ++i)  // NOLINT(modernize-loop-convert)
	{
		if (!params[i].hasMember("joint_state"))
			return false;

		if (!isStruct(params[i]["joint_state"], { "name", "position" }, "joint_state"))
			return false;

		if (!isValidStruct(params[i]["joint_state"], { "name", "position" }, "joint_state"))
			return false;

		if (!isArray(params[i]["joint_state"]["name"]) || !isArray(params[i]["joint_state"]["position"]))
			return false;

		if (params[i]["joint_state"]["name"].size() != params[i]["joint_state"]["position"].size())
			return false;

		moveit_msgs::RobotState rs;
		rs.joint_state.name.reserve(params[i]["joint_state"]["name"].size());
		rs.joint_state.name.reserve(params[i]["joint_state"]["position"].size());

		for (int j = 0; j < params[i]["joint_state"]["name"].size(); ++j)  // NOLINT(modernize-loop-convert)
		{
			std::string name = static_cast<std::string>(params[i]["joint_state"]["name"][j]);
			double position = parseDouble(params[i]["joint_state"]["position"][j]);

			rs.joint_state.name.emplace_back(name);
			rs.joint_state.position.emplace_back(position);
		}
		robot_states_.push_back(rs);
	}
	return true;
};
bool BenchmarkParameters::constructCollisionObjects(XmlRpc::XmlRpcValue& params,
                                                    std::vector<moveit_msgs::CollisionObject>& collision_objects) {
	if (!isArray(params))
		return false;

	for (int i = 0; i < params.size(); ++i)  // NOLINT(modernize-loop-convert)
	{
		if (!isStruct(params[i], { "id", "frame_id", "type" }, "Parameter"))
			return false;

		collision_objects.emplace_back(moveit_msgs::CollisionObject());

		moveit_msgs::CollisionObject& co = collision_objects.back();
		co.id = static_cast<std::string>(params[i]["id"]);
		co.header.frame_id = static_cast<std::string>(params[i]["frame_id"]);
		co.operation = co.ADD;

		if (params[i]["type"] == "mesh") {
			if (!isValidStruct(params[i], { "id", "type", "frame_id", "position", "orientation", "path", "scaling" },
			                   "collision_objects"))
				return false;
			co.meshes.resize(1);
			co.mesh_poses.resize(1);
			if (!constructMesh(params[i], co.meshes[0]))
				return false;
			if (!constructPose(params[i], co.mesh_poses[0]))
				return false;
		} else if (static_cast<std::string>(params[i]["type"]).find("primitive") != std::string::npos) {
			if (!isValidStruct(params[i], { "id", "type", "frame_id", "position", "orientation", "dimensions" },
			                   "collision_objects"))
				return false;
			co.primitives.resize(1);
			co.primitive_poses.resize(1);
			if (!constructSolidPrimitive(params[i], co.primitives[0]))
				return false;
			if (!constructPose(params[i], co.primitive_poses[0]))
				return false;
		} else {
			ROS_ERROR("Unrecognized type");
			return false;
		}
	}
	return true;
};

bool BenchmarkParameters::constructMesh(XmlRpc::XmlRpcValue& params, shape_msgs::Mesh& mesh) {
	if (!params.hasMember("path"))
		return false;

	const std::string& path = params["path"];
	Eigen::Vector3d scaling = Eigen::Vector3d::Zero();

	if (params.hasMember("scaling")) {
		if (!isArray(params["scaling"], 3, "scaling", "xyz dimension"))
			return false;
		auto& xyz = params["scaling"];
		scaling = Eigen::Vector3d(parseDouble(xyz[0]), parseDouble(xyz[1]), parseDouble(xyz[2]));
	}

	shapes::Shape* shape = shapes::createMeshFromResource(path, scaling);
	shapes::ShapeMsg shape_msg;
	shapes::constructMsgFromShape(shape, shape_msg);

	mesh = boost::get<shape_msgs::Mesh>(shape_msg);

	return true;
};

bool BenchmarkParameters::constructSolidPrimitive(XmlRpc::XmlRpcValue& params, shape_msgs::SolidPrimitive& primitive) {
	if (params["type"] == "box_primitive") {
		if (!constructBoxPrimitive(params, primitive))
			return false;
	} else if (params["type"] == "sphere_primitive") {
		if (!constructSpherePrimitive(params, primitive))
			return false;
	} else if (params["type"] == "cylinder_primitive") {
		if (!constructCylinderPrimitive(params, primitive))
			return false;
	} else if (params["type"] == "cone_primitive") {
		if (!constructConePrimitive(params, primitive))
			return false;
	} else {
		ROS_ERROR("Unrecognized Solid Primitive");
		return false;
	}
	return true;
};

// TODO make as template?
bool BenchmarkParameters::constructBoxPrimitive(XmlRpc::XmlRpcValue& params, shape_msgs::SolidPrimitive& primitive) {
	if (!params.hasMember("dimensions"))
		return false;
	if (!isArray(params["dimensions"], 3, "dimensions", "xyz box size"))
		return false;
	auto& xyz = params["dimensions"];
	primitive.type = shape_msgs::SolidPrimitive::BOX;
	primitive.dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>());
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = parseDouble(xyz[0]);
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = parseDouble(xyz[1]);
	primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = parseDouble(xyz[2]);

	return true;
};

bool BenchmarkParameters::constructSpherePrimitive(XmlRpc::XmlRpcValue& params, shape_msgs::SolidPrimitive& primitive) {
	if (!params.hasMember("dimensions"))
		return false;
	if (!isArray(params["dimensions"], 1, "dimensions", "sphere radius"))
		return false;
	auto& xyz = params["dimensions"];
	primitive.type = shape_msgs::SolidPrimitive::SPHERE;
	primitive.dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>());
	primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = parseDouble(xyz[0]);

	return true;
};

bool BenchmarkParameters::constructCylinderPrimitive(XmlRpc::XmlRpcValue& params,
                                                     shape_msgs::SolidPrimitive& primitive) {
	if (!params.hasMember("dimensions"))
		return false;
	if (!isArray(params["dimensions"], 2, "dimensions", "cylinder height and radius"))
		return false;
	auto& xyz = params["dimensions"];
	primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
	primitive.dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>());
	primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = parseDouble(xyz[0]);
	primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = parseDouble(xyz[1]);

	return true;
};

bool BenchmarkParameters::constructConePrimitive(XmlRpc::XmlRpcValue& params, shape_msgs::SolidPrimitive& primitive) {
	if (!params.hasMember("dimensions"))
		return false;
	if (!isArray(params["dimensions"], 2, "dimensions", "cone height and radius"))
		return false;
	auto& xyz = params["dimensions"];
	primitive.type = shape_msgs::SolidPrimitive::CONE;
	primitive.dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::SolidPrimitive::CONE>());
	primitive.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = parseDouble(xyz[0]);
	primitive.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] = parseDouble(xyz[1]);

	return true;
};

bool BenchmarkParameters::constructPose(XmlRpc::XmlRpcValue& params, geometry_msgs::Pose& pose) {
	pose.orientation.w = 1.0;

	if (params.hasMember("position")) {
		if (!isArray(params["position"], 3, "position", "xyz position"))
			return false;

		pose.position.x = parseDouble(params["position"][0]);
		pose.position.y = parseDouble(params["position"][1]);
		pose.position.z = parseDouble(params["position"][2]);
	}
	if (params.hasMember("orientation")) {
		if (!isArray(params["orientation"], 3, "orientation", "RPY values"))
			return false;

		auto& rpy = params["orientation"];
		tf2::Quaternion q;
		q.setRPY(parseDouble(rpy[0]), parseDouble(rpy[1]), parseDouble(rpy[2]));
		pose.orientation = toMsg(q);
	}
	return true;
};

bool BenchmarkParameters::isValidStruct(XmlRpc::XmlRpcValue& params, const std::set<std::string>& keys,
                                        const std::string& name) {
	for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it) {
		if (keys.find(it->first) == keys.end()) {
			ROS_WARN_STREAM(name << " contains unknown entity '" << it->first << "'");
			return false;
		}
	}
	return true;
}

}  // namespace benchmark_suite
}  // namespace moveit
