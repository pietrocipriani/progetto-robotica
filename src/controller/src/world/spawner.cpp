#include "controller/world/spawner.hpp"

#include <cmath>
#include <random>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>

#include "controller/util/string_format.hpp"
using controller::util::string_format;

namespace controller::world {

constexpr const char* BLOCK_SDF = R"(
<?xml version="1.0" ?>
<sdf version="1.4">
<model name="brick_%s">
<link name="link">
	<inertial>
		<mass>1</mass>
		<inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
		</inertia>
	</inertial>

	<collision name="collision">
		<geometry>
			<mesh>
			    <uri>model://brick_%s/mesh.stl</uri>
			</mesh>
		</geometry>
	</collision>

	<visual name="visual">
		<geometry>
			<mesh>
			<uri>model://brick_%s/mesh.stl</uri>
			</mesh>
		</geometry>
		<material>
			<ambient>%f %f %f %f</ambient>
			<diffuse>0.7 0.7 0.7 1.0</diffuse>
			<specular>0.01 0.01 0.01 1 1.5</specular>
			<emissive>0.0 0.0 0.0 0.0</emissive>
		</material>
        <transparency>0</transparency>
        <cast_shadows>1</cast_shadows>
	</visual>
</link>
</model>
</sdf>
)";

constexpr const char* PAD_SDF = R"(
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='pad_%s'>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <mesh>
            <uri>model://pad_%s/mesh.stl</uri>
            <scale>0.04 0.07 0.07</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.0 0.0 0.0 1.0</ambient>
          <diffuse>0.0 0.0 0.0 1.0</diffuse>
          <specular>0.1 0.1 0.1 1.0 5.0</specular>
        </material>
        <transparency>0</transparency>
        <cast_shadows>0</cast_shadows>
      </visual>
    </link>
    <static>1</static>
  </model>
</sdf>
)";

Spawner::Spawner(ros::ServiceClient&& _client) : client{_client} {}

void Spawner::spawn_block(
    planner::Block block_type,
    double x,
    double y,
    double angle,
    bool upside_down,
    const util::Color& color
) {
    static std::mt19937 rng = std::mt19937(std::random_device()());

	geometry_msgs::Point point;
	point.x = x;
	point.y = y;
	point.z = upside_down ? 0.93 : 0.87;

	geometry_msgs::Quaternion quaternion;
    if (upside_down) {
        quaternion.w = 0.0;
        quaternion.x = -std::sin(angle / 2);
        quaternion.y = std::cos(angle / 2);
        quaternion.z = 0.0;
    } else {
        quaternion.w = std::cos(angle / 2);
        quaternion.x = 0.0;
        quaternion.y = 0.0;
        quaternion.z = std::sin(angle / 2);
    }

	geometry_msgs::Pose pose;
	pose.position = point;
	pose.orientation = quaternion;

    gazebo_msgs::SpawnModel srv;
    const char* block_name = get_name(block_type);
	srv.request.model_name = string_format("%s_#%X_%d", block_name, color.rgba_hex, abs(int(rng())));
	srv.request.model_xml = string_format(BLOCK_SDF, block_name, block_name, block_name,
        color.r / 255.f, color.g / 255.f, color.b / 255.f, color.a / 255.f);
	srv.request.robot_namespace = "/gazebo/";
	srv.request.initial_pose = pose;

    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Spawned block %s, success: %d status: %s", block_name,
                (int)srv.response.success, srv.response.status_message.c_str());
        } else {
            ROS_ERROR("Error spawning block %s, success: %d status: %s", block_name,
                (int)srv.response.success, srv.response.status_message.c_str());
        }
    } else {
        ROS_ERROR("Error spawning block %s, failed to call service", block_name);
    }
}

void Spawner::spawn_pad(
    planner::Block pad_for_block_type,
    double x,
    double y,
    double angle
) {
	geometry_msgs::Point point;
	point.x = x;
	point.y = y;
	point.z = 0.87;

	geometry_msgs::Quaternion quaternion;
    quaternion.w = std::cos(angle / 2);
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = std::sin(angle / 2);

	geometry_msgs::Pose pose;
	pose.position = point;
	pose.orientation = quaternion;

    gazebo_msgs::SpawnModel srv;
    const char* block_name = get_name(pad_for_block_type);
    // do not use random names for pads, since we never want more than one instance for each
	srv.request.model_name = string_format("pad_%s", block_name);
	srv.request.model_xml = string_format(PAD_SDF, block_name, block_name);
	srv.request.robot_namespace = "/gazebo/";
	srv.request.initial_pose = pose;

    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Spawned pad %s, success: %d status: %s", block_name,
                (int)srv.response.success, srv.response.status_message.c_str());
        } else {
            ROS_ERROR("Error spawning pad %s, success: %d status: %s", block_name,
                (int)srv.response.success, srv.response.status_message.c_str());
        }
    } else {
        ROS_ERROR("Error spawning pad %s, failed to call service", block_name);
    }
}

} // namespace controller::world
