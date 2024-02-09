#include "controller/world/block_spawner.hpp"

#include <cmath>
#include <random>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>

#include "controller/util/string_format.hpp"
using controller::util::string_format;

namespace controller::world {

constexpr const char* MODEL_SDF = R"(
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
        <laser_retro>0</laser_retro>
        <max_contacts>10</max_contacts>
		<geometry>
			<mesh>
			    <uri>model://brick_%s/mesh.stl</uri>
			</mesh>
		</geometry>
        <surface>
            <friction>
                <ode>
                    <mu>1</mu>
                    <mu2>1</mu2>
                    <fdir1>0 0 0</fdir1>
                    <slip1>0</slip1>
                    <slip2>0</slip2>
                </ode>
                <torsional>
                    <coefficient>1</coefficient>
                    <patch_radius>0</patch_radius>
                    <surface_radius>0</surface_radius>
                    <use_patch_radius>1</use_patch_radius>
                    <ode>
                        <slip>0</slip>
                    </ode>
                </torsional>
            </friction>
            <bounce>
                <restitution_coefficient>0</restitution_coefficient>
                <threshold>1e+06</threshold>
            </bounce>
            <contact>
                <collide_without_contact>0</collide_without_contact>
                <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
                <collide_bitmask>1</collide_bitmask>
                <ode>
                    <soft_cfm>0</soft_cfm>
                    <soft_erp>0.2</soft_erp>
                    <kp>1e+13</kp>
                    <kd>1</kd>
                    <max_vel>0.01</max_vel>
                    <min_depth>0</min_depth>
                </ode>
                <bullet>
                    <split_impulse>1</split_impulse>
                    <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
                    <soft_cfm>0</soft_cfm>
                    <soft_erp>0.2</soft_erp>
                    <kp>1e+13</kp>
                    <kd>1</kd>
                </bullet>
            </contact>
        </surface>
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

BlockSpawner::BlockSpawner(ros::ServiceClient&& _client) : client{_client} {}

void BlockSpawner::spawn_block(
    BlockType block_type,
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
	point.z = upside_down ? 0.93 : 0.88;

	geometry_msgs::Quaternion quaternion;
    if (upside_down) {
        quaternion.w = 0.0;
        quaternion.x = std::sin(angle / 2);
        quaternion.y = std::cos(angle / 2);
        quaternion.z = 0.0;
    } else {
        quaternion.w = std::cos(angle / 2);
        quaternion.x = 0.0;
        quaternion.y = 0.0;
        quaternion.z = -std::sin(angle / 2);
    }

	geometry_msgs::Pose pose;
	pose.position = point;
	pose.orientation = quaternion;

    gazebo_msgs::SpawnModel srv;
    const char* block_name = get_name(block_type);
	srv.request.model_name = string_format("%s_#%X_%d", block_name, color.rgba_hex, abs(int(rng())));
	srv.request.model_xml = string_format(MODEL_SDF, block_name, block_name, block_name,
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

} // namespace controller::world
