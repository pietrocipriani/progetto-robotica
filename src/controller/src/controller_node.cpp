#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <random>
#include "planner.hpp"

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

constexpr const char* MODEL_SDF = R"(
<?xml version="1.0" ?>
<sdf version="1.4">
<model name="brick_1x1H">
<link name="link">
	<inertial>
		<mass>4.0</mass>
		<inertia>
		  <ixx>0.083</ixx>
		  <ixy>0.0</ixy>
		  <ixz>0.0</ixz>
		  <iyy>0.083</iyy>
		  <iyz>0.0</iyz>
		  <izz>0.083</izz>
		</inertia>
	</inertial>

	<collision name="collision">
		<geometry>
			<mesh>
			<uri>model://brick_1x1H/mesh.stl</uri>
			</mesh>
		</geometry>
	</collision>

	<visual name="visual">
		<geometry>
			<mesh>
			<uri>model://brick_1x1H/mesh.stl</uri>
			</mesh>
		</geometry>
		<material>
			<ambient>%1f %2f %3f %4f</ambient>
			<diffuse>1.0 1.0 1.0 1.0</diffuse>
			<specular>0.1 0.1 0.1 1</specular>
			<emissive>0 0 0 0</emissive>
		</material>
	</visual>
</link>
</model>
</sdf>
)";

int main(int argc, char **argv) {
	planner::BlockPose bp(1,2,3,4);
	bp.collides(bp);

    ros::init(argc, argv, "controller");
    if (argc != 1) {
        ROS_INFO("usage: controller");
        return 1;
    }

	std::random_device dev;
    std::mt19937 rng(dev());

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
	srv.request.model_name = string_format("nome_bellissimo %d", rng());
	srv.request.model_xml = string_format(MODEL_SDF, 1.0, 1.0, 0.0, 1.0);
	srv.request.robot_namespace = "/gazebo/";

	geometry_msgs::Pose pose;
	geometry_msgs::Point point;
	geometry_msgs::Quaternion quaternion;
	point.x = 0.3;
	point.y = 0.3;
	point.z = 1.0;
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = 0.0;
	quaternion.w = 1.0;
	pose.position = point;
	pose.orientation = quaternion;
	srv.request.initial_pose = pose;



    if (client.call(srv)) {
        ROS_INFO("Success: %d status: %s", (int)srv.response.success, srv.response.status_message.c_str());
    } else {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    return 0;
}