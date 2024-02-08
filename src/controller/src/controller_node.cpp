#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <random>
#include "planner.hpp"
#include "model.hpp"

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
			<specular>0.0 0.0 0.0 1</specular>
			<emissive>0 0 0 0</emissive>
		</material>
	</visual>
</link>
</model>
</sdf>
)";

void spawn_block(ros::ServiceClient& client, double x, double y) {
    static std::mt19937 rng = std::mt19937(std::random_device()());

    gazebo_msgs::SpawnModel srv;
	srv.request.model_name = string_format("nome_bellissimo_%d", abs(int(rng())));
	srv.request.model_xml = string_format(MODEL_SDF, 0.45, 0.25, 0.36, 1.0);
	srv.request.robot_namespace = "/gazebo/";

	geometry_msgs::Pose pose;
	geometry_msgs::Point point;
	geometry_msgs::Quaternion quaternion;
	point.x = x;
	point.y = y;
	point.z = 0.88 + 0.05;
	quaternion.x = 1.0;
	quaternion.y = 0.0;
	quaternion.z = 0.0;
	quaternion.w = 0.0;
	pose.position = point;
	pose.orientation = quaternion;
	srv.request.initial_pose = pose;

    if (client.call(srv)) {
        ROS_INFO("Spawned block, success: %d status: %s", (int)srv.response.success, srv.response.status_message.c_str());
    } else {
        ROS_ERROR("Failed to call spawn block service");
    }
}

std_msgs::Float64MultiArray config_to_ros(const model::UR5::Configuration& config, double gripper_pos) {
	std_msgs::Float64MultiArray data;
	const auto& v = config.vector().data();
	ROS_INFO("kinematics %.3lf %.3lf %.3lf - %.3lf %.3lf %.3lf", v[0], v[1], v[2], v[3], v[4], v[5]);
	data.data = std::vector<double>(8, 0.0);
	for (int i=0; i<6; ++i) {
		data.data[i] = v[i];
	}
	//data.data[5] -= M_PI * 2;
	data.data[6] = data.data[7] = gripper_pos;
	return data;
}

void publish_configs(
	ros::Publisher& publisher,
	const model::UR5& robot,
	Time dt,
	planner::MovementSequence::ConfigSequence& q,
	double gripper_pos
) {
	ros::Rate rate(1 / dt);
	while (!q.empty()) {
		publisher.publish(config_to_ros(q.front(), gripper_pos));
		q.pop();
		rate.sleep();
	}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
    if (argc != 1) {
        ROS_INFO("usage: controller");
        return 1;
    }
	ROS_INFO("initialized controller");

    ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<std_msgs::Float64MultiArray>(
		"/ur5/joint_group_pos_controller/command", 1);
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
	ROS_INFO("created publisher and client");

	constexpr Time dt = 0.01;
	constexpr double scale = 0.5;
	model::UR5 robot;

	std::vector<planner::BlockPose> poses{
		planner::BlockPose(0.5, 0.5, 0, 0),
		planner::BlockPose(0.8, 0.6, 0, 0),
		planner::BlockPose(0.6, 0.7, 0, 0),
		planner::BlockPose(0.2, 0.6, 0, 0),
		planner::BlockPose(0.1, 0.4, 0, 0),
		planner::BlockPose(0.8, 0.4, 0, 0),
	};

	// for (int i=0; i<poses.size()-1; i += 1) {
	// 	spawn_block(client, poses[i].pose.linear().x(), poses[i].pose.linear().y());
	// }
	// return 0;

	for (int i=0; i<poses.size()-1; i += 2) {
		if (i%2 == 0) {
			spawn_block(client, poses[i].pose.linear().x(), poses[i].pose.linear().y());
		}

		const planner::BlockMovement movement{
			poses[i],
			poses[i+1]
		};

		auto configs = planner::plan_movement(robot, movement, dt);
		ROS_INFO("movement planned %ld %ld", configs.picking.size(), configs.dropping.size());

		publish_configs(publisher, robot, dt/scale, configs.picking, 1.0);
		publish_configs(publisher, robot, dt/scale, configs.dropping, -0.2);
		ROS_INFO("finished");
	}
}