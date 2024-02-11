#include "controller/world/deleter.hpp"

#include <gazebo_msgs/DeleteModel.h>

#include "controller/world/names.hpp"

namespace controller::world {

Deleter::Deleter(ros::ServiceClient && _client) : client{_client} {}

void Deleter::delete_model(const std::string& model_name) {
    gazebo_msgs::DeleteModel srv;
    srv.request.model_name = model_name;

    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Deleted model %s, success: %d status: %s", model_name.c_str(),
                (int)srv.response.success, srv.response.status_message.c_str());
        } else {
            ROS_ERROR("Error deleting model %s, success: %d status: %s", model_name.c_str(),
                (int)srv.response.success, srv.response.status_message.c_str());
        }
    } else {
        ROS_ERROR("Error deleting model %s, failed to call service", model_name.c_str());
    }
}

void Deleter::delete_block(planner::Block block_type) {
    delete_model(planner::get_name(block_type));
}

void Deleter::delete_pad(planner::Block block_type) {
    delete_model(std::string(PAD_PREFIX) + planner::get_name(block_type));
}

} // namespace controller::world