#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    // 1. Initialize ROS 2
    rclcpp::init(argc, argv);

    // 2. Create a simple node. The constructor should be lightweight.
    auto node = std::make_shared<rclcpp::Node>("arm_drone_client");

    // 3. Create the service client using the node.
    auto client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    // 4. Wait for the service to be available.
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 1; // Indicate error
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for arming service...");
    }

    // 5. Create the request and send it asynchronously. This returns a "future".
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;
    auto future_result = client->async_send_request(request);

    // 6. Spin the node until the future is complete. This is the key step.
    //    It processes ROS 2 events (like the service response) while waiting.
    if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // 7. Check the result from the service call.
        if (future_result.get()->success) {
            RCLCPP_INFO(node->get_logger(), "Drone armed successfully!");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to arm drone.");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call arming service.");
    }

    // 8. Shut down ROS 2 cleanly.
    rclcpp::shutdown();
    return 0;
}