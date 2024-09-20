#include "rclcpp/rclcpp.hpp"                            // Include ROS 2 C++ client library
#include "lynxarm_interface/srv/set_servo_position.hpp" // Include the service message header file
#include "lynxarm_interface/srv/stop.hpp"

#include <chrono>  // Include for time literals
#include <cstdlib> // Include for std::atoll
#include <memory>  // Include for smart pointers

using namespace std::chrono_literals; // Using chrono literals for time durations

// Main function
int main(int argc, char **argv)
{
  // variables
  bool isDefaultSpeed = false;
  bool isDefaultTime = false;

  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);

  // Create a shared pointer to the node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_servo_position_client");

  // Create a client to send requests to set arm position
  rclcpp::Client<lynxarm_interface::srv::Stop>::SharedPtr stopClient = node->create_client<lynxarm_interface::srv::Stop>("Stop");

  // Create a request message
  auto emergencyStop = std::make_shared<lynxarm_interface::srv::Stop::Request>();

  // Check if the correct number of command line arguments are provided
  if (argc < 1 || argc > 5)
  {
    // Print usage message if incorrect number of arguments
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: servo_channel pulse_width speed time");
    return 1;
  }
  else
  {
    if (argc == 1)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "!! EMERGENCY STOP CALLED !!");
      // Send the emergencyStop request to set arm asynchronously
      auto result = stopClient->async_send_request(emergencyStop);
      return 0;
    }
    else if (argc == 3) // condition is true if only 2 arguments given, (no sure why it need to be 3)
    {
      isDefaultSpeed = true;
      isDefaultTime = true;
    }
    else if (argc == 4) // condition is true if only 3 arguments given, (no sure why it need to be 4)
    {
      isDefaultTime = true;
    }
  }


  // Create a client to send requests to set arm position
  rclcpp::Client<lynxarm_interface::srv::SetServoPosition>::SharedPtr client = node->create_client<lynxarm_interface::srv::SetServoPosition>("SetServoPosition");

  // Create a request message
  auto request = std::make_shared<lynxarm_interface::srv::SetServoPosition::Request>();

  // Parse command line arguments and assign values to request fields
  request->servo_channel = atoll(argv[1]);
  request->pulse_width = atoll(argv[2]);

  if (isDefaultSpeed)
  {
    request->speed = 1;
  }
  else
  {
    request->speed = atoll(argv[3]);
  }

  if (isDefaultTime)
  {
    request->time = 26;
  }
  else
  {
    request->time = atoll(argv[4]);
  }

  // Wait for the service to become available
  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      // Log an error message if interrupted while waiting
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    // Log a message indicating service is not available yet
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STATE:{WAITING} Service not available, waiting again...");
  }

  // Send the request to set arm position asynchronously
  auto result = client->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    // Check if arm position was set successfully
    if (result.get()->success)
    {
      // Log a message indicating successful arm position set
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STATE:{WAITING} Servo position set successfully!");
    }
    else
    {
      // Log an error message if failed to set arm position
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set Servo position.");
    }
  }
  else
  {
    // Log an error message if failed to call service to set arm position
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service SetServoPosition");
  }

  // Shutdown the node
  rclcpp::shutdown();

  return 0;
}
