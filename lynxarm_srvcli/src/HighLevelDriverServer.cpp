#include "HighLevelDriverServer.h"

LynxarmHLD::LynxarmHLD(const rclcpp::NodeOptions &options)
    : Node("lynxarm_hld", options),        // create node
      serialPort("/dev/ttyUSB0"),            // set serial port
      baudRate(115200),                    // set baud rate
      lowLevelDriver(serialPort, baudRate) // create low level driver using initialization
{
  // action
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<Lynx_Command>(
      this,
      "lynxcommand",
      std::bind(&LynxarmHLD::handle_goal, this, _1, _2),
      std::bind(&LynxarmHLD::handle_cancel, this, _1),
      std::bind(&LynxarmHLD::handle_accepted, this, _1));

  this->action_server_servo_ = rclcpp_action::create_server<Servo_Command>(
      this,
      "lynxcommandservo",
      std::bind(&LynxarmHLD::handle_servo_goal, this, _1, _2),
      std::bind(&LynxarmHLD::handle_servo_cancel, this, _1),
      std::bind(&LynxarmHLD::handle_servo_accepted, this, _1));

  // service
  this->service_ = this->create_service<lynxarm_interface::srv::SetServoPosition>(
      "SetServoPosition",
      std::bind(&LynxarmHLD::setServoPosition, this, std::placeholders::_1, std::placeholders::_2));

  // service
  this->stopService_ = this->create_service<lynxarm_interface::srv::Stop>(
      "Stop",
      std::bind(&LynxarmHLD::emergencyBrake, this, std::placeholders::_1, std::placeholders::_2));

  initLynxarm();
}

LynxarmHLD::~LynxarmHLD()
{
  // destructor
}

bool LynxarmHLD::sendSingleServoToPosition(
    uint8_t channel, int64_t degrees,
    int16_t speed, uint64_t time) // in degrees
{                                 /// if action is used for single servo only then
  // Extract values from the request message
  uint8_t servo_channel = channel;
  int64_t pulse_width = degrees;

  if (pulse_width <= 90)
  {
    switch (servo_channel)
    {
    case 0:
      pulse_width = convertToSafePWM(pulse_width, BASE_LEFT, BASE_RIGHT);
      break;
    case 1:
      pulse_width = convertToSafePWM(pulse_width, SHOULDER_BACKWARDS, SHOULDER_HORIZONTAL);
      break;
    case 2:
      pulse_width = convertToSafePWM(pulse_width, ELBOW_STRAIGHT, ELBOW_INWARDS);
      break;
    case 3:
      pulse_width = convertToSafePWM(pulse_width, WRIST_DOWN, WRIST_UP);
      break;
    case 4:
      pulse_width = convertToSafePWM(pulse_width, GRIPPER_CLOSED, GRIPPER_OPEN);
      break;
    case 5:
      pulse_width = convertToSafePWM(pulse_width, WRIST_ROTATE_LEFT, WRIST_ROTATE_RIGHT);
      break;
    default:
      RCLCPP_ERROR(
          rclcpp::get_logger(
              "rclcpp"),
          "Invalid servo channel. Please enter a channel between 0 and 5");
      return false;
    }
  }
  else if (pulse_width > 90 && pulse_width < 500)
  {
    RCLCPP_ERROR(
        rclcpp::get_logger(
            "rclcpp"),
        "Invalid pulse width value or angle in degrees. Please enter a degree between -90 and 90 or a PWM between 500 and 2500");
    return false;
  }

  // Log the incoming request
  RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"), "STATE:{MOVING} Incoming request:\n"
                                    "servo_channel: %d\n"
                                    "pulse_width: %ld\n"
                                    "speed: %d\n"
                                    "time: %ld\n",
      servo_channel, pulse_width, speed, time);

  // Send position data over the serial port
  bool success = lowLevelDriver.sendServoPositionData(servo_channel, pulse_width, speed, time);

  return success;
}

bool LynxarmHLD::sendToPredefinedPosition(std::string command)
{
  // switch command to enum
  // send the predefined position
  //   LowLevelDriver lowLevelDriver( serialPort, 115200);
  //   lowLevelDriver.sendPredefinedPosition();
  command = "warning fix filler string";
  return true;
}

bool LynxarmHLD::initLynxarm()
{
  return sendPredefinedPosition(PARK);
}

rclcpp_action::GoalResponse LynxarmHLD::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const lynxarm_interface::action::PredefinedCommandAction::Goal> goal)
{
  RCLCPP_INFO(
      this->get_logger(), "STATE:{MOVING} Received goal request with command: %s ",
      goal->command.c_str());
  (void)uuid;
  bool success = false;

  // Check if the received command is "park"
  if (goal->command == "park")
  {
    max_time_ = TIME_PARK_IN_US;
    // Send the predefined position "PARK" using LowLevelDriver
    success = sendPredefinedPosition(PARK);
  }
  else if (goal->command == "ready")
  {
    max_time_ = TIME_READY_IN_US;
    // Send the predefined position "PARK" using LowLevelDriver
    success = sendPredefinedPosition(READY);
  }
  else if (goal->command == "straight_up")
  {
    max_time_ = TIME_STRAIGHTUP_IN_US;
    // Send the predefined position "PARK" using LowLevelDriver
    success = sendPredefinedPosition(STRAIGHTUP);
  }
  else
  {
    // Handle other commands here if needed
    // If the received command is not recognized, reject the goal
    RCLCPP_ERROR(this->get_logger(), "Received unsupported command: %s", goal->command.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (success)
  {
    RCLCPP_INFO(
        this->get_logger(),
        "Sent 'park' command to LowLevelDriver for predefined position.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send 'park' command to LowLevelDriver.");
    return rclcpp_action::GoalResponse::REJECT;
  }
}

rclcpp_action::CancelResponse LynxarmHLD::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Lynx_Command>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "STATE:{MOVED} Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LynxarmHLD::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Lynx_Command>> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&LynxarmHLD::execute, this, _1), goal_handle}.detach();
}

void LynxarmHLD::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  // Log a message indicating that the goal is being executed
  RCLCPP_INFO(this->get_logger(), "STATE:{MOVING} Executing goal");

  // // Create a rate object to control the loop frequency
  // rclcpp::Rate loop_rate(1);

  // Get the goal and create shared pointers for feedback and result
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Lynx_Command::Feedback>();
  auto result = std::make_shared<Lynx_Command::Result>();
  auto start_time = std::chrono::high_resolution_clock::now();
  auto current_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time =
      std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

  while (elapsed_time <= max_time_)
  {
    current_time = std::chrono::high_resolution_clock::now();
    if (goal_handle->is_canceling())
    {
      // Set the result as unsuccessful and cancel the goal
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      break;
    }
    elapsed_time =
        std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

    // Update sequence (not shown in the code)
    feedback->current_time = elapsed_time;
    // Publish feedback to the client
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(
        this->get_logger(), "STATE:{MOVING}, publish feedback: %ld",
        feedback->current_time);
  }

  // Check if the goal is done
  if (rclcpp::ok())
  {
    // Set the result as successful and mark the goal as succeeded
    result->success = true;
    result->end_time = (feedback->current_time) / 1000;
    goal_handle->succeed(result);
    RCLCPP_INFO(
        this->get_logger(), "STATE:{MOVED}, Goal succeeded, arm is now in %s mode in %ld ms",
        goal->command.c_str(), (result->end_time));
  }
}

int LynxarmHLD::convertToDegrees(int pwm)
{
  // Ensure PWM is within the valid range of 500 to 2500
  int valid_pwm = std::min(std::max(pwm, 500), 2500);

  // Map the valid_pwm to the degree range of -90 - 90
  int degrees = ((valid_pwm - 500) * 180) / (2500 - 500);

  return degrees;
}

int LynxarmHLD::convertToPWM(int degrees)
{
  // Ensure degrees is within the valid range of -90 - 90
  int valid_degrees = std::min(std::max(degrees, -90), 90);

  // Map the valid_degrees to the PWM range of 500 to 2500
  int pwm = 500 + (((valid_degrees + 90) * (2500 - 500)) / 180); // 90 + 90

  return pwm;
}

int64_t LynxarmHLD::convertToSafePWM(int64_t value, int64_t minRange, int64_t maxRange)
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "orginal value: " << value << std::endl);
  // Ensure degrees is within the valid range of -90 - 90
  int valid_degrees = std::min(std::max(value, minRange), maxRange);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "valid value: " << valid_degrees << std::endl);

  int valid_pwm = convertToPWM(valid_degrees);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "valid pwm: " << valid_pwm << std::endl);

  return valid_pwm;
}

void LynxarmHLD::setServoPosition(
    /// CHANGE TO ACTION IT TAKES TIME TO MOVE
    const std::shared_ptr<lynxarm_interface::srv::SetServoPosition::Request> request,
    std::shared_ptr<lynxarm_interface::srv::SetServoPosition::Response> response)
{
  // Extract values from the request message
  int64_t servo_channel = request->servo_channel;
  int64_t pulse_width = request->pulse_width;
  int64_t speed = request->speed;
  int64_t time = request->time;

  if (pulse_width <= 90)
  {
    switch (servo_channel)
    {
    case 0:
      pulse_width = convertToSafePWM(pulse_width, BASE_LEFT, BASE_RIGHT);
      break;
    case 1:
      pulse_width = convertToSafePWM(pulse_width, SHOULDER_BACKWARDS, SHOULDER_HORIZONTAL);
      break;
    case 2:
      pulse_width = convertToSafePWM(pulse_width, ELBOW_STRAIGHT, ELBOW_INWARDS);
      break;
    case 3:
      pulse_width = convertToSafePWM(pulse_width, WRIST_DOWN, WRIST_UP);
      break;
    case 4:
      pulse_width = convertToSafePWM(pulse_width, GRIPPER_CLOSED, GRIPPER_OPEN);
      break;
    case 5:
      pulse_width = convertToSafePWM(pulse_width, WRIST_ROTATE_LEFT, WRIST_ROTATE_RIGHT);
      break;
    default:
      RCLCPP_ERROR(
          rclcpp::get_logger(
              "rclcpp"),
          "Invalid servo channel. Please enter a channel between 0 and 5");
      return;
    }
  }
  else if (pulse_width > 90 && pulse_width < 500)
  {
    RCLCPP_ERROR(
        rclcpp::get_logger(
            "rclcpp"),
        "Invalid pulse width value or angle in degrees. Please enter a degree between -90 and 90 or a PWM between 500 and 2500");
    return;
  }

  // Log the incoming request
  RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"), "STATE:{MOVING} Incoming request:\n"
                                    "servo_channel: %ld\n"
                                    "pulse_width: %ld\n"
                                    "speed: %ld\n"
                                    "time: %ld\n",
      servo_channel, pulse_width, speed, time);

  // Send position data over the serial port
  bool success = lowLevelDriver.sendServoPositionData(servo_channel, pulse_width, speed, time);

  // Set the response message based on the success of sending the position data
  response->success = success;
}

bool LynxarmHLD::sendPredefinedPosition(predefined_positions position)
{
  switch (position)
  {
  case 0:
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is currently: " << lowLevelDriver.getCommand() << std::endl);

    lowLevelDriver.queueCommand(0, convertToSafePWM(BASE_ROTOR_PARK, BASE_LEFT, BASE_RIGHT));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(
        1,
        convertToSafePWM(SHOULDER_PARK, SHOULDER_BACKWARDS, SHOULDER_HORIZONTAL));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(2, convertToSafePWM(ELBOW_PARK, ELBOW_STRAIGHT, ELBOW_INWARDS));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(3, convertToSafePWM(WRIST_PARK, WRIST_DOWN, WRIST_UP));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(4, convertToSafePWM(GRIPPER_PARK, GRIPPER_CLOSED, GRIPPER_OPEN));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(
        5,
        convertToSafePWM(WRIST_ROTATE_PARK, WRIST_ROTATE_LEFT, WRIST_ROTATE_RIGHT), 0, TIME_PARK_IN_US);
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.sendQueuedCommands();
    lowLevelDriver.clearCommand();
    return true;
  case 1:
    lowLevelDriver.queueCommand(0, convertToSafePWM(BASE_ROTOR_READY, BASE_LEFT, BASE_RIGHT));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(
        1,
        convertToSafePWM(SHOULDER_READY, SHOULDER_BACKWARDS, SHOULDER_HORIZONTAL));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(2, convertToSafePWM(ELBOW_READY, ELBOW_STRAIGHT, ELBOW_INWARDS));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(3, convertToSafePWM(WRIST_READY, WRIST_DOWN, WRIST_UP));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(4, convertToSafePWM(GRIPPER_READY, GRIPPER_CLOSED, GRIPPER_OPEN));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(
        5,
        convertToSafePWM(WRIST_ROTATE_READY, WRIST_ROTATE_LEFT, WRIST_ROTATE_RIGHT), 0, TIME_READY_IN_US);
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.sendQueuedCommands();
    lowLevelDriver.clearCommand();
    return true;
  case 2:
    lowLevelDriver.queueCommand(
        0,
        convertToSafePWM(BASE_ROTOR_STRAIGHTUP, BASE_LEFT, BASE_RIGHT));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(
        1,
        convertToSafePWM(SHOULDER_STRAIGHTUP, SHOULDER_BACKWARDS, SHOULDER_HORIZONTAL));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(
        2,
        convertToSafePWM(ELBOW_STRAIGHTUP, ELBOW_STRAIGHT, ELBOW_INWARDS));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(3, convertToSafePWM(WRIST_STRAIGHTUP, WRIST_DOWN, WRIST_UP));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(
        4,
        convertToSafePWM(GRIPPER_STRAIGHTUP, GRIPPER_CLOSED, GRIPPER_OPEN));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.queueCommand(
        5,
        convertToSafePWM(WRIST_ROTATE_STRAIGHTUP, WRIST_ROTATE_LEFT, WRIST_ROTATE_RIGHT), 0, TIME_STRAIGHTUP_IN_US);
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "command is extended, currently: " << lowLevelDriver.getCommand() << std::endl);
    lowLevelDriver.sendQueuedCommands();
    lowLevelDriver.clearCommand();
    return true;
  default:
    std::cerr << "Invalid predefined position." << std::endl;
    return false;
  }
}

rclcpp_action::GoalResponse LynxarmHLD::handle_servo_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const lynxarm_interface::action::SingleServoCommand::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(
      this->get_logger(), "STATE:{MOVING} Received goal request with values: servo: %d goal position in degrees: %d, speed in %d, time %d ",
      goal->servo, goal->position, goal->speed, goal->time);


    RCLCPP_INFO(
        this->get_logger(),
        "Sent servo to LowLevelDriver for predefined position.");
    sendSingleServoToPosition(goal->servo, goal->position, goal->speed, goal->time);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse LynxarmHLD::handle_servo_cancel(
    const std::shared_ptr<GoalHandleServo> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "STATE:{MOVED} Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LynxarmHLD::handle_servo_accepted(const std::shared_ptr<GoalHandleServo> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&LynxarmHLD::execute_servo, this, _1), goal_handle}.detach();
}

void LynxarmHLD::execute_servo(const std::shared_ptr<GoalHandleServo> goal_handle)
{
  // Log a message indicating that the goal is being executed
  RCLCPP_INFO(this->get_logger(), "STATE:{MOVING} Executing goal");

  // Get the goal and create shared pointers for feedback and result
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Servo_Command::Feedback>();
  auto result = std::make_shared<Servo_Command::Result>();
  auto start_time = std::chrono::high_resolution_clock::now();
  auto current_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time =
      std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

  while (elapsed_time <= goal->time)
  {
    current_time = std::chrono::high_resolution_clock::now();
    if (goal_handle->is_canceling())
    {
      // Set the result as unsuccessful and cancel the goal
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      break;
    }
    elapsed_time =
        std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

    // Update sequence (not shown in the code)
    feedback->current_time = elapsed_time;
    // Publish feedback to the client
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(
        this->get_logger(), "STATE:{MOVING}, publish feedback: %ld",
        feedback->current_time);
  }

  // Check if the goal is done
  if (rclcpp::ok())
  {
    // Set the result as successful and mark the goal as succeeded
    result->success = true;
    result->end_time = feedback->current_time;
    goal_handle->succeed(result);
    RCLCPP_INFO(
        this->get_logger(), "STATE:{MOVED}, Goal succeeded, servo %d is now in position %d, in %ld ms", goal->servo, goal->position,
        result->end_time);
  }
}

void LynxarmHLD::emergencyBrake(
    const std::shared_ptr<lynxarm_interface::srv::Stop::Request> request,
    std::shared_ptr<lynxarm_interface::srv::Stop::Response> response)
{
  (void)request;
  (void)response;
  lowLevelDriver.emergencyStop();
}

// Main function
int main(int argc, char **argv)
{
  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);

  // Log a message indicating readiness to set Servo position
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STATE:{WAITING} Ready to set Servo position.");

  // Spin the node
  rclcpp::spin(std::make_shared<LynxarmHLD>());

  // Shutdown the node
  rclcpp::shutdown();

  return 0;
}
