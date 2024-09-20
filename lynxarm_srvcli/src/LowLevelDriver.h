/**
 * @file LowLevelDriver.h
 * @brief Contains the declaration of the LowLevelDriver class.
 * 
 * The LowLevelDriver class provides functionality for sending serial messages, controlling servo positions,
 * setting position offsets, queuing commands, and processing the command queue. It uses the Boost.Asio library
 * for asynchronous I/O operations and provides an interface for interacting with a robot arm.
 * 
 *  @author Sonny Selten
 *  @author Elviana Cornelissen
 *  @version 1.0
 *  @date 2024-03-07 
 */

#ifndef LOW_LEVEL_DRIVER_H
#define LOW_LEVEL_DRIVER_H

#include <iostream>       
#include <string>             // Provides a string class.
#include <queue>              // Provides a queue data structure.
#include <boost/asio.hpp>     // Provides a cross-platform framework for asynchronous I/O operations.
#include "Configuration.hpp"  // Contains the configuration parameters for the low-level driver.

/**
 * @class LowLevelDriver
 * @brief Represents a low-level driver for communication with a robot arm.
 *
 * The LowLevelDriver class provides functionality for sending serial messages, controlling servo positions,
 * setting position offsets, queuing commands, and processing the command queue. It uses the Boost.Asio library
 * for asynchronous I/O operations and provides an interface for interacting with a robot arm.
 */
class LowLevelDriver
{
public:
  /**
   * @brief Initialize the LowLevelDriver object with the specified parameters.
   *
   * @param port_name The name of the serial port.
   * @param baud_rate The baud rate for serial communication.
   */
  LowLevelDriver(const std::string &port_name, uint32_t baud_rate);

  /**
   * @brief Destructor.
   * Closes the serial port if it's open.
   */
  ~LowLevelDriver();

  /**
   * @brief Sends a serial message to the low-level driver.
   *
   * This function sends a serial message to the low-level driver for communication with the robot arm.
   *
   * @param command The serial command to be sent.
   * @return True if the message was successfully sent, false otherwise.
   */
  bool sendSerialMessage(std::string command);

  /**
   * @brief Sends servo position data to the specified servo channel.
   *
   * This function is used to send servo position data to the specified servo channel.
   * It allows setting the pulse width, speed, and time for the servo movement.
   *
   * @param servo_channel The channel number of the servo.
   * @param pulse_width The desired pulse width for the servo.
   * @param speed The speed at which the servo should move (optional, default = 0).
   * @param time The time duration for the servo movement (optional, default = 0).
   * @return true if the servo position data was successfully sent, false otherwise.
   */
  bool sendServoPositionData(int servo_channel, int pulse_width, int speed = 0, int time = 0);

  /**
   * @brief Sets the position offset for a servo channel.
   *
   * This function sets the position offset for the specified servo channel.
   *
   * @param servo_channel The channel of the servo.
   * @param offset_value The offset value to be set.
   * @return True if the position offset was set successfully, false otherwise.
   */
  bool setServoPositionOffset(int servo_channel, int offset_value);

  /**
   * @brief Queues a command to control a servo channel.
   *
   * This function allows you to queue a command to control a servo channel with a specific pulse width, speed, and time at the same time.
   *
   * @param servo_channel The servo channel to control.
   * @param pulse_width The desired pulse width for the servo channel.
   * @param speed The desired speed for the servo movement (default: 0).
   * @param time The desired time for the servo movement (default: 0).
   * @return True if the command was successfully queued, false otherwise.
   */
  bool queueCommand(int servo_channel, int pulse_width, int speed = 0, int time = 0);

  /**
   * @brief Sends the queued commands to the low-level driver.
   *
   * This function sends the commands that have been queued up to the low-level driver for execution.
   *
   * @return True if the commands were successfully sent, false otherwise.
   */
  bool sendQueuedCommands();

  /**
   * @brief Clears the command for the low-level driver.
   */
  void clearCommand();

  /**
   * @brief Processes the command queue.
   * 
   * This function processes the command queue and executes the commands in the order they were received.
   * 
   * @return True if the command queue was successfully processed, false otherwise.
   */
  bool processCommandQueue();

  /**
   * @brief Gets the command.
   * 
   * @return The command as a std::string.
   */
  std::string getCommand() const;

  /**
   * @brief Stops the robot arm immediately in case of an emergency.
   * 
   * @return true if the emergency stop was successful, false otherwise.
   */
  bool emergencyStop();

private:
  /**
   * @brief Boost asio io_service for handling asynchronous operations.
   */
  boost::asio::io_service ioservice;

  /**
   * @brief Serial port object for communication.
   * 
   * This object represents a serial port used for communication. It is part of the Boost.Asio library,
   * which provides a cross-platform framework for asynchronous I/O operations. The serial port object
   * allows reading from and writing to a serial port, enabling communication with external devices.
   */
  boost::asio::serial_port serial; 

  /**
   * @brief The command to be sent over serial.
   */
  std::string command = ""; 

  /**
   * @brief Queue for storing commands to be sent.
   * 
   * This queue is used to store commands that need to be sent by the LowLevelDriver.
   * The commands are stored as strings.
   */
  std::queue<std::string> commandQueue;
};

#endif // LOW_LEVEL_DRIVER_H
