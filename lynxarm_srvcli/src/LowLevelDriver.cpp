#include "LowLevelDriver.h"

LowLevelDriver::LowLevelDriver(const std::string &port_name, uint32_t baud_rate)
    : serial(ioservice, port_name) // Port name parameter needs integration with HighLevelDriver
{
  // Set serial port options such as baud rate, flow control, parity, stop bits, and character size
  serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate)); // Baud rate parameter needs integration with HighLevelDriver
  serial.set_option(
      boost::asio::serial_port::flow_control(
          boost::asio::serial_port::flow_control ::none));
  serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
  serial.set_option(
      boost::asio::serial_port::character_size(
          boost::asio::serial_port::
              character_size(8)));

  if (!serial.is_open())
  {
    std::cerr << "Serial port is not open." << std::endl;
  }
}

// Destructor: Closes the serial port if it's open
LowLevelDriver::~LowLevelDriver()
{
  command = "";
  emergencyStop();

  if (serial.is_open())
  {
    serial.close();
  }

  while (!commandQueue.empty())
  {
    commandQueue.pop();
  }
}

bool LowLevelDriver::sendSerialMessage(std::string command)
{
  try
  {
    std::cout << " sending serial message: " << command << " : " << std::endl;
    serial.write_some(boost::asio::buffer(command));
    return true;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to send data over serial port: " << e.what() << std::endl;
    std::cerr << "Serial port not open or icorrectly specified" << std::endl;
    return false;
  }
  return false;
}

bool LowLevelDriver::sendServoPositionData(int servo_channel, int pulse_width, int speed, int time)
{
  if (!serial.is_open())
  {
    std::cerr << "Serial port is not open." << std::endl;
    return false;
  }
  std::string command = "#" + std::to_string(servo_channel) + "P" + std::to_string(pulse_width);

  if (speed != 0)
  {
    command += "S" + std::to_string(speed);
  }

  if (time != 0)
  {
    command += "T" + std::to_string(time);
  }

  command += "\r"; // Carriage return

  std::cout << " sending serial message: " << command << " : " << std::endl;
  try
  {
    serial.write_some(boost::asio::buffer(command));
    std::cout << "succes" << std::endl;
    return true;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to send data over serial port: " << e.what() << std::endl;
    return false;
  }
}

bool LowLevelDriver::setServoPositionOffset(int servo_channel, int offset_value)
{
  if (!serial.is_open())
  {
    std::cerr << "Serial port is not open." << std::endl;
    return false;
  }
  clearCommand();
  command += "#";

  if (servo_channel < 0 || servo_channel > 31)
  {
    return false;
  }
  command += std::to_string(servo_channel);

  if (offset_value < -100 || offset_value > 100)
  {
    return false;
  }
  command += " PO" + std::to_string(offset_value);
  return sendQueuedCommands();
}

bool LowLevelDriver::queueCommand(int servo_channel, int pulse_width, int speed, int time)
{
  command += "#";

  if (servo_channel < 0 || servo_channel > 31)
  {
    return false;
  }
  command += std::to_string(servo_channel);
  if (pulse_width < 500 || pulse_width > 2500)
  {
    return false;
  }
  command += "P" + std::to_string(pulse_width);
  if (speed < 1)
  {
    return false;
  }
  command += "S" + std::to_string(speed);
  if (time < 0 || time > 65535)
  {
    return false;
  }
  command += "T" + std::to_string(time);
  return true;
}

bool LowLevelDriver::sendQueuedCommands()
{
  command += "\r"; // Carriage return

  // ch 0-31 pw 500-2500 microseconds spd 0 + microseconds/s ,time microseconds 0-65535, cr ASCII 13
  if (!serial.is_open())
  {
    std::cerr << "Serial port is not open." << std::endl;
    clearCommand();
    return false;
  }

  if (command.length() > 1)
  {
    if (sendSerialMessage(command))
    {
      return true;
    }
  }

  return false;
}

void LowLevelDriver::clearCommand()
{
  command = "";
  return;
}

bool LowLevelDriver::processCommandQueue()
{
  while (!commandQueue.empty())
  {
    sendSerialMessage(commandQueue.front());
    commandQueue.pop();
  }

  if (commandQueue.empty())
  {
    return true;
  }
  else
  {
    return false;
  }
  return false;
}

std::string LowLevelDriver::getCommand() const
{
  return command;
}

bool LowLevelDriver::emergencyStop()
{
  for (int i = 0; i < SERVO_AMOUNT; ++i)
  {
    std::string stopCommand = "STOP";
    stopCommand += std::to_string(i);
    stopCommand += "\r";
    sendSerialMessage(stopCommand);
  }
  return false;
}
