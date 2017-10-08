#include <ros/ros.h>
// Standard ROS headers
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// Specific services

// Specific dynamic reconfigure parameters

// Standard C/C++ headers
#include <math.h>
#include <stdlib.h>
#include <cmath>
#include <list>
#include <vector>

#define USE_BOOST

#ifdef USE_BOOST
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#else
#include <errno.h>    // Error number definitions
#include <fcntl.h>    // File control definitions
#include <stdio.h>    // standard input / output functions
#include <string.h>   // string function definitions
#include <termios.h>  // POSIX terminal control definitionss
#include <time.h>     // time calls
#include <unistd.h>   // UNIX standard function definitions

#endif

/**
 *	Class RoboClaw
 *	Handles navigation on track
 */
class RoboClaw
{
public:
  RoboClaw();
  ~RoboClaw()
  {
  }

  // Public Functions
  void compute(void);

private:
  // Subscribers
  void linearSpeedCallback(const std_msgs::Float32::ConstPtr& msg);
  void angularSpeedCallback(const std_msgs::Float32::ConstPtr& msg);
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);

  // Services

  // Dynamic reconfigure

  // Private Functions
  // Used to change the speed value of motor 1
  void write_RoboClaw_forward_M1(char addr, int32_t speed);
  // Used to change the speed value of motor 1
  void write_RoboClaw_backward_M1(char addr, int32_t speed);
  // Used to change the speed value of motor 2
  void write_RoboClaw_forward_M2(char addr, int32_t speed);
  // Used to change the speed value of motor 2
  void write_RoboClaw_backward_M2(char addr, int32_t speed);
  // Used to change the speed value of motor 2
  void write_RoboClaw_drive_M1(char addr, int32_t speed);
  // Used to change the speed value of motor 2
  void write_RoboClaw_drive_M2(char addr, int32_t speed);
  /* With ENCODERS */
  // of motor 1
  void write_RoboClaw_PID_M1(char addr, int32_t D, int32_t P, int32_t I, int32_t QQPS);
  // of motor 2
  void write_RoboClaw_PID_M2(char addr, int32_t D, int32_t P, int32_t I, int32_t QQPS);
  // Used to change the speed value of motor 1
  void write_RoboClaw_speed_M1(char addr, int32_t speed);
  // Used to change the speed value of motor 2
  void write_RoboClaw_speed_M2(char addr, int32_t speed);
  // Used to change the speed value of motors 1 and 2
  void write_RoboClaw_speed_M1M2(char addr, int32_t speedM1, int32_t speedM2);
  // Used to change the speed value of motor 1 and 2 during a specific distance
  void write_RoboClaw_speed_dist_M1M2(char addr, int32_t speedM1, int32_t distanceM1, int32_t speedM2,
                                      int32_t distanceM2);

  ros::NodeHandle nh;

  ros::Subscriber _sub_linear_speed;
  ros::Subscriber _sub_angular_speed;
  ros::Subscriber _sub_twist;

  std::string _port;  ///< @brief The serial port the driver is attached to
#ifdef USE_BOOST
  uint32_t baud_rate_;  ///< @brief The baud rate for the serial connection

  boost::shared_ptr<boost::asio::serial_port> serial_;

  boost::asio::io_service io;
  boost::array<uint8_t, 16> raw_bytes_;
  boost::array<uint8_t, 2> speed_;
#else
  int fd;                        // file description for the serial port
  struct termios port_settings;  // structure to store the port settings in
#endif

  int _port_number1;
  int _port_number2;

  double _linear_value_x;
  double _linear_value_y;
  double _angular_value;
  double _lin_coeff;
  double _ang_coeff;

  int _motor1_side, _motor2_side;
  int _motor3_side, _motor4_side;
  bool _inverted_angular_rotation;
};

RoboClaw::RoboClaw()
{
  ros::NodeHandle nhp("~");

  nhp.param<std::string>("usb_port", _port, "/dev/ttyUSB0");
  nhp.param<int>("serial_port_number1", _port_number1, 128);
  nhp.param<int>("serial_port_number2", _port_number2, 129);

  int P, I, D, Q;
  nhp.param<int>("P", P, 150384);
  nhp.param<int>("I", I, 80005);
  nhp.param<int>("D", D, 60076);
  nhp.param<int>("Q", Q, 11000);

  nhp.param<double>("linear_coeff", _lin_coeff, 1000.0);
  nhp.param<double>("angular_coeff", _ang_coeff, 1000.0);

  nhp.param<int>("motor1_side", _motor1_side, 1);
  nhp.param<int>("motor2_side", _motor2_side, 1);
  nhp.param<int>("motor3_side", _motor3_side, 1);
  nhp.param<int>("motor4_side", _motor4_side, 1);

  nhp.param<bool>("inverted_angular_rotation", _inverted_angular_rotation, false);

  ros::NodeHandle nhe("");

  // nhe.param<double>("distance_point_backward", _distance_point_backward, 1.0);

  _sub_linear_speed = nh.subscribe<std_msgs::Float32>("linear_speed", 2, &RoboClaw::linearSpeedCallback, this);
  _sub_angular_speed = nh.subscribe<std_msgs::Float32>("angular_speed", 2, &RoboClaw::angularSpeedCallback, this);
  _sub_twist = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 2, &RoboClaw::twistCallback, this);

#ifdef USE_BOOST
  baud_rate_ = 38400;

  try
  {
    // boost::asio::serial_port serial_(io, port_);
    serial_ = boost::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(io, _port));
    serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    // boost::asio::write(serial_,boost::asio::buffer(sendThis,1));
    serial_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  }
  catch (boost::system::system_error ex)
  {
    ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s",
              ex.what());
    // return -1;
  }
#else

  // fd = open("/dev/ttyROBOCLAW", O_RDWR | O_NOCTTY | O_NDELAY);
  fd = open(_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd == -1)  // if open is unsucessful
  {
    // perror("open_port: Unable to open /dev/ttyS0 - ");
    printf("open_port: Unable to open /dev/ttyUSB0. \n");
  }
  else
  {
    fcntl(fd, F_SETFL, 0);
    printf("port is open.\n");
  }

  cfsetispeed(&port_settings, B38400);  // set baud rates
  cfsetospeed(&port_settings, B38400);

  port_settings.c_cflag &= ~PARENB;  // set no parity, stop bits, data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;

  tcsetattr(fd, TCSANOW, &port_settings);  // apply the settings to the port
#endif

  write_RoboClaw_PID_M1(_port_number1, P, I, D, Q);
  write_RoboClaw_PID_M2(_port_number1, P, I, D, Q);
  write_RoboClaw_PID_M1(_port_number2, P, I, D, Q);
  write_RoboClaw_PID_M2(_port_number2, P, I, D, Q);
}

void RoboClaw::compute(void)
{
}

/********************/
/********************/
/* TOPICS CALLBACKS */
/********************/
/********************/

void RoboClaw::linearSpeedCallback(const std_msgs::Float32::ConstPtr& msg)
{
  double speed_motor1;
  double speed_motor2;
  double speed_motor3;
  double speed_motor4;

  _linear_value_x = msg->data * _lin_coeff;

  speed_motor1 = _linear_value_x - _linear_value_y + _angular_value;
  speed_motor2 = _linear_value_x + _linear_value_y - _angular_value;
  speed_motor3 = _linear_value_x - _linear_value_y - _angular_value;
  speed_motor4 = _linear_value_x + _linear_value_y + _angular_value;

  write_RoboClaw_speed_M1M2(_port_number1, int32_t(_motor1_side * speed_motor1), int32_t(_motor2_side * speed_motor2));
  write_RoboClaw_speed_M1M2(_port_number2, int32_t(_motor3_side * speed_motor3), int32_t(_motor4_side * speed_motor4));
}

void RoboClaw::angularSpeedCallback(const std_msgs::Float32::ConstPtr& msg)
{
  double speed_motor1;
  double speed_motor2;
  double speed_motor3;
  double speed_motor4;

  if (_inverted_angular_rotation)
  {
    _angular_value = -msg->data * _ang_coeff;
  }
  else
  {
    _angular_value = msg->data * _ang_coeff;
  }

  speed_motor1 = _linear_value_x - _linear_value_y + _angular_value;
  speed_motor2 = _linear_value_x + _linear_value_y - _angular_value;
  speed_motor3 = _linear_value_x - _linear_value_y - _angular_value;
  speed_motor4 = _linear_value_x + _linear_value_y + _angular_value;

  write_RoboClaw_speed_M1M2(_port_number1, int32_t(_motor1_side * speed_motor1), int32_t(_motor2_side * speed_motor2));
  write_RoboClaw_speed_M1M2(_port_number2, int32_t(_motor3_side * speed_motor3), int32_t(_motor4_side * speed_motor4));
}

void RoboClaw::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double speed_motor1;
  double speed_motor2;
  double speed_motor3;
  double speed_motor4;

  _linear_value_x = msg->linear.x * _lin_coeff;
  _linear_value_y = msg->linear.y * _lin_coeff;

  if (_inverted_angular_rotation)
  {
    _angular_value = -msg->angular.z * _ang_coeff;
  }
  else
  {
    _angular_value = msg->angular.z * _ang_coeff;
  }

  speed_motor1 = _linear_value_x - _linear_value_y + _angular_value;
  speed_motor2 = _linear_value_x + _linear_value_y - _angular_value;
  speed_motor3 = _linear_value_x - _linear_value_y - _angular_value;
  speed_motor4 = _linear_value_x + _linear_value_y + _angular_value;

  write_RoboClaw_speed_M1M2(_port_number1, int32_t(_motor1_side * speed_motor1), int32_t(_motor2_side * speed_motor2));
  write_RoboClaw_speed_M1M2(_port_number2, int32_t(_motor3_side * speed_motor3), int32_t(_motor4_side * speed_motor4));
}

/**********************/
/**********************/
/* SERVICES CALLBACKS */
/**********************/
/**********************/

/***********************/
/* ROBO CLAW FUNCTIONS */
/***********************/

/* Without ENCODERS */

// Used to change the speed value of motor 1
void RoboClaw::write_RoboClaw_forward_M1(char addr, int32_t speed)
{
  char checkSUM;
  checkSUM = (addr + 0 + ((char)(speed & 0xFF))) & 0x7F;

  unsigned char commands[4];
  commands[0] = addr;
  commands[1] = 0;
  commands[2] = ((char)(speed & 0xFF));
  commands[3] = checkSUM;
#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 4));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 4);                  // Send data
#endif
}

// Used to change the speed value of motor 1
void RoboClaw::write_RoboClaw_backward_M1(char addr, int32_t speed)
{
  char checkSUM;
  checkSUM = (addr + 1 + ((char)(speed & 0xFF))) & 0x7F;

  unsigned char commands[4];
  commands[0] = addr;
  commands[1] = 1;
  commands[2] = ((char)(speed & 0xFF));
  commands[3] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 4));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 4);                  // Send data
#endif
}

// Used to change the speed value of motor 2
void RoboClaw::write_RoboClaw_forward_M2(char addr, int32_t speed)
{
  char checkSUM;
  checkSUM = (addr + 4 + ((char)(speed & 0xFF))) & 0x7F;

  unsigned char commands[4];
  commands[0] = addr;
  commands[1] = 4;
  commands[2] = ((char)(speed & 0xFF));
  commands[3] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 4));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 4);                  // Send data
#endif
}

// Used to change the speed value of motor 2
void RoboClaw::write_RoboClaw_backward_M2(char addr, int32_t speed)
{
  char checkSUM;
  checkSUM = (addr + 5 + ((char)(speed & 0xFF))) & 0x7F;

  unsigned char commands[4];
  commands[0] = addr;
  commands[1] = 5;
  commands[2] = ((char)(speed & 0xFF));
  commands[3] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 4));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 4);                  // Send data
#endif
}

// Used to change the speed value of motor 2
void RoboClaw::write_RoboClaw_drive_M1(char addr, int32_t speed)
{
  char checkSUM;
  checkSUM = (addr + 6 + ((char)(speed & 0xFF))) & 0x7F;

  unsigned char commands[4];
  commands[0] = addr;
  commands[1] = 6;
  commands[2] = ((char)(speed & 0xFF));
  commands[3] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 4));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 4);                  // Send data
#endif
}

// Used to change the speed value of motor 2
void RoboClaw::write_RoboClaw_drive_M2(char addr, int32_t speed)
{
  char checkSUM;
  checkSUM = (addr + 7 + ((char)(speed & 0xFF))) & 0x7F;

  unsigned char commands[4];
  commands[0] = addr;
  commands[1] = 7;
  commands[2] = ((char)(speed & 0xFF));
  commands[3] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 4));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 4);                  // Send data
#endif
}

/* With ENCODERS */

// of motor 1
void RoboClaw::write_RoboClaw_PID_M1(char addr, int32_t D, int32_t P, int32_t I, int32_t QQPS)
{
  char checkSUM;
  checkSUM = (addr + 28 + ((char)((D >> 24) & 0xFF)) + ((char)((D >> 16) & 0xFF)) + ((char)((D >> 8) & 0xFF)) +
              ((char)(D & 0xFF)) + ((char)((P >> 24) & 0xFF)) + ((char)((P >> 16) & 0xFF)) + ((char)((P >> 8) & 0xFF)) +
              ((char)(P & 0xFF)) + ((char)((I >> 24) & 0xFF)) + ((char)((I >> 16) & 0xFF)) + ((char)((I >> 8) & 0xFF)) +
              ((char)(I & 0xFF)) + ((char)((QQPS >> 24) & 0xFF)) + ((char)((QQPS >> 16) & 0xFF)) +
              ((char)((QQPS >> 8) & 0xFF)) + ((char)(QQPS & 0xFF))) &
             0x7F;

  unsigned char commands[19];
  commands[0] = addr;
  commands[1] = 28;
  commands[2] = ((char)((D >> 24) & 0xFF));
  commands[3] = ((char)((D >> 16) & 0xFF));
  commands[4] = ((char)((D >> 8) & 0xFF));
  commands[5] = ((char)(D & 0xFF));
  commands[6] = ((char)((P >> 24) & 0xFF));
  commands[7] = ((char)((P >> 16) & 0xFF));
  commands[8] = ((char)((P >> 8) & 0xFF));
  commands[9] = ((char)(P & 0xFF));
  commands[10] = ((char)((I >> 24) & 0xFF));
  commands[11] = ((char)((I >> 16) & 0xFF));
  commands[12] = ((char)((I >> 8) & 0xFF));
  commands[13] = ((char)(I & 0xFF));
  commands[14] = ((char)((QQPS >> 24) & 0xFF));
  commands[15] = ((char)((QQPS >> 16) & 0xFF));
  commands[16] = ((char)((QQPS >> 8) & 0xFF));
  commands[17] = ((char)(QQPS & 0xFF));
  commands[18] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 19));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 19);                 // Send data
#endif
}

// of motor 2
void RoboClaw::write_RoboClaw_PID_M2(char addr, int32_t D, int32_t P, int32_t I, int32_t QQPS)
{
  char checkSUM;
  checkSUM = (addr + 29 + ((char)((D >> 24) & 0xFF)) + ((char)((D >> 16) & 0xFF)) + ((char)((D >> 8) & 0xFF)) +
              ((char)(D & 0xFF)) + ((char)((P >> 24) & 0xFF)) + ((char)((P >> 16) & 0xFF)) + ((char)((P >> 8) & 0xFF)) +
              ((char)(P & 0xFF)) + ((char)((I >> 24) & 0xFF)) + ((char)((I >> 16) & 0xFF)) + ((char)((I >> 8) & 0xFF)) +
              ((char)(I & 0xFF)) + ((char)((QQPS >> 24) & 0xFF)) + ((char)((QQPS >> 16) & 0xFF)) +
              ((char)((QQPS >> 8) & 0xFF)) + ((char)(QQPS & 0xFF))) &
             0x7F;

  unsigned char commands[19];
  commands[0] = addr;
  commands[1] = 29;
  commands[2] = ((char)((D >> 24) & 0xFF));
  commands[3] = ((char)((D >> 16) & 0xFF));
  commands[4] = ((char)((D >> 8) & 0xFF));
  commands[5] = ((char)(D & 0xFF));
  commands[6] = ((char)((P >> 24) & 0xFF));
  commands[7] = ((char)((P >> 16) & 0xFF));
  commands[8] = ((char)((P >> 8) & 0xFF));
  commands[9] = ((char)(P & 0xFF));
  commands[10] = ((char)((I >> 24) & 0xFF));
  commands[11] = ((char)((I >> 16) & 0xFF));
  commands[12] = ((char)((I >> 8) & 0xFF));
  commands[13] = ((char)(I & 0xFF));
  commands[14] = ((char)((QQPS >> 24) & 0xFF));
  commands[15] = ((char)((QQPS >> 16) & 0xFF));
  commands[16] = ((char)((QQPS >> 8) & 0xFF));
  commands[17] = ((char)(QQPS & 0xFF));
  commands[18] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 19));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 19);                 // Send data
#endif
}

// Used to change the speed value of motor 1
void RoboClaw::write_RoboClaw_speed_M1(char addr, int32_t speed)
{
  char checkSUM;
  checkSUM = (addr + 35 + ((char)((speed >> 24) & 0xFF)) + ((char)((speed >> 16) & 0xFF)) +
              ((char)((speed >> 8) & 0xFF)) + ((char)(speed & 0xFF))) &
             0x7F;

  unsigned char commands[7];
  commands[0] = addr;
  commands[1] = 35;
  commands[2] = ((char)((speed >> 24) & 0xFF));
  commands[3] = ((char)((speed >> 16) & 0xFF));
  commands[4] = ((char)((speed >> 8) & 0xFF));
  commands[5] = ((char)(speed & 0xFF));
  commands[6] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 7));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 7);                  // Send data
#endif
}

// Used to change the speed value of motor 2
void RoboClaw::write_RoboClaw_speed_M2(char addr, int32_t speed)
{
  char checkSUM;
  checkSUM = (addr + 36 + ((char)((speed >> 24) & 0xFF)) + ((char)((speed >> 16) & 0xFF)) +
              ((char)((speed >> 8) & 0xFF)) + ((char)(speed & 0xFF))) &
             0x7F;

  unsigned char commands[7];
  commands[0] = addr;
  commands[1] = 36;
  commands[2] = ((char)((speed >> 24) & 0xFF));
  commands[3] = ((char)((speed >> 16) & 0xFF));
  commands[4] = ((char)((speed >> 8) & 0xFF));
  commands[5] = ((char)(speed & 0xFF));
  commands[6] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 7));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 7);                  // Send data
#endif
}

// Used to change the speed value of motors 1 and 2
void RoboClaw::write_RoboClaw_speed_M1M2(char addr, int32_t speedM1, int32_t speedM2)
{
  char checkSUM;

  checkSUM = (addr + 37 + ((char)((speedM1 >> 24) & 0xFF)) + ((char)((speedM1 >> 16) & 0xFF)) +
              ((char)((speedM1 >> 8) & 0xFF)) + ((char)(speedM1 & 0xFF)) + ((char)((speedM2 >> 24) & 0xFF)) +
              ((char)((speedM2 >> 16) & 0xFF)) + ((char)((speedM2 >> 8) & 0xFF)) + ((char)(speedM2 & 0xFF))) &
             0x7F;

  unsigned char commands[11];
  commands[0] = addr;
  commands[1] = 37;
  commands[2] = ((char)((speedM1 >> 24) & 0xFF));
  commands[3] = ((char)((speedM1 >> 16) & 0xFF));
  commands[4] = ((char)((speedM1 >> 8) & 0xFF));
  commands[5] = ((char)(speedM1 & 0xFF));
  commands[6] = ((char)((speedM2 >> 24) & 0xFF));
  commands[7] = ((char)((speedM2 >> 16) & 0xFF));
  commands[8] = ((char)((speedM2 >> 8) & 0xFF));
  commands[9] = ((char)(speedM2 & 0xFF));
  commands[10] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 11));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 11);                 // Send data
#endif
}

// Used to change the speed value of motor 1 and 2 during a specific distance
void RoboClaw::write_RoboClaw_speed_dist_M1M2(char addr, int32_t speedM1, int32_t distanceM1, int32_t speedM2,
                                              int32_t distanceM2)
{
  char checkSUM;
  checkSUM =
      (addr + 43 + ((char)((speedM1 >> 24) & 0xFF)) + ((char)((speedM1 >> 16) & 0xFF)) +
       ((char)((speedM1 >> 8) & 0xFF)) + ((char)(speedM1 & 0xFF)) + ((char)((speedM2 >> 24) & 0xFF)) +
       ((char)((speedM2 >> 16) & 0xFF)) + ((char)((speedM2 >> 8) & 0xFF)) + ((char)(speedM2 & 0xFF)) +
       ((char)((distanceM1 >> 24) & 0xFF)) + ((char)((distanceM1 >> 16) & 0xFF)) + ((char)((distanceM1 >> 8) & 0xFF)) +
       ((char)(distanceM1 & 0xFF)) + ((char)((distanceM2 >> 24) & 0xFF)) + ((char)((distanceM2 >> 16) & 0xFF)) +
       ((char)((distanceM2 >> 8) & 0xFF)) + ((char)(distanceM2 & 0xFF)) + 1) &
      0x7F;

  unsigned char commands[20];
  commands[0] = addr;
  commands[1] = 43;
  commands[2] = ((char)((speedM1 >> 24) & 0xFF));
  commands[3] = ((char)((speedM1 >> 16) & 0xFF));
  commands[4] = ((char)((speedM1 >> 8) & 0xFF));
  commands[5] = ((char)(speedM1 & 0xFF));
  commands[6] = ((char)((distanceM1 >> 24) & 0xFF));
  commands[7] = ((char)((distanceM1 >> 16) & 0xFF));
  commands[8] = ((char)((distanceM1 >> 8) & 0xFF));
  commands[9] = ((char)(distanceM1 & 0xFF));
  commands[10] = ((char)((speedM2 >> 24) & 0xFF));
  commands[11] = ((char)((speedM2 >> 16) & 0xFF));
  commands[12] = ((char)((speedM2 >> 8) & 0xFF));
  commands[13] = ((char)(speedM2 & 0xFF));
  commands[14] = ((char)((distanceM2 >> 24) & 0xFF));
  commands[15] = ((char)((distanceM2 >> 16) & 0xFF));
  commands[16] = ((char)((distanceM2 >> 8) & 0xFF));
  commands[17] = ((char)(distanceM2 & 0xFF));
  commands[18] = 1;
  commands[19] = checkSUM;

#ifdef USE_BOOST
  serial_->write_some(boost::asio::buffer(commands, 20));
// boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
  write(fd, commands, 20);                 // Send data
#endif
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RoboClaw");

  RoboClaw node;

  ros::Rate loop_rate(150);

  while (ros::ok())
  {
    // node.compute();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
