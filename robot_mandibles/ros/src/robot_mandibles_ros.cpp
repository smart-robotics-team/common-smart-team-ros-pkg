// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <robot_mandibles/robot_mandiblesConfig.h>

// ROS message includes
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16.h>

// other includes
#include <robot_mandibles_common.cpp>


class robot_mandibles_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<robot_mandibles::robot_mandiblesConfig> server;
    dynamic_reconfigure::Server<robot_mandibles::robot_mandiblesConfig>::CallbackType f;

    ros::Publisher servo_speed_;
    ros::Publisher servo1_;
    ros::Publisher servo2_;
    ros::Subscriber position_;

    robot_mandibles_data component_data_;
    robot_mandibles_config component_config_;
    robot_mandibles_impl component_implementation_;

    robot_mandibles_ros() : np_("~")
    {
        f = boost::bind(&robot_mandibles_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        servo_speed_ = n_.advertise<std_msgs::UInt16>("servo_speed", 1);
        servo1_ = n_.advertise<std_msgs::UInt16>("servo1", 1);
        servo2_ = n_.advertise<std_msgs::UInt16>("servo2", 1);
        position_ = n_.subscribe("position", 1, &robot_mandibles_impl::topicCallback_position, &component_implementation_);

        np_.param("servo1_position0", component_config_.servo1_position0, (int)90);
        np_.param("servo2_position0", component_config_.servo2_position0, (int)90);
        np_.param("servo1_position1", component_config_.servo1_position1, (int)100);
        np_.param("servo2_position1", component_config_.servo2_position1, (int)100);
        np_.param("servo1_position2", component_config_.servo1_position2, (int)80);
        np_.param("servo2_position2", component_config_.servo2_position2, (int)80);
        np_.param("move_speed", component_config_.move_speed, (int)3);
        np_.param("wait_step_1_to_2", component_config_.wait_step_1_to_2, (double)1.0);
    }
    void topicCallback_position(const std_msgs::UInt16::ConstPtr& msg)
    {
        component_data_.in_position = *msg;
    }

    void configure_callback(robot_mandibles::robot_mandiblesConfig &config, uint32_t level)
    {
        component_config_.servo1_position0 = config.servo1_position0;
        component_config_.servo2_position0 = config.servo2_position0;
        component_config_.servo1_position1 = config.servo1_position1;
        component_config_.servo2_position1 = config.servo2_position1;
        component_config_.servo1_position2 = config.servo1_position2;
        component_config_.servo2_position2 = config.servo2_position2;
        component_config_.move_speed = config.move_speed;
        component_config_.wait_step_1_to_2 = config.wait_step_1_to_2;
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_servo_speed_active = true;
        component_data_.out_servo1_active = true;
        component_data_.out_servo2_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_servo_speed_active)
            servo_speed_.publish(component_data_.out_servo_speed);
        if (component_data_.out_servo1_active)
            servo1_.publish(component_data_.out_servo1);
        if (component_data_.out_servo2_active)
            servo2_.publish(component_data_.out_servo2);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "robot_mandibles");

    robot_mandibles_ros node;
    node.configure();

    ros::Rate loop_rate(10.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
