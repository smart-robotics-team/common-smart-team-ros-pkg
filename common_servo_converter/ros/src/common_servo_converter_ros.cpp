// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <common_servo_converter/common_servo_converterConfig.h>

// ROS message includes
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16.h>

// other includes
#include <common_servo_converter_common.cpp>


class common_servo_converter_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<common_servo_converter::common_servo_converterConfig> server;
    dynamic_reconfigure::Server<common_servo_converter::common_servo_converterConfig>::CallbackType f;

    ros::Publisher output_;
    ros::Subscriber input_;

    common_servo_converter_data component_data_;
    common_servo_converter_config component_config_;
    common_servo_converter_impl component_implementation_;

    common_servo_converter_ros() : np_("~")
    {
        f = boost::bind(&common_servo_converter_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        output_ = n_.advertise<std_msgs::UInt16>("output", 1);
        input_ = n_.subscribe("input", 1, &common_servo_converter_impl::topicCallback_input, &component_implementation_);

        np_.param("inverted", component_config_.inverted, (int)0);
        np_.param("offset", component_config_.offset, (int)0);
    }
    void topicCallback_input(const std_msgs::UInt16::ConstPtr& msg)
    {
        component_data_.in_input = *msg;
    }

    void configure_callback(common_servo_converter::common_servo_converterConfig &config, uint32_t level)
    {
        component_config_.inverted = config.inverted;
        component_config_.offset = config.offset;
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_output_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_output_active)
            output_.publish(component_data_.out_output);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "common_servo_converter");

    common_servo_converter_ros node;
    node.configure();

    ros::Rate loop_rate(20.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
