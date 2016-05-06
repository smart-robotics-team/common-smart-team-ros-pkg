// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <battery_checker/battery_checkerConfig.h>

// ROS message includes
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

// other includes
#include <battery_checker_common.cpp>


class battery_checker_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<battery_checker::battery_checkerConfig> server;
    dynamic_reconfigure::Server<battery_checker::battery_checkerConfig>::CallbackType f;

    ros::Publisher info_string_;
    ros::Publisher need_charge_;
    ros::Subscriber battery_voltage_;

    battery_checker_data component_data_;
    battery_checker_config component_config_;
    battery_checker_impl component_implementation_;

    battery_checker_ros() : np_("~")
    {
        f = boost::bind(&battery_checker_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        info_string_ = n_.advertise<std_msgs::String>("info_string", 1);
        need_charge_ = n_.advertise<std_msgs::Empty>("need_charge", 1);
        battery_voltage_ = n_.subscribe("battery_voltage", 1, &battery_checker_impl::topicCallback_battery_voltage, &component_implementation_);

        np_.param("voltage_limit", component_config_.voltage_limit, (double)4.0);
        np_.param("time_voltage_info", component_config_.time_voltage_info, (double)600.0);
        np_.param("time_need_charge", component_config_.time_need_charge, (double)300);
    }
    void topicCallback_battery_voltage(const std_msgs::Float32::ConstPtr& msg)
    {
        component_data_.in_battery_voltage = *msg;
    }

    void configure_callback(battery_checker::battery_checkerConfig &config, uint32_t level)
    {
        component_config_.voltage_limit = config.voltage_limit;
        component_config_.time_voltage_info = config.time_voltage_info;
        component_config_.time_need_charge = config.time_need_charge;
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_info_string_active = true;
        component_data_.out_need_charge_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_info_string_active)
            info_string_.publish(component_data_.out_info_string);
        if (component_data_.out_need_charge_active)
            need_charge_.publish(component_data_.out_need_charge);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "battery_checker");

    battery_checker_ros node;
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
