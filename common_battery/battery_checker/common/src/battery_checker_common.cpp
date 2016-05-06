// ROS message includes
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

/* protected region user include files on begin */
/* protected region user include files end */

class battery_checker_config
{
public:
    double voltage_limit;
    double time_voltage_info;
    double time_need_charge;
};

class battery_checker_data
{
// autogenerated: don't touch this class
public:
    //input data
    std_msgs::Float32 in_battery_voltage;
    //output data
    std_msgs::String out_info_string;
    bool out_info_string_active;
    std_msgs::Empty out_need_charge;
    bool out_need_charge_active;
};

class battery_checker_impl
{
    /* protected region user member variables on begin */
	battery_checker_config local_config;

	ros::Time last_voltage_info;
	ros::Time last_need_charge;

	bool charge_needed;
	bool output_voltage_info;
	std_msgs::Float32 current_voltage;
    /* protected region user member variables end */

public:
    battery_checker_impl() 
    {
        /* protected region user constructor on begin */
    	last_voltage_info = ros::Time::now();
    	last_need_charge = ros::Time::now();

    	charge_needed = false;
    	output_voltage_info = false;
    	current_voltage.data = 0.0;
        /* protected region user constructor end */
    }

    void configure(battery_checker_config config) 
    {
        /* protected region user configure on begin */
    	local_config = config;
        /* protected region user configure end */
    }

    void update(battery_checker_data &data, battery_checker_config config)
    {
        /* protected region user update on begin */
    	data.out_info_string_active = false;
    	data.out_need_charge_active = false;
    	if(output_voltage_info)
    	{
    		std::ostringstream ss;
    		ss << current_voltage.data;
    		std::string s(ss.str());
    		data.out_info_string.data = std::string("Current battery voltage : ") + s;
    		data.out_info_string_active = true;
                output_voltage_info = false;
    	}

    	if(charge_needed)
    	{
    		data.out_need_charge_active = true;
		
                std::ostringstream ss;
                ss << current_voltage.data;
                std::string s(ss.str());
                data.out_info_string.data = std::string("WARNING! Battery NEEDS charge : ") + s;
                data.out_info_string_active = true;
                charge_needed = false;
    	}
        /* protected region user update end */
    }

    void topicCallback_battery_voltage(const std_msgs::Float32::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for battery_voltage on begin */
        current_voltage.data = msg->data;

        output_voltage_info = false;
        if( (ros::Time::now().toSec() - last_voltage_info.toSec()) > local_config.time_voltage_info )
        {
		output_voltage_info = true;
        	last_voltage_info = ros::Time::now();
        }

        charge_needed = false;
        if(msg->data < local_config.voltage_limit)
        {
        	if( (ros::Time::now().toSec() - last_need_charge.toSec()) > local_config.time_need_charge )
        	{
        		charge_needed = true;
        		last_need_charge = ros::Time::now();
        	}
        }
    	/* protected region user implementation of subscribe callback for battery_voltage end */
    }



    /* protected region user additional functions on begin */
    /* protected region user additional functions end */
};
