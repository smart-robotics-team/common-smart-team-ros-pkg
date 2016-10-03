// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <common_float_multiplexer/common_float_multiplexerConfig.h>

// ROS message includes
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32.h>

// other includes
#include <common_float_multiplexer_common.cpp>


class common_float_multiplexer_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<common_float_multiplexer::common_float_multiplexerConfig> server;
    dynamic_reconfigure::Server<common_float_multiplexer::common_float_multiplexerConfig>::CallbackType f;

    ros::Publisher output_;
    ros::Subscriber index_;
    ros::Subscriber input0_;
    ros::Subscriber input1_;
    ros::Subscriber input2_;
    ros::Subscriber input3_;

    common_float_multiplexer_data component_data_;
    common_float_multiplexer_config component_config_;
    common_float_multiplexer_impl component_implementation_;
    tf::TransformListener& tf_;

    common_float_multiplexer_ros(tf::TransformListener& tf) : np_("~"), tf_(tf)
    {
        f = boost::bind(&common_float_multiplexer_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        output_ = n_.advertise<std_msgs::Float32>("output", 1);
        index_ = n_.subscribe("index", 1, &common_float_multiplexer_impl::topicCallback_index, &component_implementation_);
        input0_ = n_.subscribe("input0", 1, &common_float_multiplexer_impl::topicCallback_input0, &component_implementation_);
        input1_ = n_.subscribe("input1", 1, &common_float_multiplexer_impl::topicCallback_input1, &component_implementation_);
        input2_ = n_.subscribe("input2", 1, &common_float_multiplexer_impl::topicCallback_input2, &component_implementation_);
        input3_ = n_.subscribe("input3", 1, &common_float_multiplexer_impl::topicCallback_input3, &component_implementation_);

        np_.param("starting_index", component_config_.starting_index, (int)0);
    }
    void topicCallback_index(const std_msgs::Int8::ConstPtr& msg)
    {
        component_data_.in_index = *msg;
    }
    void topicCallback_input0(const std_msgs::Float32::ConstPtr& msg)
    {
        component_data_.in_input0 = *msg;
    }
    void topicCallback_input1(const std_msgs::Float32::ConstPtr& msg)
    {
        component_data_.in_input1 = *msg;
    }
    void topicCallback_input2(const std_msgs::Float32::ConstPtr& msg)
    {
        component_data_.in_input2 = *msg;
    }
    void topicCallback_input3(const std_msgs::Float32::ConstPtr& msg)
    {
        component_data_.in_input3 = *msg;
    }

    void configure_callback(common_float_multiplexer::common_float_multiplexerConfig &config, uint32_t level)
    {
        component_config_.starting_index = config.starting_index;
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

    ros::init(argc, argv, "common_float_multiplexer");

    tf::TransformListener listener(ros::Duration(10));
    common_float_multiplexer_ros node(listener);
    node.configure();

    ros::Rate loop_rate(50.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
