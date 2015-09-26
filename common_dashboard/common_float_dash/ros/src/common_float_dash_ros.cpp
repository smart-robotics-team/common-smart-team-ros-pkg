// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <common_float_dash/common_float_dashConfig.h>

// ROS message includes
#include <std_msgs/Float32.h>

// other includes
#include <common_float_dash_common.cpp>


class common_float_dash_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<common_float_dash::common_float_dashConfig> server;
    dynamic_reconfigure::Server<common_float_dash::common_float_dashConfig>::CallbackType f;

    ros::Subscriber input_;

    common_float_dash_data component_data_;
    common_float_dash_config component_config_;
    common_float_dash_impl component_implementation_;

    common_float_dash_ros() : np_("~")
    {
        f = boost::bind(&common_float_dash_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        input_ = n_.subscribe("input", 1, &common_float_dash_impl::topicCallback_input, &component_implementation_);

        np_.param("address", component_config_.address, (std::string)"192.168.0.1");
        np_.param("port", component_config_.port, (std::string)"3030");
        np_.param("type", component_config_.type, (std::string)"value");
        np_.param("auth", component_config_.auth, (std::string)"YOUR_AUTH_TOKEN");
        np_.param("dash_topic", component_config_.dash_topic, (std::string)"synergie");
    }
    void topicCallback_input(const std_msgs::Float32::ConstPtr& msg)
    {
        component_data_.in_input = *msg;
    }

    void configure_callback(common_float_dash::common_float_dashConfig &config, uint32_t level)
    {
        component_config_.address = config.address;
        component_config_.port = config.port;
        component_config_.type = config.type;
        component_config_.auth = config.auth;
        component_config_.dash_topic = config.dash_topic;
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "common_float_dash");

    common_float_dash_ros node;
    node.configure();

    ros::Rate loop_rate(0.5);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
