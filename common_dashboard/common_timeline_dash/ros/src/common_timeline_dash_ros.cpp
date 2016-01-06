// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <common_timeline_dash/common_timeline_dashConfig.h>

// ROS message includes
#include <std_msgs/String.h>

// other includes
#include <common_timeline_dash_common.cpp>


class common_timeline_dash_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<common_timeline_dash::common_timeline_dashConfig> server;
    dynamic_reconfigure::Server<common_timeline_dash::common_timeline_dashConfig>::CallbackType f;

    ros::Subscriber input_;

    common_timeline_dash_data component_data_;
    common_timeline_dash_config component_config_;
    common_timeline_dash_impl component_implementation_;

    common_timeline_dash_ros() : np_("~")
    {
        f = boost::bind(&common_timeline_dash_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        input_ = n_.subscribe("input", 1, &common_timeline_dash_impl::topicCallback_input, &component_implementation_);

        np_.param("file_path", component_config_.file_path, (std::string)"/home/jbot/home_dashboard_project/timeline_data.yml");
    }
    void topicCallback_input(const std_msgs::String::ConstPtr& msg)
    {
        component_data_.in_input = *msg;
    }

    void configure_callback(common_timeline_dash::common_timeline_dashConfig &config, uint32_t level)
    {
        component_config_.file_path = config.file_path;
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

    ros::init(argc, argv, "common_timeline_dash");

    common_timeline_dash_ros node;
    node.configure();

    ros::Rate loop_rate(1.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
