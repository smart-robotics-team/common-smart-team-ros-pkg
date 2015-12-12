// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <remove_self_pointcloud/remove_self_pointcloudConfig.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

// other includes
#include <remove_self_pointcloud_common.cpp>


class remove_self_pointcloud_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<remove_self_pointcloud::remove_self_pointcloudConfig> server;
    dynamic_reconfigure::Server<remove_self_pointcloud::remove_self_pointcloudConfig>::CallbackType f;

    ros::Publisher output_;
    ros::Subscriber input_;
    ros::Subscriber robot_position_;

    remove_self_pointcloud_data component_data_;
    remove_self_pointcloud_config component_config_;
    remove_self_pointcloud_impl component_implementation_;

    remove_self_pointcloud_ros() : np_("~")
    {
        f = boost::bind(&remove_self_pointcloud_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        output_ = n_.advertise<sensor_msgs::PointCloud2>("output", 1);
        input_ = n_.subscribe("input", 1, &remove_self_pointcloud_impl::topicCallback_input, &component_implementation_);
        robot_position_ = n_.subscribe("robot_position", 1, &remove_self_pointcloud_impl::topicCallback_robot_position, &component_implementation_);

        np_.param("inhib_size", component_config_.inhib_size, (double)0.05);
    }
    void topicCallback_input(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        component_data_.in_input = *msg;
    }
    void topicCallback_robot_position(const nav_msgs::Odometry::ConstPtr& msg)
    {
        component_data_.in_robot_position = *msg;
    }

    void configure_callback(remove_self_pointcloud::remove_self_pointcloudConfig &config, uint32_t level)
    {
        component_config_.inhib_size = config.inhib_size;
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

    ros::init(argc, argv, "remove_self_pointcloud");

    remove_self_pointcloud_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
