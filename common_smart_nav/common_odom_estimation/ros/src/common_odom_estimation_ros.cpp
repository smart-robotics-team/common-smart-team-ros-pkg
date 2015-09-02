// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <common_odom_estimation/common_odom_estimationConfig.h>

// ROS message includes
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

// other includes
#include <common_odom_estimation_common.cpp>


class common_odom_estimation_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<common_odom_estimation::common_odom_estimationConfig> server;
    dynamic_reconfigure::Server<common_odom_estimation::common_odom_estimationConfig>::CallbackType f;

    ros::Subscriber imu_;
    ros::Subscriber cmd_vel_;
    ros::Subscriber beacon_;

    common_odom_estimation_data component_data_;
    common_odom_estimation_config component_config_;
    common_odom_estimation_impl component_implementation_;

    common_odom_estimation_ros() : np_("~")
    {
        f = boost::bind(&common_odom_estimation_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        imu_ = n_.subscribe("imu", 1, &common_odom_estimation_impl::topicCallback_imu, &component_implementation_);
        cmd_vel_ = n_.subscribe("cmd_vel", 1, &common_odom_estimation_impl::topicCallback_cmd_vel, &component_implementation_);
        beacon_ = n_.subscribe("beacon", 1, &common_odom_estimation_impl::topicCallback_beacon, &component_implementation_);

        np_.param("enable_imu", component_config_.enable_imu, (int)0);
        np_.param("enable_fake", component_config_.enable_fake, (int)1);
        np_.param("enable_beacon", component_config_.enable_beacon, (int)0);
    }
    void topicCallback_imu(const sensor_msgs::Imu::ConstPtr& msg)
    {
        component_data_.in_imu = *msg;
    }
    void topicCallback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
    {
        component_data_.in_cmd_vel = *msg;
    }
    void topicCallback_beacon(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        component_data_.in_beacon = *msg;
    }

    void configure_callback(common_odom_estimation::common_odom_estimationConfig &config, uint32_t level)
    {
        component_config_.enable_imu = config.enable_imu;
        component_config_.enable_fake = config.enable_fake;
        component_config_.enable_beacon = config.enable_beacon;
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

    ros::init(argc, argv, "common_odom_estimation");

    common_odom_estimation_ros node();
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
