// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <common_odom_estimation/common_odom_estimationConfig.h>

// ROS message includes
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Empty.h>

// other includes
#include <common_odom_estimation_common.cpp>


class common_odom_estimation_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<common_odom_estimation::common_odom_estimationConfig> server;
    dynamic_reconfigure::Server<common_odom_estimation::common_odom_estimationConfig>::CallbackType f;

    ros::Publisher odom_est_;
    ros::Subscriber imu_;
    ros::Subscriber cmd_vel_;
    ros::Subscriber beacon_;
    ros::Subscriber init_;

    common_odom_estimation_data component_data_;
    common_odom_estimation_config component_config_;
    common_odom_estimation_impl component_implementation_;

    common_odom_estimation_ros() : np_("~")
    {
        f = boost::bind(&common_odom_estimation_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        odom_est_ = n_.advertise<nav_msgs::Odometry>("odom_est", 1);
        imu_ = n_.subscribe("imu", 1, &common_odom_estimation_impl::topicCallback_imu, &component_implementation_);
        cmd_vel_ = n_.subscribe("cmd_vel", 1, &common_odom_estimation_impl::topicCallback_cmd_vel, &component_implementation_);
        beacon_ = n_.subscribe("beacon", 1, &common_odom_estimation_impl::topicCallback_beacon, &component_implementation_);
        init_ = n_.subscribe("init", 1, &common_odom_estimation_impl::topicCallback_init, &component_implementation_);

        np_.param("enable_imu", component_config_.enable_imu, (bool)false);
        np_.param("enable_fake", component_config_.enable_fake, (bool)true);
        np_.param("enable_beacon", component_config_.enable_beacon, (bool)false);
        np_.param("parent_link", component_config_.parent_link, (std::string)"map");
        np_.param("child_link", component_config_.child_link, (std::string)"base_link");
        np_.param("poseX", component_config_.poseX, (double)0.0);
        np_.param("poseY", component_config_.poseY, (double)0.0);
        np_.param("theta", component_config_.theta, (double)0.0);
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
    void topicCallback_init(const std_msgs::Empty::ConstPtr& msg)
    {
        component_data_.in_init = *msg;
    }

    void configure_callback(common_odom_estimation::common_odom_estimationConfig &config, uint32_t level)
    {
        component_config_.enable_imu = config.enable_imu;
        component_config_.enable_fake = config.enable_fake;
        component_config_.enable_beacon = config.enable_beacon;
        component_config_.parent_link = config.parent_link;
        component_config_.child_link = config.child_link;
        component_config_.poseX = config.poseX;
        component_config_.poseY = config.poseY;
        component_config_.theta = config.theta;
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_odom_est_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_odom_est_active)
            odom_est_.publish(component_data_.out_odom_est);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "common_odom_estimation");

    common_odom_estimation_ros node;
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
