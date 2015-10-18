// ROS message includes
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Empty.h>

/* protected region user include files on begin */
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
/* protected region user include files end */

class common_odom_estimation_config
{
public:
    bool enable_imu;
    bool enable_fake;
    bool enable_beacon;
    std::string parent_link;
    std::string child_link;
    double poseX;
    double poseY;
    double theta;
};

class common_odom_estimation_data
{
// autogenerated: don't touch this class
public:
    //input data
    sensor_msgs::Imu in_imu;
    geometry_msgs::Twist in_cmd_vel;
    geometry_msgs::Pose2D in_beacon;
    std_msgs::Empty in_init;
    //output data
    nav_msgs::Odometry out_odom_est;
    bool out_odom_est_active;
};

class common_odom_estimation_impl
{
    /* protected region user member variables on begin */
	common_odom_estimation_config localconfig;

	tf::StampedTransform t;
	tf::TransformBroadcaster broadcaster;

	geometry_msgs::Pose2D estimated_pose;
	geometry_msgs::Twist last_cmd_vel;
	sensor_msgs::Imu last_imu;

	ros::Time last_beacon_time;
	ros::Time last_imu_time;

	double integral_imu_x;
	double integral_imu_y;

	double offset_theta;
    /* protected region user member variables end */

public:
    common_odom_estimation_impl() 
    {
        /* protected region user constructor on begin */
    	estimated_pose.x = 0.0;
    	estimated_pose.y = 0.0;
    	estimated_pose.theta = 0.0;

    	last_beacon_time = ros::Time::now();
    	last_imu_time = ros::Time::now();

    	integral_imu_x = 0.0;
    	integral_imu_y = 0.0;

	offset_theta = 0.0;
    	/* protected region user constructor end */
    }

    void configure(common_odom_estimation_config config) 
    {
        /* protected region user configure on begin */
    	localconfig = config;
        /* protected region user configure end */
    }

    void update(common_odom_estimation_data &data, common_odom_estimation_config config)
    {
        /* protected region user update on begin */

        if(localconfig.enable_fake && localconfig.enable_beacon && localconfig.enable_imu)
        {
		//if(last_beacon_time.toSec() > 0.1)
		//{
			if(fabs(last_cmd_vel.linear.x) > 0.01 || fabs(last_cmd_vel.linear.y) > 0.01)
			{
				double dt = ros::Time::now().toSec() - data.out_odom_est.header.stamp.toSec();
				//estimated_pose.x += ( (integral_imu_x * cos(estimated_pose.theta) + integral_imu_y * sin(estimated_pose.theta)) * dt);
				//estimated_pose.y += ( (integral_imu_y * cos(estimated_pose.theta) + integral_imu_x * sin(estimated_pose.theta)) * dt);
                                estimated_pose.x += ( (last_cmd_vel.linear.x * cos(estimated_pose.theta) - last_cmd_vel.linear.y * sin(estimated_pose.theta)) * dt);
                                estimated_pose.y += ( -(last_cmd_vel.linear.y * cos(estimated_pose.theta) + last_cmd_vel.linear.x * sin(estimated_pose.theta)) * dt);
			}
		//}
	}

	if(localconfig.enable_fake && localconfig.enable_beacon && !localconfig.enable_imu)
        {
        	double dt = ros::Time::now().toSec() - data.out_odom_est.header.stamp.toSec();
	        if(last_beacon_time.toSec() > 0.1) 
                {       
                        if(fabs(last_cmd_vel.linear.x) > 0.01 || fabs(last_cmd_vel.linear.y) > 0.01 )
                        {       
                                estimated_pose.x += ( (last_cmd_vel.linear.x * cos(estimated_pose.theta) + last_cmd_vel.linear.y * sin(estimated_pose.theta)) * dt);
                                estimated_pose.y += ( (last_cmd_vel.linear.y * cos(estimated_pose.theta) + last_cmd_vel.linear.x * sin(estimated_pose.theta)) * dt);
                        }       
                }
		if(fabs(last_cmd_vel.angular.z) > 0.001 )
                {
			estimated_pose.theta += ( last_cmd_vel.angular.z * dt );
		}

        }

	t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(estimated_pose.theta), tf::Vector3(estimated_pose.x, estimated_pose.y, 0.0)),
		                                ros::Time::now(), localconfig.parent_link, localconfig.child_link);

	t.stamp_ = ros::Time::now();
	broadcaster.sendTransform(t);

	data.out_odom_est.child_frame_id = localconfig.child_link;
	data.out_odom_est.header.frame_id = localconfig.parent_link;
	data.out_odom_est.header.stamp = ros::Time::now();

	data.out_odom_est.pose.pose.position.x = estimated_pose.x;
	data.out_odom_est.pose.pose.position.y = estimated_pose.y;
	data.out_odom_est.pose.pose.position.z = 0.0;

	data.out_odom_est.pose.pose.orientation = tf::createQuaternionMsgFromYaw(estimated_pose.theta);
        /* protected region user update end */
    }

    void topicCallback_imu(const sensor_msgs::Imu::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for imu on begin */
    	if(localconfig.enable_fake)
    	{
		if(fabs(last_cmd_vel.linear.x) > 0.01 || fabs(last_cmd_vel.linear.y) > 0.01)
		{
			integral_imu_x = integral_imu_x + msg->linear_acceleration.x * (ros::Time::now().toSec() - last_imu_time.toSec());
			integral_imu_y = integral_imu_y + msg->linear_acceleration.y * (ros::Time::now().toSec() - last_imu_time.toSec());
		}
		else
		{
			// Don't compute
			integral_imu_x = 0.0;
			integral_imu_y = 0.0;
		}
	}
	else
	{
		// Don't compute
	}

	//std::cout << "IMU x speed : " << integral_imu_x << "   " << integral_imu_y << std::endl;

    	estimated_pose.theta = tf::getYaw(msg->orientation) - offset_theta;
    	last_imu_time = ros::Time::now();
	last_imu = *msg;
        /* protected region user implementation of subscribe callback for imu end */
    }
    void topicCallback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for cmd_vel on begin */
    	last_cmd_vel = *msg;
        /* protected region user implementation of subscribe callback for cmd_vel end */
    }
    void topicCallback_beacon(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for beacon on begin */
    	last_beacon_time = ros::Time::now();
    	estimated_pose.x = msg->x;
    	estimated_pose.y = msg->y;
    	/* protected region user implementation of subscribe callback for beacon end */
    }
    void topicCallback_init(const std_msgs::Empty::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for init on begin */
    	
	offset_theta = tf::getYaw(last_imu.orientation);

	std::cout << "offset : " << offset_theta << "   " << tf::getYaw(last_imu.orientation) << std::endl;

	estimated_pose.x = localconfig.poseX;
	estimated_pose.y = localconfig.poseY;
	estimated_pose.theta = localconfig.theta;

        /* protected region user implementation of subscribe callback for init end */
    }



    /* protected region user additional functions on begin */
    /* protected region user additional functions end */
};
