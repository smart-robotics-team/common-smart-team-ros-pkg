#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//#include <move_base/move_base.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/GetPlan.h>
#include <navfn/navfn.h>
#include <navfn/navfn_ros.h>
#include <global_planner/planner_core.h>

#include <pluginlib/class_loader.h>

#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

//#include "common_smart_nav/GetPlan.h"
#include "common_smart_nav/GetRobotPose.h"
#include "common_smart_nav/GetPlan.h"
#include "common_smart_nav/GetDistance.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                                         // for in-/output
#include <string.h>                                        // strcat
#include <fcntl.h>                                         // for 'O_RDONLY' deklaration
#include <termios.h>                                       // for serial

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>

#include <vector>
#include <list>


#define STOP     0
#define PAUSE    1
#define RUN        2


class TrajectoryManager 
{
	public:
		TrajectoryManager(tf::TransformListener& tf, std::string name);
		~TrajectoryManager();
		void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);

		// Goal suscriber
		ros::Subscriber goal_sub_;
		ros::Subscriber pathdone_sub_;
		ros::Subscriber computepath_sub_;
		ros::Subscriber pause_sub_;
		ros::Subscriber resume_sub_;

		ros::Publisher path_pub;
		ros::Publisher pose_pub;
		ros::Publisher pathimpossible_pub;

		ros::ServiceServer pose_service;
		ros::ServiceServer distance_service;
		ros::ServiceServer path_service;

		int cpt_pathimp;

	private:
		void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
		void pathDoneCallback(const std_msgs::Empty::ConstPtr & pose);
		void computePathCallback(const std_msgs::Empty::ConstPtr & pose);
		void pauseCallback(const std_msgs::Empty::ConstPtr & pose);
		void resumeCallback(const std_msgs::Empty::ConstPtr & pose);
		void computePointHead(nav_msgs::Path& path);
		void computePointZdirection(nav_msgs::Path& path, double value);
		void computePath(void);
		void planThread(void);
		void publishPath(void);

		bool getRobotPose(common_smart_nav::GetRobotPose::Request  &req, common_smart_nav::GetRobotPose::Response &res );
		bool getDistance(common_smart_nav::GetDistance::Request  &req, common_smart_nav::GetDistance::Response &res );
		bool getPath(common_smart_nav::GetPlan::Request &req, common_smart_nav::GetPlan::Response &res);

		double compute_distance(nav_msgs::Path path_to_compute);
		double getHeadingFromQuat(geometry_msgs::Quaternion pose);

		void goalActionCallback(void);

		ros::NodeHandle nh;

		int status; // STOP PAUSE RUN
		int cpt;
		char sem;

		nav_msgs::Path my_path;
		geometry_msgs::PoseStamped final_pose;
		geometry_msgs::PoseStamped current_pose;

		boost::thread* planner_thread_;

		tf::TransformListener& tf_;
		costmap_2d::Costmap2DROS* planner_costmap_;
		//navfn::NavfnROS* planner_;
		boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
		pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;

		std::string map_name;

		actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;
		move_base_msgs::MoveBaseFeedback action_feedback_;
		move_base_msgs::MoveBaseResult action_result_;
		int action_goal;

};

TrajectoryManager::TrajectoryManager(tf::TransformListener& tf, std::string name):tf_(tf),bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"), as_(nh, "move_base_smart", false)
{


	std::string costmap_name;
	std::string planner_name;
	ros::NodeHandle nhp("~");

	//nh.param<std::string>("costmap_name", costmap_name, "ROBOT_costmap");
	//nh.param<std::string>("planner_name", planner_name, "ROBOT_planner");
	nhp.getParam("costmap_name", costmap_name);
	nhp.getParam("planner_name", planner_name);
	nhp.getParam("map_name", map_name);

	std::string global_planner;
	//nhp.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
	nhp.param("base_global_planner", global_planner, std::string("global_planner/GlobalPlanner"));

	status = 0;    
	cpt = 0;
	cpt_pathimp = 0;
	sem = 1;

	goal_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("/ROBOT/goal", 2, &TrajectoryManager::goalCallback, this);
	pose_pub = nh.advertise < geometry_msgs::PoseStamped > ("/ROBOT/poseStamped", 2);

	path_pub = nh.advertise < nav_msgs::Path > ("/ROBOT/plan", 5);

	pathdone_sub_ = nh.subscribe < std_msgs::Empty > ("/ROBOT/path_done", 1, &TrajectoryManager::pathDoneCallback, this);
	computepath_sub_ = nh.subscribe < std_msgs::Empty > ("/ROBOT/compute_path", 4, &TrajectoryManager::computePathCallback, this);
	pause_sub_ = nh.subscribe < std_msgs::Empty > ("/ROBOT/pause", 2, &TrajectoryManager::pauseCallback, this);
	resume_sub_ = nh.subscribe < std_msgs::Empty > ("/ROBOT/resume", 2, &TrajectoryManager::resumeCallback, this);

	pathimpossible_pub = nh.advertise < std_msgs::Empty > ("/goal_unreachable", 2);

	pose_service = nh.advertiseService("/ROBOT/get_robot_pose", &TrajectoryManager::getRobotPose, this);
	distance_service = nh.advertiseService("/ROBOT/get_distance", &TrajectoryManager::getDistance, this);
	path_service = nh.advertiseService("/ROBOT/get_path", &TrajectoryManager::getPath, this);

	//just an arbitrary point in space
	current_pose.header.frame_id = map_name;
	current_pose.header.stamp = ros::Time();
	current_pose.pose.position.x = 0.0;
	current_pose.pose.position.y = 0.0;
	current_pose.pose.position.z = 0.0;

	current_pose.pose.orientation.x = 0.0;
	current_pose.pose.orientation.y = 0.0;
	current_pose.pose.orientation.z = 0.0;
	current_pose.pose.orientation.w = 1.0;

	my_path.poses = std::vector < geometry_msgs::PoseStamped > ();

	if (my_path.poses.size() > (my_path.poses.max_size() - 2)) {
		my_path.poses.std::vector < geometry_msgs::PoseStamped >::pop_back();
	}

	my_path.header.frame_id = map_name;

	final_pose.pose.position.x = 0.0;
	final_pose.pose.position.y = 0.14;
	final_pose.pose.position.z = 0.0;
	//final_pose.theta = 0.0;

	//tf_ = tf;

	//create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
	planner_costmap_ = new costmap_2d::Costmap2DROS(costmap_name, tf_);
	planner_costmap_->pause();

	//initialize the global planner
	//bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner");
	//planner_ = new navfn::NavfnROS(planner_name, planner_costmap_);
	try {
		planner_ = bgp_loader_.createInstance(global_planner);
		planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_);
	} catch (const pluginlib::PluginlibException& ex) {
		ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
		exit(1);
	}
	//planner_ = new nav_core::BaseGlobalPlanner(planner_name, planner_costmap_);
	//planner->initialize("NESTOR_planner", planner_costmap);

	//register the goal and feeback callbacks
	as_.registerGoalCallback(boost::bind(&TrajectoryManager::goalActionCallback, this));
	action_goal = 0;

	planner_costmap_->start();    
	as_.start();

	status = PAUSE;


	planner_thread_ = new boost::thread(boost::bind(&TrajectoryManager::planThread, this));
}

TrajectoryManager::~TrajectoryManager()
{

	planner_thread_->interrupt();
	planner_thread_->join();

}


void TrajectoryManager::goalActionCallback(void)
{
	final_pose = as_.acceptNewGoal()->target_pose;
	//final_pose = goal->goal;
	action_goal = 1;
	ROS_INFO("NEW POSE (ACTION)");
	/*
	   planner_costmap_->stop();
	   planner_costmap_->resetLayers();
	   planner_costmap_->start();
	 */
	computePath();
	publishPath();

	status = RUN;
}


void TrajectoryManager::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose)
{
	// Assuming the angles are in radians.
	double c1 = cos(heading / 2);
	double s1 = sin(heading / 2);
	double c2 = cos(attitude / 2);

	double s2 = sin(attitude / 2);
	double c3 = cos(bank / 2);
	double s3 = sin(bank / 2);
	double c1c2 = c1 * c2;
	double s1s2 = s1 * s2;

	pose->pose.orientation.w = c1c2 * c3 - s1s2 * s3;
	pose->pose.orientation.x = c1c2 * s3 + s1s2 * c3;
	pose->pose.orientation.y = s1 * c2 * c3 + c1 * s2 * s3;
	pose->pose.orientation.z = c1 * s2 * c3 - s1 * c2 * s3;
}

double TrajectoryManager::compute_distance(nav_msgs::Path path_to_compute)
{
	double distance = 0.0;
	geometry_msgs::Pose2D current_pose;

	if ( !(path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::empty()) ) {
		current_pose.x = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;
		current_pose.y = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;
		path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::erase (path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::begin());

		//ROS_ERROR("current X : %f, X : %f / current Y : %f, Y : %f", current_pose.x, path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, current_pose.y, path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y);

		while ( !(path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::empty()) ) {
			distance += sqrt( pow(current_pose.x - path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, 2) + pow(current_pose.y - path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y, 2) );

			current_pose.x = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;
			current_pose.y = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;
			path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::erase (path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::begin());

			//ROS_ERROR("current X : %f, X : %f / current Y : %f, Y : %f", current_pose.x, path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, current_pose.y, path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y);
		}

	}
	//ROS_ERROR("Distance : %f", distance);

	return distance;

}

bool TrajectoryManager::getRobotPose(common_smart_nav::GetRobotPose::Request  &req,
		common_smart_nav::GetRobotPose::Response &res )
{
	res.pose = current_pose;
	//res.sum = req.a + req.b;
	//ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	//ROS_INFO("sending back response: [%ld]", (long int)res.sum);
	//action_feedback_.base_position = current_pose;
	//as_.publishFeedback(action_feedback_);
	return true;
}

bool TrajectoryManager::getDistance(common_smart_nav::GetDistance::Request  &req,
		common_smart_nav::GetDistance::Response &res )
{

	std::vector<geometry_msgs::PoseStamped> global_plan;
	nav_msgs::Path tmp_path;

        while (sem < 1) {
                usleep(10000);
        }
        sem = 0;

	//make sure we have a costmap for our planner
	if(planner_costmap_ == NULL){
		ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
		//return false;
	}

	tf::Stamped<tf::Pose> global_pose;
	if(!planner_costmap_->getRobotPose(global_pose)){
		ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
		//return false;
	}

	geometry_msgs::PoseStamped start;
	//if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
	//if(req.start.header.frame_id == "")
	tf::poseStampedTFToMsg(global_pose, start);

	current_pose = start;

	//costmap_2d::Costmap2D* pCostmap = planner_costmap_->getCostmap();
	//boost::unique_lock< boost::shared_mutex > lock(*(pCostmap->getLock()));
	//boost::unique_lock< boost::shared_mutex > lock(*(planner_costmap_->getCostmap()->getLock()));

	if(planner_->makePlan(start, req.goal, global_plan)){
		//ROS_ERROR("planner makes plan");
		if(!global_plan.empty()){
			global_plan.push_back(req.goal);
			//ROS_ERROR("globalplan filled");
		}
	}

	tmp_path.header.frame_id = map_name;
	tmp_path.poses = global_plan;

	res.distance.data = compute_distance(tmp_path);

	//action_feedback_.base_position = current_pose;
	//as_.publishFeedback(action_feedback_);

	sem = 1;

	return true;
}


bool TrajectoryManager::getPath(common_smart_nav::GetPlan::Request  &req, common_smart_nav::GetPlan::Response &res )
{
	//res.sum = req.a + req.b;
	//ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	//ROS_INFO("sending back response: [%ld]", (long int)res.sum);

	std::vector<geometry_msgs::PoseStamped> global_plan;
	nav_msgs::Path tmp_path;

	while (sem < 1) {
		usleep(10000);
	}
	sem = 0;

	//make sure we have a costmap for our planner
	if(planner_costmap_ == NULL){
		ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
		//return false;
	}

	tf::Stamped<tf::Pose> global_pose;
	if(!planner_costmap_->getRobotPose(global_pose)){
		ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
		//return false;
	}

	geometry_msgs::PoseStamped start;
	//if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
	//if(req.start.header.frame_id == "")
	tf::poseStampedTFToMsg(global_pose, start);

	current_pose = start;

	boost::unique_lock< boost::shared_mutex > lock(*(planner_costmap_->getCostmap()->getLock()));
	//boost::unique_lock< costmap_2d::Costmap2D::mutex_t > lock(*(planner_costmap_->getCostmap()->getMutex()));

	if(planner_->makePlan(start, req.goal, global_plan)){
		//ROS_ERROR("planner makes plan");
		if(!global_plan.empty()){
			global_plan.push_back(req.goal);
			//ROS_ERROR("globalplan filled");
		}
	}

	tmp_path.header.frame_id = map_name;
	tmp_path.poses = global_plan;

	res.plan = tmp_path;

	//action_feedback_.feedback = current_pose;
	//as_.publishFeedback(action_feedback_);

	sem = 1;

	return true;
}


void TrajectoryManager::pathDoneCallback(const std_msgs::Empty::ConstPtr & pose)
{
	status = PAUSE;

	if( action_goal == 1 ) {
                //action_result_.result = current_pose;
                as_.setSucceeded(action_result_);
                action_goal = 0;
                ROS_INFO("END POSE (ACTION)");
        }

	//planner_costmap_->resetLayers();
	//planner_costmap_->resetMapOutsideWindow(0.001, 0.001);
}

void TrajectoryManager::pauseCallback(const std_msgs::Empty::ConstPtr & pose)
{
	status = STOP;
	planner_costmap_->pause();
}

void TrajectoryManager::resumeCallback(const std_msgs::Empty::ConstPtr & pose)
{
	status = PAUSE;
	planner_costmap_->resume();
}

void TrajectoryManager::computePathCallback(const std_msgs::Empty::ConstPtr & pose)
{

	computePath();
	publishPath();
	status = RUN;
}

void TrajectoryManager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
{
	final_pose = *pose;
	ROS_INFO("NEW POSE");
	//ROS_ERROR("globalplan GOAL : %lf", getHeadingFromQuat(final_pose.pose.orientation));

	computePath();
	publishPath();

	status = RUN;
}

//void TrajectoryManager::goalActionCallback(const common_smart_nav::move_robotGoalConstPtr &goal)

/**
 *    Compute a vector for each point
 *    of the track with yaw
 */
void TrajectoryManager::computePointHead(nav_msgs::Path& path)
{
	for(int i = 0; i < path.poses.size(); ++i)
	{
		double ux, uy;
		double p1, p2;

		if(i == (path.poses.size() - 1))
		{
			p1 = i;
			p2 = 0;
		}
		else
		{
			p1 = i;
			p2 = i + 1;
		}
		ux = path.poses[p2].pose.position.x - path.poses[p1].pose.position.x;
		uy = path.poses[p2].pose.position.y - path.poses[p1].pose.position.y;
		path.poses[p1].pose.orientation = tf::createQuaternionMsgFromYaw(atan2(uy, ux));
	}
}

/**
 *    Compute a Z for each point
 *    of the track
 */
void TrajectoryManager::computePointZdirection(nav_msgs::Path& path, double value)
{
	for(int i = 0; i < path.poses.size(); ++i)
	{
		path.poses[i].pose.position.z = value;
	}
}

void TrajectoryManager::computePath(void)
{
	std::vector<geometry_msgs::PoseStamped> global_plan;
	nav_msgs::Path tmp_path;

	while (sem < 1) {
		usleep(10000);
	}
	sem = 0;

	//make sure we have a costmap for our planner
	if(planner_costmap_ == NULL){
		ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
		//return false;
	}

	tf::Stamped<tf::Pose> global_pose;



	if(!planner_costmap_->getRobotPose(global_pose)){
		ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
		//return false;
	}

	geometry_msgs::PoseStamped start;
	//if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
	//if(req.start.header.frame_id == "")
	tf::poseStampedTFToMsg(global_pose, start);
	//start = current_pose;
	//start.pose.position.z = 1.0;
	//final_pose.pose.position.z = 1.0;
	current_pose = start;

	//costmap_2d::Costmap2D* pCostmap = planner_costmap_->getCostmap();
	boost::unique_lock< boost::shared_mutex > lock(*(planner_costmap_->getCostmap()->getLock()));
	//boost::unique_lock< costmap_2d::Costmap2D::mutex_t > lock(*(planner_costmap_->getCostmap()->getMutex()));

	if(planner_->makePlan(start, final_pose, global_plan)){
		//ROS_ERROR("planner makes plan");
		if(!global_plan.empty()){
			global_plan.pop_back();
			global_plan.push_back(final_pose);
			//ROS_ERROR("globalplan filled");
		}
	}

	tmp_path.header.frame_id = map_name;
	tmp_path.poses = global_plan;

	/*
	ROS_ERROR("globalplan : %lf", getHeadingFromQuat(tmp_path.poses.back().pose.orientation));
	ROS_ERROR("tmp PATH %lf %lf / %lf %lf %lf %lf", tmp_path.poses.back().pose.position.x, tmp_path.poses.back().pose.position.y,
                                                        tmp_path.poses.back().pose.orientation.x, tmp_path.poses.back().pose.orientation.y,
                                                        tmp_path.poses.back().pose.orientation.z, tmp_path.poses.back().pose.orientation.w);
	ROS_ERROR("final pose %lf %lf / %lf %lf %lf %lf", final_pose.pose.position.x, final_pose.pose.position.y,
                                                        final_pose.pose.orientation.x, final_pose.pose.orientation.y,
                                                        final_pose.pose.orientation.z, final_pose.pose.orientation.w);
	*/
	// lock
	my_path = tmp_path;
	computePointHead(my_path);
	TrajectoryManager::computePointZdirection(my_path, 1.0);

	// unlock

	if( action_goal == 1 ) {
                action_feedback_.base_position = current_pose;
                as_.publishFeedback(action_feedback_);
        }
	//ROS_ERROR("globalplan : %lf", getHeadingFromQuat(my_path.poses.back().pose.orientation));

	sem = 1;

}

double TrajectoryManager::getHeadingFromQuat(geometry_msgs::Quaternion pose)
{
        double tmp = 0.0;
        tmp = asin(2*pose.x*pose.y + 2*pose.z*pose.w);
        //ROS_ERROR("%f %f %f %f / %f", pose.x, pose.y, pose.z, pose.w, atan2(2*pose.y*pose.w-2*pose.x*pose.z , 1 - 2*pose.y*pose.y - 2*pose.z*pose.z));
        //ROS_ERROR("%f %f %f %f / %f", pose.x, pose.y, pose.z, pose.w, asin(2*pose.x*pose.y + 2*pose.z*pose.w));
        //ROS_ERROR("%f %f %f %f / %f", pose.x, pose.y, pose.z, pose.w, atan2(2*pose.x*pose.w-2*pose.y*pose.z , 1 - 2*pose.x*pose.x - 2*pose.z*pose.z));
        if( fabs(atan2(2*pose.y*pose.w-2*pose.x*pose.z , 1 - 2*pose.y*pose.y - 2*pose.z*pose.z)) < 0.1) {
                return tmp;
        }
        else {
                if(tmp >= 0)
                        return 3.1415926 - tmp;
                else
                        return -3.1415926 - tmp;
        }
        //return atan2(2*pose.y*pose.w-2*pose.x*pose.z , 1 - 2*pose.y*pose.y - 2*pose.z*pose.z);

}


void TrajectoryManager::publishPath(void)
{
	path_pub.publish(my_path);
	/*ROS_ERROR("Publish PATH %lf %lf / %lf %lf %lf %lf", my_path.poses.back().pose.position.x, my_path.poses.back().pose.position.y,
							my_path.poses.back().pose.orientation.x, my_path.poses.back().pose.orientation.y, 
							my_path.poses.back().pose.orientation.z, my_path.poses.back().pose.orientation.w);
	*/
}

void TrajectoryManager::planThread(void)
{
	ros::Rate r(20);
	while(nh.ok()) {

		tf::Stamped<tf::Pose> global_pose;
		geometry_msgs::PoseStamped start;
		switch(status) {
			case STOP:
				cpt++;
				if(cpt > 100) {
					cpt = 0;
				}
				if(cpt == 10) {
					ROS_INFO("Trying to reset layers");

					while (sem < 1) {
						usleep(1000);
					}
					sem = 0;
					planner_costmap_->stop();
					planner_costmap_->resetLayers();
					planner_costmap_->start();
					sem = 1;
				}

				break;
			case PAUSE:
				//                costmap_2d::Costmap2D* pCostmap = planner_costmap_->getCostmap();
				//                boost::unique_lock< boost::shared_mutex > lock(*(pCostmap->getLock()));
				if(planner_costmap_ == NULL){
					ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
					//return false;
				}

				if(!planner_costmap_->getRobotPose(global_pose)){
					ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
					//return false;
				}

				//if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
				//if(req.start.header.frame_id == "")
				tf::poseStampedTFToMsg(global_pose, start);

				//ROS_ERROR("PathPlanner : Compute current pose");
				current_pose = start;
				//action_feedback_.base_position = current_pose;
				//as_.publishFeedback(action_feedback_);
				pose_pub.publish(current_pose);


				cpt++;
				if(cpt > 100) {
					cpt = 0;
				}
				if(cpt == 10) {
					ROS_INFO("Trying to reset layers");
					//planner_costmap_->resetLayers();
					//planner_costmap_->getCostmap()->resetMaps();
					//boost::unique_lock< boost::shared_mutex > lock(*(planner_costmap_->getCostmap()->getLock()));
					//planner_costmap_->getCostmap()->resetMap(0,0,100,100);

					while (sem < 1) {
						usleep(1000);
					}
					sem = 0;
					planner_costmap_->stop();
					planner_costmap_->resetLayers();
					planner_costmap_->start();
					sem = 1;
				}


				break;
			case RUN:
				cpt++;
				if(cpt > 100) {
					cpt = 0;
					computePath();
					//computePointHead(my_path);
					publishPath();
				}
				if(cpt == 50) {
					computePath();
					//computePointHead(my_path);
					publishPath();
				}
				if(cpt == 10) {
					ROS_INFO("Trying to reset layers");
					//planner_costmap_->resetLayers();
					//planner_costmap_->getCostmap()->resetMaps();
					//boost::unique_lock< boost::shared_mutex > lock(*(planner_costmap_->getCostmap()->getLock()));
					//planner_costmap_->getCostmap()->resetMap(0,0,100,100);

					while (sem < 1) {
						usleep(1000);
					}
					sem = 0;
					planner_costmap_->stop();
					planner_costmap_->resetLayers();
					planner_costmap_->start();
					sem = 1;
				}
				break;    
			default:
				break;
		}
		r.sleep();
	}

}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{


	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "Trajectory_Manager");
	tf::TransformListener listener(ros::Duration(10));
	TrajectoryManager trajectorymanager(listener, ros::this_node::getName());

	ros::spin();

	ros::shutdown();
}


