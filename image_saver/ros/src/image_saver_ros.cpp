// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <image_saver/image_saverConfig.h>

// ROS message includes
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>

// other includes
#include <image_saver_common.cpp>


class image_saver_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<image_saver::image_saverConfig> server;
    dynamic_reconfigure::Server<image_saver::image_saverConfig>::CallbackType f;

    ros::Publisher save_done_;
    ros::Subscriber save_picture_;
    ros::Subscriber video_input_;

    image_saver_data component_data_;
    image_saver_config component_config_;
    image_saver_impl component_implementation_;

    image_saver_ros() : np_("~")
    {
        f = boost::bind(&image_saver_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        save_done_ = n_.advertise<std_msgs::String>("save_done", 1);
        save_picture_ = n_.subscribe("save_picture", 1, &image_saver_impl::topicCallback_save_picture, &component_implementation_);
        video_input_ = n_.subscribe("video_input", 1, &image_saver_impl::topicCallback_video_input, &component_implementation_);

        np_.param("save_path", component_config_.save_path, (std::string)"~/save.jpg");
    }
    void topicCallback_save_picture(const std_msgs::Empty::ConstPtr& msg)
    {
        component_data_.in_save_picture = *msg;
    }
    void topicCallback_video_input(const sensor_msgs::Image::ConstPtr& msg)
    {
        component_data_.in_video_input = *msg;
    }

    void configure_callback(image_saver::image_saverConfig &config, uint32_t level)
    {
        component_config_.save_path = config.save_path;
        configure();
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_save_done_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_save_done_active)
            save_done_.publish(component_data_.out_save_done);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "image_saver");

    image_saver_ros node;
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
