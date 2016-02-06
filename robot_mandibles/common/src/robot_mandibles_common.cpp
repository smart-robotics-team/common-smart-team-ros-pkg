// ROS message includes
#include "ros/ros.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16.h>

/* protected region user include files on begin */
#define NOSTATE 		0
#define GOTO_0_STEP1 	1
#define GOTO_0_STEP2 	2
#define IN_O 			3
#define GOTO_1_STEP1 	11
#define GOTO_1_STEP2 	12
#define IN_1 			13
#define GOTO_2_STEP1 	21
#define GOTO_2_STEP2 	22
#define IN_2 			23

#define TO_POSITION_0 	0
#define TO_POSITION_1 	1
#define TO_POSITION_2 	2
/* protected region user include files end */

class robot_mandibles_config
{
public:
    int servo1_position0;
    int servo2_position0;
    int servo1_position1;
    int servo2_position1;
    int servo1_position2;
    int servo2_position2;
    int move_speed;
    double wait_step_1_to_2;
};

class robot_mandibles_data
{
// autogenerated: don't touch this class
public:
    //input data
    std_msgs::UInt16 in_position;
    //output data
    std_msgs::UInt16 out_servo_speed;
    bool out_servo_speed_active;
    std_msgs::UInt16 out_servo1;
    bool out_servo1_active;
    std_msgs::UInt16 out_servo2;
    bool out_servo2_active;
};

class robot_mandibles_impl
{
    /* protected region user member variables on begin */
	robot_mandibles_config local_config;

	int state;
	ros::Time timer;
    /* protected region user member variables end */

public:
    robot_mandibles_impl() 
    {
        /* protected region user constructor on begin */
    	state = NOSTATE;
    	timer = ros::Time::now();
        /* protected region user constructor end */
    }

    void configure(robot_mandibles_config config) 
    {
        /* protected region user configure on begin */
    	local_config = config;
        /* protected region user configure end */
    }

    void update(robot_mandibles_data &data, robot_mandibles_config config)
    {
        /* protected region user update on begin */
    	data.out_servo_speed_active = false;
		data.out_servo1_active = false;
		data.out_servo2_active = false;
    	switch(state){
    	case NOSTATE:
    		data.out_servo_speed_active = false;
    		data.out_servo1_active = false;
    		data.out_servo2_active = false;
    		break;
    	case GOTO_0_STEP1:
    	    data.out_servo_speed_active = true;
    	    data.out_servo1_active = false;
    	    data.out_servo2_active = true;
    	    data.out_servo_speed.data = (uint16_t)local_config.move_speed;
    	    data.out_servo1.data = (uint16_t)local_config.servo1_position0;
    	    data.out_servo2.data = (uint16_t)local_config.servo2_position0;
    	    timer = ros::Time::now();
    	    state = GOTO_0_STEP2;
    	    break;
    	case GOTO_0_STEP2:
    		if( (ros::Time::now().toSec() - timer.toSec()) > local_config.wait_step_1_to_2 ){
				data.out_servo_speed_active = true;
				data.out_servo1_active = true;
				data.out_servo2_active = true;
				data.out_servo_speed.data = (uint16_t)local_config.move_speed;
				data.out_servo1.data = (uint16_t)local_config.servo1_position0;
				data.out_servo2.data = (uint16_t)local_config.servo2_position0;
				state = IN_O;
    		}
			break;
    	case IN_O:
			data.out_servo_speed_active = false;
			data.out_servo1_active = false;
			data.out_servo2_active = false;
			data.out_servo_speed.data = (uint16_t)local_config.move_speed;
			data.out_servo1.data = (uint16_t)local_config.servo1_position0;
			data.out_servo2.data = (uint16_t)local_config.servo2_position0;
			break;
    	case GOTO_1_STEP1:
			data.out_servo_speed_active = true;
			data.out_servo1_active = true;
			data.out_servo2_active = false;
			data.out_servo_speed.data = (uint16_t)local_config.move_speed;
			data.out_servo1.data = (uint16_t)local_config.servo1_position1;
			data.out_servo2.data = (uint16_t)local_config.servo2_position1;
			timer = ros::Time::now();
			state = GOTO_1_STEP2;
			break;
    	case GOTO_1_STEP2:
    		if( (ros::Time::now().toSec() - timer.toSec()) > local_config.wait_step_1_to_2 ){
				data.out_servo_speed_active = true;
				data.out_servo1_active = true;
				data.out_servo2_active = true;
				data.out_servo_speed.data = (uint16_t)local_config.move_speed;
				data.out_servo1.data = (uint16_t)local_config.servo1_position1;
				data.out_servo2.data = (uint16_t)local_config.servo2_position1;
				state = IN_1;
    		}
			break;
    	case IN_1:
			data.out_servo_speed_active = false;
			data.out_servo1_active = false;
			data.out_servo2_active = false;
			data.out_servo_speed.data = (uint16_t)local_config.move_speed;
			data.out_servo1.data = (uint16_t)local_config.servo1_position1;
			data.out_servo2.data = (uint16_t)local_config.servo2_position1;
			break;
    	case GOTO_2_STEP1:
			data.out_servo_speed_active = true;
			data.out_servo1_active = true;
			data.out_servo2_active = false;
			data.out_servo_speed.data = (uint16_t)local_config.move_speed;
			data.out_servo1.data = (uint16_t)local_config.servo1_position2;
			data.out_servo2.data = (uint16_t)local_config.servo2_position2;
			timer = ros::Time::now();
			state = GOTO_2_STEP2;
			break;
		case GOTO_2_STEP2:
			if( (ros::Time::now().toSec() - timer.toSec()) > local_config.wait_step_1_to_2 ){
				data.out_servo_speed_active = true;
				data.out_servo1_active = true;
				data.out_servo2_active = true;
				data.out_servo_speed.data = (uint16_t)local_config.move_speed;
				data.out_servo1.data = (uint16_t)local_config.servo1_position2;
				data.out_servo2.data = (uint16_t)local_config.servo2_position2;
				state = IN_2;
			}
			break;
		case IN_2:
			data.out_servo_speed_active = false;
			data.out_servo1_active = false;
			data.out_servo2_active = false;
			data.out_servo_speed.data = (uint16_t)local_config.move_speed;
			data.out_servo1.data = (uint16_t)local_config.servo1_position2;
			data.out_servo2.data = (uint16_t)local_config.servo2_position2;
			break;
    	default:
    		break;
    	}

        /* protected region user update end */
    }

    void topicCallback_position(const std_msgs::UInt16::ConstPtr& msg)
    {
        /* protected region user implementation of subscribe callback for position on begin */
    	switch(msg->data){
    	case TO_POSITION_0:
    		if(state != IN_O)
    			state = GOTO_0_STEP1;
    		break;
    	case TO_POSITION_1:
    		if(state != IN_O)
    		    state = GOTO_1_STEP2;
    		else
    			state = GOTO_1_STEP1;
    	    break;
    	case TO_POSITION_2:
    		if(state != IN_O)
				state = GOTO_2_STEP2;
			else
				state = GOTO_2_STEP1;
    	    break;
    	default:
    		break;
    	}
        /* protected region user implementation of subscribe callback for position end */
    }



    /* protected region user additional functions on begin */
    /* protected region user additional functions end */
};