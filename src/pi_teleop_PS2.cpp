#include "ros/ros.h"
#include "ros/param.h"
#include <stdlib.h>
#include "../PiPS2/PiPS2.h"
#include <wiringPi.h>
#include <geometry_msgs/Twist.h>



class Pi_Teleop_PS2 
{
	public:
		Pi_Teleop_PS2();
		void publishTwist(char x, char y, char yaw);
		
	private:
		// Ros nodehandle. Main access point for all ROS comms
		ros::NodeHandle nh;
		
		// Twist message object
		geometry_msgs::Twist twist_msg;
		
		// Twist publisher
		ros::Publisher twistPub;
		
		double _xLimit;
		double _yLimit;
		double _yawLimit;
		int cmdPin;
		int dataPin;
		int clkPin;
		int attnPin;

	
};

// Class constructor
Pi_Teleop_PS2::Pi_Teleop_PS2()
{
	// Create the twist publisher
	twistPub = nh.advertise<geometry_msgs::Twist>("vel_set", 100);
	
	//if (!ros::NodeHandle::getParam("x_limit", _xLimit)) _xLimit = 1.2;
	//if (!ros::NodeHandle::getParam("y_limit", _yLimit)) _yLimit = 1.2;
	//if (!ros::NodeHandle::getParam("yaw_limit", _yawLimit)) _yawLimit = 1.2;
	
	//if (!ros::NodeHandle::getParam("ps2_cmd_pin", cmdPin)) cmdPin = 10;
	//if (!ros::NodeHandle::getParam("ps2_data_pin", dataPin)) dataPin = 9;
	//if (!ros::NodeHandle::getParam("ps2_clk_pin", clkPin)) clkPin = 11;
	//if (!ros::NodeHandle::getParam("ps2_attn_pin", attnPin)) attnPin = 8;
	
	ros::NodeHandle nh_private("~");
	nh_private.param("x_limit", _xLimit, 1.2);
	nh_private.param("y_limit", _yLimit, 1.2);
	nh_private.param("yaw_limit", _yawLimit, 1.2);
	
	nh_private.param("ps2_cmd_pin", cmdPin, 10);
	nh_private.param("ps2_data_pin", dataPin, 9);
	nh_private.param("ps2_clk_pin", clkPin, 11);
	nh_private.param("ps2_attn_pin", attnPin, 8);	
	
}

void Pi_Teleop_PS2::publishTwist(char x, char y, char yaw)
{
	twist_msg.linear.x = _xLimit * ((double)x - 128.0)/128.0;
	twist_msg.linear.y = _yLimit * (127.0 - (double)y)/128.0;
	twist_msg.angular.z = _yawLimit * (128.0-(double)yaw)/128.0;
	
	twistPub.publish(twist_msg);
}

int main(int argc, char** argv)
{
	// Initialise ros node
	ros::init(argc, argv, "pi_teleop_ps2");
	
	// Instantiate the PS2 releop class
	Pi_Teleop_PS2 teleop;
	// loop at 10 Hz
	ros::Rate loop_rate(10);
	
	// Set up witing pi. NOTE: MUST RUN scripts/setPiGpio.sh first to export system pins
	wiringPiSetupSys();
	system("gpio -g mode 10 out");
	system("gpio -g mode 9 in");
	system("gpio -g mode 11 out");
	system("gpio -g mode 8 out");
	
	// Create a PIPS2 object
	PIPS2 pips2;
	
	// Initialise controller	
	if (!pips2.initializeController(10, 9, 11, 8))
	{
		fprintf(stderr, "Failed to configure gamepad\nController is not responding.\nExiting ...\n");
		return -1;
	}
	
	// Now do a re-init to set the mode to all pressures returned
	int returnVal = pips2.reInitializeController(ALLPRESSUREMODE);
	if (returnVal == -1)
	{
		printf("Invalid Mode\n");
	} else if (returnVal == -2)
	{
		printf("Took too many tries to reinit.\n");
	}
	
	// Loop
	while (ros::ok())
	{
		// Read the controller.
		pips2.readPS2();		
		//printf("Right Joy   Horizontal = %d\tVertical = %d\n", pips2.PS2data[5], pips2.PS2data[6]);
		//printf("Left Joy   Horizontal = %d\tVertical = %d\n", pips2.PS2data[7], pips2.PS2data[8]);
		
		teleop.publishTwist(pips2.PS2data[7], pips2.PS2data[8], pips2.PS2data[5]);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}