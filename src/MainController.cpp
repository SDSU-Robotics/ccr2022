//general includes needed for ROS files
#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/Joy.h>
#include "std_msgs/Bool.h"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/motorcontrol/SensorCollection.h"

//include to access JoyMap.h file wich stores all the button mapping for Joystick
#include "JoyMap.h"
#include "DeviceIDs.h"
#include "ControllerFunctions.h"
//required
using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


TalonSRX Left = {DeviceIDs::LeftMotorTal};
TalonSRX Right = {DeviceIDs::RightMotorTal};
TalonSRX Weapon = {DeviceIDs::WeaponTal};

//function needed by all .cpp files
int main (int argc, char **argv)
{
	//required for ROS to work. The name inside the quotations is the name of this node. Cannot be the same as other node names
    ros::init(argc, argv, "MainController");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	//listen to any joystick input and call the appropriate listener function accordingly. Required
	ros::Subscriber joySub = n.subscribe("Excv/joy", 100, joyListener);
	
	//create array for buttons/axes Required
	bool buttons[12];
	double axes[6];

	// publishers: one for each message that we are broadcasting. The name within the quotations must be unique (this is the TOPIC).
	ros::Publisher l_motor = n.advertise<std_msgs::Float32>("LMotor", 100); // left motor
	ros::Publisher r_motor = n.advertise<std_msgs::Float32>("RMotor", 100); // right motor
	ros::Publisher w_motor = n.advertise<std_msgs::Float32>("WMotor", 100); // weapon motor
	ros::Publisher power = n.advertise<std_msgs::Float32>("PowerOff", 100); // power shutoff

	// button/axis mapping
	const int leftmotor = {JoyMap::LeftMotor};
	const int rightmotor = {JoyMap::RightMotor};
	const int weaponmotor = {JoyMap::Weapon};
	const int poweroff = {JoyMap::PowerOff};

	// messages: one for each publisher above. Data type must match that of the publisher
	std_msgs::Float32 l_motor_msg;
	std_msgs::Float32 r_motor_msg;
	std_msgs::Float32 w_motor_msg;
	std_msgs::Float32 power_msg;

	bool lpress = false;
	bool rpress = false;

	bool weaponcurrent = false;
	bool weaponon = false;
    
	//required for ROS to work
	while (ros::ok())
	{
        // fill array (2) with values from array (1)
		getJoyVals(buttons, axes);


		// publish speed data
		l_motor.publish(l_motor_msg);
		r_motor.publish(r_motor_msg);
		w_motor.publish(w_motor_msg);
		power.publish(power_msg);

		F32Toggle(buttons[weaponmotor], weaponcurrent, weaponon, w_motor_msg);
        AxisPressandHold(axes[leftmotor], Left, l_motor_msg, lpress);
		AxisPressandHold(axes[rightmotor], Right, r_motor_msg, rpress);
        
		//required for ROS to work
		ros::spinOnce();
		loop_rate.sleep();
	}
	//required for .cpp to work
	return 0;
}