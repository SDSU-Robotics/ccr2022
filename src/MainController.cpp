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

//required
using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

#define minSpeed 0
#define maxSpeed 1
#define stepSize 0.1

bool _buttons[12] = { 0 };
double _axes[6] = { 0 };
void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
void getJoyVals(bool buttons[], double axes[]);

void F32Toggle(const bool b, bool &current, bool &on, std_msgs::Float32 &msg);
void ToggleUpDown(const bool down, const bool up, bool &currentButton4, bool &currentButton5, std_msgs::Float32 &message);
void ButtonPressAndHold(bool button, std_msgs::Float32 &msg, bool pressed);
void AxisPressandHold(double axis, std_msgs::Float32 &msg, bool pressed);
void Shutdown(const bool b, bool &current, bool &off, std_msgs::Float32 &lmsg, std_msgs::Float32 &rmsg, std_msgs::Float32 &wmsg);

/*TalonSRX Left = {DeviceIDs::LeftMotorTal};
TalonSRX Right = {DeviceIDs::RightMotorTal};
TalonSRX Weapon = {DeviceIDs::WeaponTal};*/

//function needed by all .cpp files
int main (int argc, char **argv)
{
	//required for ROS to work. The name inside the quotations is the name of this node. Cannot be the same as other node names
    ros::init(argc, argv, "MainController", ros::init_options::AnonymousName);
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

	bool powercurrent = false;
	bool poweron = false;
    
	//required for ROS to work
	while (ros::ok())
	{
        // fill array (2) with values from array (1)
		getJoyVals(buttons, axes);

		F32Toggle(buttons[weaponmotor], weaponcurrent, weaponon, w_motor_msg);
        AxisPressandHold(axes[leftmotor], l_motor_msg, lpress);
		AxisPressandHold(axes[rightmotor], r_motor_msg, rpress);
		Shutdown(buttons[poweroff], powercurrent, poweron, l_motor_msg, r_motor_msg, w_motor_msg);

		// publish speed data
		l_motor.publish(l_motor_msg);
		r_motor.publish(r_motor_msg);
		w_motor.publish(w_motor_msg);
		power.publish(power_msg);
        
		//required for ROS to work
		ros::spinOnce();
		loop_rate.sleep();
	}
	//required for .cpp to work
	return 0;
}

void joyListener(const sensor_msgs::Joy::ConstPtr& Joy)
{
	for (int i = 0 ; i < 12; i++)
		_buttons[i] = Joy->buttons[i];

    for (int i = 0; i < 6; i++)
        _axes[i] = Joy->axes[i];
}

/*
    REQUIRED IN MAIN!
    bool buttons[12];
	double axes[6];

    REQUIRED IN WHILE(ROS::OK) LOOP!
    getJoyVals(buttons, axes);
*/
void getJoyVals(bool buttons[], double axes[])
{
    for (int i = 0; i < 12; i++)
        buttons[i] = _buttons[i];

    for (int i = 0; i < 6; i++)
        axes[i] = _axes[i];
}

/*
    Requirements in main:
    -- make sure that you have a set of these variables for each button you use
    bool current
    bool on
    std_msgs::Float32 msg

    while(ros::ok) loop requirments:
    F32Toggle(buttons[{JoyMap::<button_name>}], current, on, msg);
*/
void F32Toggle(const bool b, bool &current, bool &on, std_msgs::Float32 &msg)
{
	bool prev = current;
	current = b;

	if (prev && !current)
	{
		on = !on;
		//ROS_INFO("PRESSED");
	}
		
	if (on)
	{
		//ROS_INFO("ON");
		msg.data = 1;
	}
	else
	{
		//ROS_INFO("OFF");
		msg.data = 0;
	}
}


void ToggleUpDown(const bool down, const bool up, bool &currentButton4, bool &currentButton5, std_msgs::Float32 &message)
{

	bool lastButton4;
	bool lastButton5; 
	//gets the last state of the buttons
	lastButton4 = currentButton4;
	lastButton5 = currentButton5;
	//sets the last state of the button to the current state of the button
	currentButton5 = up;
	currentButton4 = down;
	// sets the boolean value of current value to the value in keys

	if (lastButton5 && !currentButton5)
	{
		if (message.data < maxSpeed)
			message.data = message.data + stepSize;
		
		else if(message.data >= maxSpeed)
			message.data = maxSpeed;
		
	}
	else if (lastButton4 && !currentButton4)
	{
		if (message.data > minSpeed)
			message.data = message.data - stepSize;
		
		else if (message.data <= minSpeed)
			message.data = minSpeed;
		
	}
} 

void ButtonPressAndHold(bool button, std_msgs::Float32 &msg, bool pressed)
{
    if(button)
        pressed = true;
    
    if (!pressed)
    {
        msg.data = 0;
    }
    else if(pressed && button)
    {
        msg.data = 1;
        pressed = false;
    }
}

void AxisPressandHold(double axis, std_msgs::Float32 &msg, bool pressed)
{
    if(axis != 0)
        pressed = true;
    else
		pressed = false;
	
	if(!pressed)
    {
        msg.data = 0;
    }
    else
    {
        msg.data = axis;
        pressed = false;
    }

	//ROS_INFO("Axis: %f", axis);
	//ROS_INFO("Pressed: %d", pressed);
}

void Shutdown(const bool b, bool &current, bool &off, std_msgs::Float32 &lmsg, std_msgs::Float32 &rmsg, std_msgs::Float32 &wmsg)
{
	bool prev = current;
	current = b;

	if (prev && !current)
	{
		off = !off;
		//ROS_INFO("PRESSED");
	}
		
	if (off)
	{
		//ROS_INFO("ON");
		lmsg.data = 0;
		rmsg.data = 0;
		wmsg.data = 0;
	}
}
