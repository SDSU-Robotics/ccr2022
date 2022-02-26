/*//general includes needed for ROS files
#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/Joy.h>
#include "std_msgs/Bool.h"
//include to access JoyMap.h file wich stores all the button mapping for Joystick
#include "JoyMap.h"
#include "DeviceIDs.h"
#include "ControllerFunctions.h"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/motorcontrol/SensorCollection.h"
using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

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
/*void getJoyVals(bool buttons[], double axes[]) const
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
/*void F32Toggle(const bool b, bool &current, bool &on, std_msgs::Float32 &msg)
{
	bool prev = current;
	current = b;

	if (prev && !current)
	{
		on = !on;
		ROS_INFO("PRESSED");
	}
		
	if (on)
	{
		ROS_INFO("ON");
		msg.data = 1;
	}
	else
	{
		ROS_INFO("OFF");
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



void ButtonPressAndHold(bool button, bool pressed, std_msgs::Float32 msg)
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

    ctre::phoenix::unmanaged::FeedEnable(100);
}

void AxisPressandHold(bool axis, std_msgs::Float32 msg, bool pressed)
{
    if(axis)
        pressed = true;
    
    if (!pressed)
    {
        msg.data = 0;
    }
    else if(pressed && axis)
    {
        msg.data = axis;
        pressed = false;
    }

    ctre::phoenix::unmanaged::FeedEnable(100);
}*/