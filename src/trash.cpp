//general includes needed for ROS files
#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt16.h"
#include <sensor_msgs/Joy.h>
#include "std_msgs/Bool.h"
//include to access JoyMap.h file wich stores all the button mapping for Joystick
#include "JoyMap.h"
#include "DeviceIDs.h"
//required
using namespace std;

#define DRIVE_SCALE 1

//class to store functions and data in. Required
class Listener
{
//decalre all functions
public:
	void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
	void getJoyVals(bool buttons[], double axes[]) const;
	void F32Toggle(const bool b, bool &current, bool &on, std_msgs::Float32 &msg);
	void ToggleAugerSpeed(const bool down, const bool up, bool &currentButton4, bool &currentButton5, std_msgs::Float32 &message);
	void toggleServo(const bool b, bool &current, bool &on, std_msgs::UInt16 &msg);
	double stepSize = 0.1;
	double maxSpeed = 1;
	double minSpeed = -1;

//declare array (1)
private:
    bool _buttons[12] = { 0 };
	double _axes[6] = { 0 };
	
};

//function to fill array (1) with Joystick INput
void Listener::joyListener(const sensor_msgs::Joy::ConstPtr& Joy)
{
	for (int i = 0 ; i < 12; i++)
		_buttons[i] = Joy->buttons[i];

    for (int i = 0; i < 6; i++)
        _axes[i] = Joy->axes[i];
}

//function to fill array (2) with values from array(1)
void Listener::getJoyVals(bool buttons[], double axes[]) const
{
    for (int i = 0; i < 12; i++)
        buttons[i] = _buttons[i];

    for (int i = 0; i < 6; i++)
        axes[i] = _axes[i];
}

void Listener::F32Toggle(const bool b, bool &current, bool &on, std_msgs::Float32 &msg)
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

void Listener::toggleServo(const bool b, bool &current, bool &on, std_msgs::UInt16 &msg)
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
		msg.data = 180;
	}
	else
	{
		ROS_INFO("OFF");
		msg.data = 0;
	}
}

void Listener::ToggleAugerSpeed(const bool down, const bool up, bool &currentButton4, bool &currentButton5, std_msgs::Float32 &message)
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


//function needed by all .cpp files
int main (int argc, char **argv)
{
	//required for ROS to work. The name inside the quotations is the name of this node. Cannot be the same as other node names
    ros::init(argc, argv, "ManualDriveController");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	//initialize the Listner class to access the function we need. Required
	Listener listener;

	//listen to any joystick input and call the appropriate listener function accordingly. Required
	ros::Subscriber joySub = n.subscribe("Excv/joy", 100, &Listener::joyListener, &listener);
	
	//create array (2). Required
	bool buttons[12];
	double axes[6];

	int augerUp = {JoyMap::AugerToggleUp};
	int augerDown = {JoyMap::AugerToggleDown};
	bool augerCurrent = false; // initialized to false
	bool augerOn = false; // initialized to false

	int linAct = {JoyMap::LinActToggle};
	bool linActCurrent = false; // initialized to false
	bool linActOn = false; // initialized to false

	int servo = {JoyMap::Servo};
	bool servoCurrent = false;
	bool servoOn = false;

	//get the number corresponding to the joystick's left stick and right stick
	int Lstick_Yaxis = {JoyMap::LeftDrive};
    int Rstick_Yaxis = {JoyMap::RightDrive};

	int leftTrigger = {JoyMap::LeftTrigger};
    int rightTrigger = {JoyMap::RightTrigger};

	// publishers: one for each message that we are broadcasting. The name within the quations must be unique (this is the TOPIC).
    ros::Publisher r_drive_pub = n.advertise<std_msgs::Float32>("RDrvPwr", 100);
	ros::Publisher l_drive_pub = n.advertise<std_msgs::Float32>("LDrvPwr", 100);
	ros::Publisher auger_toggle_pub = n.advertise<std_msgs::Float32>("AugerToggle", 100);
	ros::Publisher lin_act_toggle_pub = n.advertise<std_msgs::Float32>("LinActToggle", 100);
	ros::Publisher servo_toggle_pub = n.advertise<std_msgs::UInt16>("ServoToggle", 100);
	ros::Publisher augerRaisePub = n.advertise<std_msgs::Int32>("Auger_Raise", 100);
	ros::Publisher augerLowerPub = n.advertise<std_msgs::Int32>("Auger_Lower", 100);

	// messages: one for each publisher above. Data type must match that of the publisher
    std_msgs::Float32 l_speed_msg;
    std_msgs::Float32 r_speed_msg;
	std_msgs::Float32 auger_toggle_msg;
	std_msgs::Float32 lin_act_toggle_msg;
	std_msgs::UInt16 servo_toggle_msg;
	std_msgs::Int32 augerRaise;
	std_msgs::Int32 augerLower;

	//required for ROS to work
	while (ros::ok())
	{
        // fill array (2) with values from array (1)
		listener.getJoyVals(buttons, axes);

		// get controller values
		float speed = axes[1]; // left Y
		float turn = 1 * axes[3]; // right X

		// get speed data
		l_speed_msg.data = axes[Lstick_Yaxis] * DRIVE_SCALE; // left up/down stick
		r_speed_msg.data = axes[Rstick_Yaxis] * DRIVE_SCALE; // right up/down stick
		// publish speed data
		l_drive_pub.publish(l_speed_msg);
		r_drive_pub.publish(r_speed_msg);

		// toggle Y button to turn auger motor on/off
		//listener.F32Toggle(buttons[auger], augerCurrent, augerOn, auger_toggle_msg);
		listener.ToggleAugerSpeed(buttons[augerDown], buttons[augerUp], augerCurrent, augerOn, auger_toggle_msg);
		auger_toggle_pub.publish(auger_toggle_msg);

		// toggle A button to toggle linear actuator
		listener.F32Toggle(buttons[linAct], linActCurrent, linActOn, lin_act_toggle_msg);
		lin_act_toggle_pub.publish(lin_act_toggle_msg);

		// toggle B button to toggle servo
		listener.toggleServo(buttons[servo], servoCurrent, servoOn, servo_toggle_msg);
		servo_toggle_pub.publish(servo_toggle_msg);
		
		// Lower(LT) and Raise(RT) Auger
		augerRaise.data = axes[leftTrigger]*-1;
    	augerLower.data = axes[rightTrigger]*-1;

		augerRaisePub.publish(augerRaise);
		augerLowerPub.publish(augerLower);

		//required for ROS to work
		ros::spinOnce();
		loop_rate.sleep();
	}
	//required for .cpp to work
	return 0;
}