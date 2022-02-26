#include "ctre/Phoenix.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "DeviceIDs.h"
#include "ctre/phoenix/motorcontrol/SensorCollection.h"
#include <iostream>
#include <string>

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

#define LINEAR_ADJ 1
#define ANGULAR_ADJ 1

void getLSpeed(const std_msgs::Float32 lspeed);
void getRSpeed(const std_msgs::Float32 rspeed);
void getWSpeed(const std_msgs::Float32 wspeed);

void setMotorOutput();
void getMotorStatus(const std_msgs::Float32 motor);
bool motor_status = 0;
float leftPower = 0;
float rightPower = 0;
float weaponPower = 0;
int targetPos = 1000;

TalonSRX leftmotor = {DeviceIDs::LeftMotorTal};
TalonSRX rightmotor = {DeviceIDs::RightMotorTal};
TalonSRX weaponmotor = {DeviceIDs::WeaponTal};


int main (int argc, char **argv)
{
    ros::init(argc, argv, "MainDrive");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

    // sets the message to the message variable
	std_msgs::Float32 l_current_msg;
	std_msgs::Float32 r_current_msg;
    std_msgs::Float32 w_current_msg;

	phoenix::platform::can::SetCANInterface("can0");

	ros::Subscriber l_motor_sub = n.subscribe("LMotor", 100, getLSpeed);
	ros::Subscriber r_motor_sub = n.subscribe("RMotor", 100, getRSpeed);
    ros::Subscriber w_motor_sub = n.subscribe("WMotor", 100, getWSpeed);


	while (ros::ok())
	{
		setMotorOutput();
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void setMotorOutput()
{
	leftmotor.Set(ControlMode::PercentOutput, leftPower);
	rightmotor.Set(ControlMode::PercentOutput, rightPower);
    weaponmotor.Set(ControlMode::PercentOutput, weaponPower);
	
	ctre::phoenix::unmanaged::FeedEnable(100);	
}

void getLSpeed(const std_msgs::Float32 lspeed)
{
	leftPower = lspeed.data;
}

void getRSpeed(const std_msgs::Float32 rspeed)
{
	rightPower = rspeed.data;
}

void getWSpeed(const std_msgs::Float32 wspeed)
{
    weaponPower = wspeed.data;
}

void getMotorStatus(const std_msgs::Float32 motor)
{
	motor_status = motor.data;
}