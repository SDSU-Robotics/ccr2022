#ifndef CONTROLLER_FUNCTIONS
#define CONTROLLER_FUNCTIONS

bool _buttons[12] = { 0 };
double _axes[6] = { 0 };
void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
void getJoyVals(bool buttons[], double axes[]);

void F32Toggle(const bool b, bool &current, bool &on, std_msgs::Float32 &msg);
void ToggleUpDown(const bool down, const bool up, bool &currentButton4, bool &currentButton5, std_msgs::Float32 &message);
void ButtonPressAndHold (TalonSRX Talon, bool cw, bool pressed);
void AxisPressandHold(bool axis, TalonSRX Talon, std_msgs::Float32 msg, bool pressed);



#endif