#ifndef DEVICEIDS_H
#define DEVICEIDS_H

namespace DeviceIDs
{
    /* to name a button/axis:
        static const int <name> = <number>;
        - the name is whatever is appropriate
        - check the Talon for the ID number
    */
	static const int LeftMotorTal = 1;
    static const int RightMotorTal = 2;
    static const int WeaponTal = 3;
};

#endif