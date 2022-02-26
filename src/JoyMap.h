#ifndef JOYMAP_H
#define JOYMAP_H

namespace JoyMap
{
    /* to name a button/axis:
        static const int <name> = <number>;
        - the name is whatever is appropriate
        - check the comment block below for the right number 
    */
    
    static const int PowerOff = 8; // Power/Xbox button
    static const int Weapon = 0;   // A

    static const int LeftMotor = 1;  // Up/Down Axis stick left
    static const int RightMotor = 4; // Up/Down Axis stick right
};

#endif
/***************************************
    joy.buttons:
    Index   Button
    0       A
    1       B
    2       X
    3       Y
    4       LB
    5       RB
    6       back
    7       start
    8       power/Xbox button
    9       Button stick left
    10      Button stick right
    
    joy.axes:
    Index   Axis
    0       Left/Right Axis stick left
    1       Up/Down Axis stick left
    2       LT
    3       Left/Right Axis stick right
    4       Up/Down Axis stick right
    5       RT
    6       cross key left/right
    7       cross key up/down 
****************************************/