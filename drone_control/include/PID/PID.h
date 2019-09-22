#ifndef _PID_H
#define _PID_H

#include <ros/ros.h>
#include <ros/time.h>

struct params
{
    float p;    //Proportional Gain
    float i;    //Integral Gain
    float d;    //Derivative Gain
};

struct limits
{
    float min;
    float max;
};

class PID
{
    params k;       //Gains of the PID controller
    params e;       //Errors of the PID controller

    float target;   //Target value to be achieved by PID
    float g;        //Scaling factor for PID
    float rate;     //Rate of PID controller

    limits effort;   //Limits of output value to be returned
    limits i_error;  //Limits of allowable integral error for PID

    bool initialized;   //Flag to check if PID controller is initialized
    bool target_set;    //Flag to check if PID controller has target set

    bool first;       //Flag to check if PID controller is in first iteration       

    public:
            PID();
            PID(float, params, float, limits, limits);       //Initializes scaling gain, PID gains and limits
            void init(float,params,float, limits, limits);   //Function to initialize PID controller
            void set_target(float);                          //Sets target value for PID
            float get_effort(float);                         //Takes in current value and returns control effort to achieve target value
};

#endif
