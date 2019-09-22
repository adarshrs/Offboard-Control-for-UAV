#include <ros/ros.h>
#include <ros/time.h>
#include "PID.h"

PID::PID()
{
    initialized = false;
}

PID::PID(float init_g, params init_k, float init_rate, limits init_effort, limits init_i_error)
{
    first = true;

    g = init_g;
    k.p = init_k.p;
    k.i = init_k.i;
    k.d = init_k.d;
    rate = init_rate;

    effort.max = init_effort.max;
    effort.min = init_effort.min;

    i_error.max = init_i_error.max;
    i_error.min = init_i_error.min;

    initialized = true;
    target_set = false;
}

void PID::init(float init_g, params init_k, float init_rate, limits init_effort, limits init_i_error)
{
    first = true;
    g = init_g;
    k.p = init_k.p;
    k.i = init_k.i;
    k.d = init_k.d;
    rate = init_rate;

    effort.max = init_effort.max;
    effort.min = init_effort.min;

    i_error.max = init_i_error.max;
    i_error.min = init_i_error.min;

    initialized = true;
    target_set = false;
}

void PID::set_target(float t)
{
    if(!initialized)
        throw std::runtime_error("PID Not initialized");

    target = t;
    target_set = true;
}

float PID::get_effort(float current)
{
    if (!initialized)
        throw std::runtime_error("PID Not initialized");

    if (!target_set)
        throw std::runtime_error("No target set");

    if(first == true)
        e.p = g * (target - current);

    float error = g*(target - current);
    
    e.d = (error - e.p) * rate;
    e.p = error;
    e.i += error / rate;

    if(e.i>i_error.max)
        e.i = i_error.max;

    else if(e.i<i_error.min)
        e.i = i_error.min;

    float new_effort = k.p * e.p + k.i * e.i + k.d * e.d;

    if(new_effort>effort.max)
        new_effort = effort.max;

    else if(new_effort<effort.min)
        new_effort = effort.min;

    return new_effort;
}