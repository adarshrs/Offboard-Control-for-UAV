#ifndef _DroneControl_H
#define _DroneControl_H

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <thread>
#include <atomic>
#include "PID/PID.h"


// MAV_FRAME
/*  uint8 FRAME_GLOBAL = 0                   # Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
    uint8 FRAME_LOCAL_NED = 1                # Local coordinate frame, Z-up (x: north, y: east, z: down).
    uint8 FRAME_MISSION = 2                  # NOT a coordinate frame, indicates a mission command.
    uint8 FRAME_GLOBAL_RELATIVE_ALT = 3      # Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
    uint8 FRAME_LOCAL_ENU = 4                # Local coordinate frame, Z-down (x: east, y: north, z: up)
    uint8 FRAME_GLOBAL_INT = 5               # Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL)
    uint8 FRAME_GLOBAL_RELATIVE_ALT_INT = 6  # Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
    uint8 FRAME_LOCAL_OFFSET_NED = 7         # Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
    uint8 FRAME_BODY_NED = 8                 # Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
    uint8 FRAME_BODY_OFFSET_NED = 9          # Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
    uint8 FRAME_GLOBAL_TERRAIN_ALT = 10      # Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
    uint8 FRAME_GLOBAL_TERRAIN_ALT_INT = 11  # Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
*/


// Basic modes from MAV_MODE
/*  uint8 MAV_MODE_PREFLIGHT = 0
    uint8 MAV_MODE_STABILIZE_DISARMED = 80
    uint8 MAV_MODE_STABILIZE_ARMED = 208
    uint8 MAV_MODE_MANUAL_DISARMED = 64
    uint8 MAV_MODE_MANUAL_ARMED = 192
    uint8 MAV_MODE_GUIDED_DISARMED = 88
    uint8 MAV_MODE_GUIDED_ARMED = 216
    uint8 MAV_MODE_AUTO_DISARMED = 92
    uint8 MAV_MODE_AUTO_ARMED = 220
    uint8 MAV_MODE_TEST_DISARMED = 66
    uint8 MAV_MODE_TEST_ARMED = 194
*/

struct drone_position_global
{
    float N;
    float E;
    float D;
};

struct velocity
{
    float F;
    float R;
    float D;
};

//====================================================================================================================================
//==================================      Velocity Controller Class      =============================================================
//====================================================================================================================================

class VelocityController
{

    drone_position_global p; //Pose of drone in world frame (NED according to px4)
    velocity v;             //Velocity of drone in body frame (FRD according to px4)

    float max_roll;         //Max velocity along roll in m/s
    float max_pitch;        //Max velocity along pitch in m/s

    ros::NodeHandle nh;                     //Node handler for publishing, subscribing, etc
    
    ros::ServiceClient mode_client;         //Service client for changing mode of drone
    ros::ServiceClient arming_client;       //Service client for arming of drone
    ros::ServiceClient frame_client;        //Service client for changing frame of drone
    ros::ServiceClient takeoff_client;      //Service client for takeoff
    ros::ServiceClient land_client;         //Service client for landing

    std::thread *altitude_manager;          //Hold and change altitude
    PID altitude_pid;                       //PID controller for altitude
    std::atomic<bool> alt_release;          //Flag to release altitude PID thread
    std::atomic<float> target_altitude;     //Target altitude to be set for drone
    std::atomic<float> current_altitude;    //Current altitude of the drone
    std::atomic<float> throttle_effort;     //Throttle value to be supplied to the drone

    std::thread *move_manager;              //Move about
    std::atomic<bool> landing;              //Flag to set when landing
    std::atomic<float> roll_effort;         //Roll effort to be set
    std::atomic<float> pitch_effort;        //Pitch effort to be set
    std::atomic<float> yaw_effort;          //Yaw effort to be set

    //geometry_msgs::TwistStamped msg_takeoff;    //Message for velocity command
    mavros_msgs::PositionTarget msg_takeoff;

    ros::Subscriber gps_sub;                    //Subscriber for GPS data
    ros::Publisher vel_pub;                     //Publisher for velocity data

    void set_throttle();
    void publish_mover();

    public : 
        VelocityController();
        VelocityController(char*, char*);
        ~VelocityController();

        void get_altitude(const nav_msgs::Odometry &); //Callback to set altitude of the drone from GPS data

        int arm();             //Arm the drone
        int set_frame(int);    //Change frame
        int takeoff(float);    //Takeoff
        int land();            //Land

        void move(float, float, float);         //Roll, pitch, yaw in m/s
        void position_hold();                   //Hold altitude with zero roll pitch yaw
        void set_altitude(float);               //Sets altitude to a particular value using PID

        drone_position_global get_pose();
};

#endif
