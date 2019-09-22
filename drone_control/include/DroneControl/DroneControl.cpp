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
#include "DroneControl.h"
#include "PID/PID.h"

VelocityController::VelocityController()
{
        p.N = 0;
        p.E = 0;
        p.D = 0;

        v.F = 0;
        v.R = 0;
        v.D = 0;

    
    // Subscribe to GPS data

        gps_sub = nh.subscribe("/mavros/local_position/odom", 1000, &VelocityController::get_altitude, this);
        vel_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        // Set mode to prearm, register guided mode
        
        mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        mavros_msgs::SetMode srv_setmode;
        srv_setmode.request.base_mode = 0;
        srv_setmode.request.custom_mode = "GUIDED";
        mode_client.call(srv_setmode);
    
        if (mode_client.call(srv_setmode))
        {
            ROS_INFO("SetMode Success");
        }

        else
        {
            throw std::runtime_error("FATAL ERROR: Unable to set mode. Exiting Program...\n");
        }

    // Altitude manager
        params alt_gains = {0.3, 0.002, 0.04};      //Default Values
        limits effort = {-2,2};
        limits i_error = {-2,2};
        altitude_pid.init(1, alt_gains, 10, effort, i_error);
    

    pitch_effort = 0;
    roll_effort = 0;
    yaw_effort = 0;

    // Sleep to allow enough time for changes to effect

        sleep(2);
}

void VelocityController::set_throttle()
{
    ros::Rate rate(10);
    
    while(!alt_release)
    {
        altitude_pid.set_target(target_altitude);
        
        throttle_effort = altitude_pid.get_effort(current_altitude);

        ros::spinOnce();
        rate.sleep();
    }    
}

void VelocityController::publish_mover()
{
    ros::Rate rate(10);

    while(!landing)
    {
        // Set according to the FRD frame used by px4
        msg_takeoff.velocity.y = pitch_effort;
        msg_takeoff.velocity.x = roll_effort;
        msg_takeoff.yaw = yaw_effort;
        msg_takeoff.yaw_rate = 0.5;
        msg_takeoff.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        msg_takeoff.type_mask = 0b0000011111000100;

        //if(throttle_effort < 0.05 && throttle_effort>-0.05)
          //  throttle_effort = 0;

        msg_takeoff.velocity.z = throttle_effort;
        vel_pub.publish(msg_takeoff);

        ros::spinOnce();
        rate.sleep();

    }
}

void VelocityController::set_altitude(float a)
{
    target_altitude = a;
}

void VelocityController::get_altitude(const nav_msgs::Odometry &odom)
{
    p.D = odom.pose.pose.position.z; //GPS position returned in ENU frame, converting to NED used by px4
    current_altitude = p.D;
}

int VelocityController::arm()
{
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if (arming_client.call(srv) && srv.response.success)
    {
        ROS_INFO("ARM successful");
        sleep(2);
        return 1;
    }
    else
    {
        ROS_ERROR("ARM FAILED");
        return 0;
    }
}

int VelocityController::set_frame(int frame_num)
{
    frame_client = nh.serviceClient<mavros_msgs::SetMavFrame>("/mavros/setpoint_velocity/mav_frame");
    mavros_msgs::SetMavFrame srv_frame;
    srv_frame.request.mav_frame = frame_num;    // 8 = Body NED Frame
    
    if (frame_client.call(srv_frame) && srv_frame.response.success)
    {
        ROS_INFO("Frame changed to BODY NED");
        sleep(3);
        return 1;
    }
    
    else
    {
        ROS_ERROR("FRAME CHANGE FAILED");
        return 0;
    }
}

int VelocityController::takeoff(float height)
{
    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = height;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    
    if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    {
        ROS_INFO("Takeoff started...");
        sleep(5);
        ROS_INFO("Takeoff successful to height %f", p.D);
        target_altitude = height;
        alt_release = false;
        landing = false;
        altitude_manager = new std::thread(&VelocityController::set_throttle, this);
        move_manager = new std::thread(&VelocityController::publish_mover, this);
        return 1;
    }

    else
    {
        ROS_ERROR("TAKEOFF FAILED");
        return 0;
    }
}

int VelocityController::land()
{
    alt_release = true;
    altitude_manager->join();
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = -1*p.D;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if (land_client.call(srv_land) && srv_land.response.success)
    {
        ROS_INFO("Land Complete");
    }

    else
    {
        ROS_ERROR("LAND FAILED");
    }
}

void VelocityController::move(float roll, float pitch, float yaw)
{
    roll_effort = roll;
    pitch_effort = pitch;
    yaw_effort = yaw;   
}

void VelocityController::position_hold()
{
    move(0,0,0);
    set_altitude(current_altitude);
}

drone_position_global VelocityController::get_pose()
{
    return p;
}

VelocityController::~VelocityController()
{
    alt_release = true;
    landing = true;
    altitude_manager->join();
    move_manager->join();
    throttle_effort = 0;
    move(0, 0, 0);
    sleep(3);
}