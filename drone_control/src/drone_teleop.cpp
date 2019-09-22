#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <ncurses.h>
#include <atomic>
#include <thread>
#include "DroneControl/DroneControl.h"
#include "PID/PID.h"

//Shutdown handler
    // Signal-safe flag for whether shutdown is requested
    sig_atomic_t volatile g_request_shutdown = 0;

    // Replacement SIGINT handler
    void mySigIntHandler(int sig)
    {
        g_request_shutdown = 1;
    }

    // Replacement "shutdown" XMLRPC callback
    void shutdownCallback(XmlRpc::XmlRpcValue &params, XmlRpc::XmlRpcValue &result)
    {
        int num_params = 0;
        if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
            num_params = params.size();
        if (num_params > 1)
        {
            std::string reason = params[1];
            ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
            g_request_shutdown = 1; // Set flag
        }

        result = ros::xmlrpc::responseInt(1, "", 0);
    }
//

std::atomic<char> input('k');

void get_input()
{
    while(input!='q')
    {
        input = getch();
    }
}


int main(int argc, char** argv)
{
    // Override SIGINT handler
        ros::init(argc, argv, "alt_hold_test", ros::init_options::NoSigintHandler);
        signal(SIGINT, mySigIntHandler);
    //

    // Override XMLRPC shutdown
        ros::XMLRPCManager::instance()->unbind("shutdown");
        ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
    //

    sleep(1);

    // Create velocity controller instance for drone
    VelocityController iris;

    ros::Rate rate(10);

    // Arm the drone
    if (!iris.arm())
        return -1;

    // Set frame of reference of drone
    iris.set_frame(8); // 8 = Body NED Frame

    if(!iris.takeoff(2))
        return -1;

    sleep(2);

    initscr();

    iris.set_altitude(4);

    float current_alt = 4;
    float vel_roll = 0;
    float vel_pitch = 0;

    char prev_input = '\0';
    std::thread input_reader(&get_input);

    while(!g_request_shutdown)
    {
        mvprintw(0, 0, "Roll right:     l\nRoll left:      j\nPitch forward:  i\nPitch back:     ,\nMove up:        u\nMove down:      o\nStop: k\n\nSent Altitude: %f\n\n", current_alt);

        iris.move(vel_roll, vel_pitch, 0);

        if(prev_input!=input)
        {
            
            prev_input = input;

            switch(input)
            {
                case 'i':
                            vel_pitch = 0.75;
                            vel_roll = 0;
                            mvprintw(11,0,"\rPitching forward\t\t");
                            break;
                case ',':
                            vel_pitch = -0.75;
                            vel_roll = 0;
                            mvprintw(11, 0, "\rPitching backward\t\t");
                            break;
                case 'l':

                            vel_roll = 0.75;
                            vel_pitch = 0;
                            mvprintw(11, 0, "\rRolling right\t\t");
                            break;
                case 'j':
                            vel_roll = -0.75;
                            vel_pitch = 0;
                            mvprintw(11, 0, "\rRolling left\t\t");
                            break;
                case 'u':
                            prev_input = '\0';
                            current_alt += 0.3;
                            iris.set_altitude(current_alt);
                            mvprintw(11, 0, "\rMoving up\t\t\t\t");
                            break;
                case 'o':
                            prev_input = '\0';
                            current_alt -= 0.3;
                            iris.set_altitude(current_alt);
                            mvprintw(11, 0, "\rMoving down\t\t\t\t");
                            break;
                
                case 'k':
                            vel_pitch = 0;
                            vel_roll = 0;
                            iris.position_hold();
                            current_alt = iris.current_altitude;
                            mvprintw(11, 0, "\rStop\t\t\t\t");
                            break;

                case 'q':
                            g_request_shutdown = 1;
                            break;
            }
        }

        refresh();
        rate.sleep();
    }

    endwin();

    input_reader.join();

    iris.land();
}
