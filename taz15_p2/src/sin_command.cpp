
// SIN COMMANDER NODE MODIFIED TO BE A SERVER FOR HW2 - TRENT ZIEMER 9/20/2016

#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<taz15_p2/sinCmdSrv.h>
//global variables for callback functions to populate for use in main program
std_msgs::Float64 g_vel_cmd;

// Global pointers so that the callback and int main() can both access the publisher
ros::NodeHandle * g_nh;
ros::Publisher * g_pub;

double g_client_command_amplitude;
double g_client_command_frequency;

bool callback(taz15_p2::sinCmdSrvRequest& request, taz15_p2::sinCmdSrvResponse& response)
{
   ROS_INFO("Server received a client request command.");
   
// Set global variables here based on received request data
	g_client_command_amplitude = request.amplitude;
	g_client_command_frequency = request.frequency;

// Junk ---->
	response.received = true;

	return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sin_command");

	ros::NodeHandle nh;

	// Declare variables, with global pointers for use by callbacks
	g_nh = &nh;
	ros::Publisher pub = g_nh->advertise<std_msgs::Float64>("vel_cmd", 1);
	g_pub = &pub;
	ros::start();
    ros::ServiceServer service = nh.advertiseService("sin_cmd_srv", callback);

   // Continually update vel_cmd, spinning once to potentially update from callback action
   while(ros::ok())
	{
        double vel_cmd = g_client_command_amplitude*sin(g_client_command_frequency*ros::Time::now().toSec());
	// Assign vel command to publishable object
        g_vel_cmd.data = vel_cmd;
	// Now publish

        g_pub->publish(g_vel_cmd); 
	// Log info for user to cout        
	ROS_INFO("Velocity Command Sent out is = %f", g_vel_cmd.data);
	ros::spinOnce();
	}

    return 0; // Should never get here, unless roscore dies!
}
