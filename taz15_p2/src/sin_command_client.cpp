
// SIN COMMANDER CLIENT NODE - TRENT ZIEMER 9/20/2016

#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<taz15_p2/sinCmdSrv.h>
std_msgs::Float64 g_vel_cmd;

int main(int argc, char **argv) {
    ros::init(argc, argv, "an_example_ros_client");
    ros::NodeHandle nh;

    // Declare the user-defined sine wave parameters
    double desAmplitude = 0.0;
    double desFrequency = 0.0;


while(ros::ok())
{
    // Prompt user for input
    std::cout << "Please type an amplitude and frequency specification now:";
    std::cin >> desAmplitude >> desFrequency;

    ros::ServiceClient client = nh.serviceClient<taz15_p2::sinCmdSrv>("sin_cmd_srv");

    // Service and it's message parts
    taz15_p2::sinCmdSrv srv;
    srv.request.amplitude = desAmplitude;
    srv.request.frequency = desFrequency;

	// Send data and check if good...
    if(client.call(srv))
	{
		std::cout << "Client sent request for service and got back a successful received response." << std::endl;
	}
	else
	{
		std::cout << "Client message received no response!" << std::endl;
	}
	// Now ask user for another sine command!
}
    return 0;
}
