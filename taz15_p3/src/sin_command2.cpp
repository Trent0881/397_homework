// SIN COMMANDER NODE MODIFIED TO BE AN ACTION SERVER FOR HW3 - TRENT ZIEMER 9/27/2016

#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<actionlib/server/simple_action_server.h>
#include<taz15_p3/commandActionAction.h>
//global variables for callback functions to populate for use in main program
std_msgs::Float64 g_vel_cmd;

// Global pointers so that the callback and int main() can both access the publisher

ros::Publisher * g_pub;

double g_client_command_amplitude;
double g_client_command_frequency;
double g_client_command_cycles;

class MyActionServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<taz15_p3::commandActionAction> simple_server_obj_;

    taz15_p3::commandActionGoal action_goal_obj_;
    taz15_p3::commandActionFeedback action_feedback_obj_;
    taz15_p3::commandActionResult action_result_obj_;
public:
    MyActionServer();

    ~MyActionServer(void){};

    void executeCB(const actionlib::SimpleActionServer<taz15_p3::commandActionAction>::GoalConstPtr & goal);
};


// MyActionServer class constructor
MyActionServer::MyActionServer() : simple_server_obj_ (nh_, "action_server_name", boost::bind(&MyActionServer::executeCB, this, _1), false)
{
    // Start the server by calling the server objects method
    simple_server_obj_.start();
}

// method thiiing
void MyActionServer::executeCB(const actionlib::SimpleActionServer<taz15_p3::commandActionAction>::GoalConstPtr & goal)
{
    g_client_command_amplitude = goal->amplitude;
    g_client_command_frequency = goal->frequency;
    g_client_command_cycles = goal->cycles;

    ros::Time first_cycle_time = ros::Time::now();
    ros::Duration command_duration = ros::Duration(g_client_command_cycles/g_client_command_frequency);

    double vel_cmd;
    std::cout << "The resulting duration is: " << command_duration << std::endl << std::endl;
    while((ros::Time::now() - first_cycle_time) < command_duration)
    {
        vel_cmd = g_client_command_amplitude*sin(g_client_command_frequency*ros::Time::now().toSec()*2*3.141592657);
        // Assign vel command to global publishable object
        g_vel_cmd.data = vel_cmd;
        //std::cout << g_client_command_amplitude << g_client_command_frequency << g_client_command_cycles << std::endl;
        ros::spinOnce();
    }
    // When we are done, assign vel command of 0 to "reset" it.
    g_vel_cmd.data = 0;
    action_result_obj_.goal_success = 1;
    simple_server_obj_.setSucceeded(action_result_obj_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sin_command"); //name this node
    ros::NodeHandle nh; // node handle

    // Publishing velocity commands
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("vel_cmd", 1);
    g_pub = &pub;

    // Creating the object is all you need to do...
    MyActionServer my_action_server_obj_;

    // Continually update vel_cmd, spinning once to potentially update from callback action
    while(ros::ok())
    {
        // Now publish
        g_pub->publish(g_vel_cmd);
        // Log info for user to cout
        // ROS_INFO("Velocity Command Sent out is = %f", g_vel_cmd.data);
        ros::spinOnce();
    }
    ros::spin();
    return 0; // Should never get here, unless roscore dies!
}
