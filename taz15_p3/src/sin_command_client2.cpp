// SIN COMMANDER CLIENT NODE - TRENT ZIEMER 9/27/2016

#include<ros/ros.h>
#include<std_msgs/Float64.h>

#include <actionlib/client/simple_action_client.h>
#include<taz15_p3/commandActionAction.h>

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
            const taz15_p3::commandActionResultConstPtr& result)
{
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    //int diff = result->output - result->goal_stamp;
    //ROS_INFO("got result output = %d; goal_stamp = %d; diff = %d",result->output,result->goal_stamp,diff);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_action_client_node"); // name this node

    double des_amplitude;
    double des_frequency;
    double des_cycles;

    std::cout << "Please type an amplitude, a frequency, and the number of cycles to print for a sine wave:" << std::endl;
    std::cin >> des_amplitude >> des_frequency >> des_cycles;

    // here is a "goal" object compatible with the server, as defined in example_action_server/action
    taz15_p3::commandActionGoal goal;

    // use the name of our server, which is: example_action (named in example_action_server.cpp)
    // the "true" argument says that we want our new client to run as a separate thread (a good idea)
    actionlib::SimpleActionClient<taz15_p3::commandActionAction> action_client("action_server_name", true);

    goal.amplitude = des_amplitude;
    goal.frequency = des_frequency;
    goal.cycles = des_cycles;

    action_client.sendGoal(goal, &doneCb);

    // attempt to connect to the server
    bool server_exists = action_client.waitForServer(ros::Duration(30));
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists)
    {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    // stuff a goal message:
    action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
    //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
    bool finished_before_timeout = action_client.waitForResult();

    if (!finished_before_timeout)
    {
        ROS_WARN("giving up waiting on result for goal number.........");
        return 0;
    }
    else
    {
        //if here, then server returned a result to us
    }
    return 0;
}
