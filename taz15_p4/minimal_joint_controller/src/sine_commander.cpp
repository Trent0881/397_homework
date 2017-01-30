
//NODE TO SEND OUT SINUSOIDAL POS COMMANDS BLINDLY FOR HW4 EECS 397 ROS - TRENT ZIEMER 10/3/2016
// BASED OFF EXAMPLE SINE COMMANDER/MINIMAL PUBLISHER IN CLASS CODE

#include <ros/ros.h>
#include <std_msgs/Float64.h>
// Include math for sine function computations
#include <math.h>
int main(int argc, char **argv) {
    ros::init(argc, argv, "two_joint_sin_command"); // name of this node will be "minimal_publisher2"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher j1_pos_cmd = n.advertise<std_msgs::Float64>("pos_cmd", 1);
    ros::Publisher j2_pos_cmd = n.advertise<std_msgs::Float64>("pos_cmd2", 1);
    //"topic1" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups

    std_msgs::Float64 input_float; //create a variable of type "Float64",
    std_msgs::Float64 pos_cmd; //create a variable of type "Float64",
    std_msgs::Float64 pos_cmd2; //create a variable of type "Float64",

   ros::Rate naptime(100); //create a ros object from the ros “Rate” class;
   //set the sleep timer for 1Hz repetition rate (arg is in units of Hz)

    input_float.data = 0.0;

    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok())
    {
        input_float.data = input_float.data + 0.0001; //increment by 0.0001 each iteration

        pos_cmd.data = 1*sin(5*input_float.data);
        pos_cmd2.data = 1*sin(20*input_float.data);


        j1_pos_cmd.publish(pos_cmd); // publish the value--of type Float64--
        j2_pos_cmd.publish(pos_cmd2);

        naptime.sleep();
    }
}

