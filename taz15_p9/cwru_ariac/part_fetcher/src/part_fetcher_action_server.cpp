// Created by TZ Dec 2 2016
// Part Fetcher package
// Action server for part fetcher action client

// use with object_grabber action server called "objectGrabberActionServer"
// in file object_grabber_as.cpp

//client gets gripper ID from param server
// gets grasp strategy options from manip_properties(gripper_ID,object_ID)
// two primary fncs:
//   **   object_grab(object_id, object_pickup_pose, grasp_strategy, approach_strategy, depart_strategy)
//  **   object_dropoff(object_id, object_destination_pose, grasp_strategy, dropoff_strategy, depart_strategy)
//      have default args for grasp_strategy, depart_strategy, ...
//       default is grab from above, approach and depart from above
//      grasp strategy implies a grasp transform--to be used by action service for planning paths
//       all coords expressed as object frame w/rt named frame--which must have a 
//        kinematic path to system_ref_frame (e.g. simply use system_ref_frame)


#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <object_grabber/object_grabberAction.h>
#include<part_fetcher/PartFetcherAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include<generic_gripper_services/genericGripperInterface.h>

using namespace std;
XformUtils xformUtils; //type conversion utilities

geometry_msgs::PoseStamped g_object_pickup_poseStamped;
geometry_msgs::PoseStamped g_object_dropoff_poseStamped;
int g_object_ID;
bool g_received_order=false;

int g_object_grabber_return_code;

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    g_object_grabber_return_code = result->return_code;
    ROS_INFO("got result output = %d; ", g_object_grabber_return_code);
}

class PartFetcherActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<part_fetcher::PartFetcherAction> as_;
    
    // here are some message types to communicate with our client(s)
    part_fetcher::PartFetcherGoal goal_; // goal message, received from client
    part_fetcher::PartFetcherResult result_; // put results here, to be sent back to the client when done w/ goal
    part_fetcher::PartFetcherFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    PartFetcherActionServer(); //define the body of the constructor outside of class definition

    ~PartFetcherActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<part_fetcher::PartFetcherAction>::GoalConstPtr& goal);
};


PartFetcherActionServer::PartFetcherActionServer() :
   as_(nh_, "part_fetcher", boost::bind(&PartFetcherActionServer::executeCB, this, _1),false) 
{
    ROS_INFO("in constructor of PartFetcherActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

void PartFetcherActionServer::executeCB(const actionlib::SimpleActionServer<part_fetcher::PartFetcherAction>::GoalConstPtr& goal) {
    
     g_object_pickup_poseStamped = goal->object_frame;
     g_object_dropoff_poseStamped = goal->desired_frame;
     g_object_ID = goal->object_id;
     ROS_INFO("requested fetch of part_id = %d from location ",g_object_ID);
     xformUtils.printStampedPose(g_object_pickup_poseStamped);
     ROS_INFO("requested dropoff at pose: ");
     xformUtils.printStampedPose(g_object_dropoff_poseStamped);
     g_received_order=true; //set flag that new order has been received    
    
    result_.rtn_code=0; 
    as_.setSucceeded(result_); 


	// Added action client stuff


	// This should come from EPFC
    int object_id = ObjectIdCodes::TOY_BLOCK_ID;

    object_grabber::object_grabberGoal object_grabber_goal;
	// Instead if declaring new pose objects to hard code in "set_example_object_frames" function, declare them equal to received poses from callback inputs
    geometry_msgs::PoseStamped object_pickup_poseStamped = g_object_pickup_poseStamped;
    geometry_msgs::PoseStamped object_dropoff_poseStamped = g_object_dropoff_poseStamped;

    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("object_grabber_action_service", true);

    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); // if here, then we connected to the server; 


    bool finished_before_timeout;
    ROS_INFO("sending test code: ");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::TEST_CODE;
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result 1");
    }

    //move to waiting pose
    ROS_INFO("sending command to move to waiting pose");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE;
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result 2");
    }

    ROS_INFO("sending a grab-object command");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::GRAB_OBJECT; //specify the action to be performed 
    object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID; // specify the object to manipulate                
    object_grabber_goal.object_frame = object_pickup_poseStamped; //and the object's current pose
    object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
    object_grabber_goal.speed_factor = 1.0;
    ROS_INFO("sending goal to grab object: ");
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    ROS_INFO("waiting on result");
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result 3");
    }
 
    //move to waiting pose
    ROS_INFO("sending command to move to waiting pose");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE;
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result 4");
    }
    
    //could instead hand-tune some intermediate, joint-space poses:
    //    Eigen::VectorXd joint_angles;
    // resize to 6 joints and populate w/ magic numbers
    //rtn_val=arm_motion_commander.plan_jspace_path_current_to_qgoal(joint_angles);
    //rtn_val=arm_motion_commander.execute_planned_path(); 
   
    ROS_INFO("sending a dropoff-object command");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::DROPOFF_OBJECT; //specify the action to be performed 
    object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID; // specify the object to manipulate                
    object_grabber_goal.object_frame = object_dropoff_poseStamped; //and the object's current pose
    object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
    object_grabber_goal.speed_factor = 1.0;
    ROS_INFO("sending goal to dropoff object: ");
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    ROS_INFO("waiting on result");
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result 5");
    }

// end added action client stuff

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "part_fetcher_action_server");
    ros::NodeHandle nh;
    double dt_wait = 0.5; //check status this often
    double wait_time;
    const double MAX_WAIT_TIME = 30.0; // 30-second watchdog
    ros::Duration sleeper(dt_wait);

////////
    ROS_INFO("instantiating the part-fetcher action server: ");

    PartFetcherActionServer as_object; 
    ROS_INFO("going into spin");

    while (ros::ok()) {
        ros::spinOnce(); 
        ros::Duration(0.1).sleep();
        if (g_received_order) {
         ROS_WARN("main: received new order; should act on it!");
         g_received_order=false;
        }
    }
////////


}
