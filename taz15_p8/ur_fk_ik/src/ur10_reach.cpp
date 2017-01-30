// My version of baxter_reachability but for the UR10 robot.
// Outputs to a file all the locations that the UR10 robot can reach
//    at z-levels pertinent to the NIST competition scenario.

#include <ur_fk_ik/ur_kin.h>
//#include <sensor_msgs/JointState.h>
//#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <fstream>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Transform.h>

vector<int> g_arm_joint_indices;
vector<string> g_ur_jnt_names;
Eigen::VectorXd g_q_vec;
#define VECTOR_DIM 6

using namespace std;

typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;

int main(int argc, char **argv) {
    ros::init(argc, argv, "ur10_reachability");
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    Vectorq6x1 q_in;
    q_in<<0,0,0,0,0,0;

    // UR-10 Robot forward kinematics solver object
    UR10FwdSolver ur10_fs;
    // UR-10 Robot reverse kinematics solver object
    UR10IkSolver ur10_is;

    b_des<<0,0,-1; //tool flange pointing down
    n_des<<0,0,1; //x-axis pointing forward...arbitrary
    t_des = b_des.cross(n_des); //consistent right-hand frame

    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;


    //std::vector<Vectorq7x1> q_solns;
    std::vector<Eigen::VectorXd> q_solns;
    Vectorq6x1 qsoln;

    Eigen::Affine3d a_tool_des; // expressed in DH frame
    a_tool_des.linear() = R_des;
    //a_tool_des.translation() << x_des,0,0;
    double x_des,y_des,z_des;
    double x_min = 0.4;
    double x_max = 1.5;
    double y_min = -1.5;
    double y_max = 1.0;

    // Possible relevant z-heights to choose from
    double z1 = 0.0;
    double z2 = 0.724275;
    double z3 = 0.950316;
    double z4 = 0.903960;
    double z5 = 0.750201;
    double z6 = 1.099893;

    double dx = 0.05;
    double dy = 0.05;

    Eigen::Vector3d p_des;
    int nsolns;
    std::vector<Eigen::Vector3d> reachable_points;

    for (double x_des = x_min;x_des<x_max;x_des+=dx) {
        for (double y_des = y_min; y_des<y_max; y_des+=dy) {
            // Set x and y
            p_des[0] = x_des;
            p_des[1] = y_des;

            p_des[2] = z1; // Set this to whichever height you want...!
            a_tool_des.translation() = p_des;
            nsolns = ur10_is.ik_solve(a_tool_des, q_solns);
            if (nsolns>0) {
                ROS_INFO("Solution found at x,y = %f, %f",p_des[0],p_des[1]);
                reachable_points.push_back(p_des);
            }
        }
    }

    // Record points as a csv file named "reachable_x_y"
    ROS_INFO("Saving the results...");
    nsolns = reachable_points.size();
    ofstream outfile;
    outfile.open("reachable_x_y");

    for (int i=0;i<nsolns;i++) {
        p_des = reachable_points[i];
        outfile << p_des[0] << ", " << p_des[1] << endl;
    }
    outfile.close();
    ROS_INFO("Done.");

    return 0;
}
