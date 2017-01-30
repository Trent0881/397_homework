# Comments on ur10_reach:

# This is the file that produces the data sets for this assignment. Replace z1 on line 84 with z1 - z6 for whichever height you want to test (and then recompile). These heights are in the order given in the assignment, not ascending or descending height order.

# ur_fk_ik
Library for FK/IK for UR10 robot, and an example "main" to illustrate use of the library.

## Example usage
Start up UR10 in Gazebo with:
`roslaunch ur_gazebo ur10.launch`
Run example program using FK/IK library with:
`rosrun ur_fk_ik ur10_fk_ik_test_main`
Move the robot around with example client:
`rosrun ur_traj_client ur_traj_client_pre_pose`

    
