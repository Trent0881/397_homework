// TRENT ZIEMER - Copied from WSN code 10/20/2016
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h> //ALWAYS need to include this

#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
#include <sensor_msgs/LaserScan.h>
using namespace std;

//these are globals
tf::TransformListener *g_listener_ptr; //a transform listener
XformUtils xformUtils; //instantiate an object of XformUtils
vector <Eigen::Vector3d> g_pt_vecs_wrt_lidar_frame; //will hold 3-D points in LIDAR frame
vector <Eigen::Vector3d> g_pt_vecs_wrt_world_frame; //will hold 3_D points in world frame

// Global record of all block scan points in the current sweep (gets reset each time we calculate a full block)
vector<Eigen::Vector3d> g_block_points;

// State machine booleans that latch the scans to the SECOND block we find.
bool block_detected;
bool block_finished;
bool wait_for_block_again;
bool block_detected_again;
bool block_finished_again;

// Function to return the mean value from a vector of points
double find_avg(vector<double> list_of_pts)
{
    double avg = 0;
    for (int i = 0; i < list_of_pts.size(); i++)
    {
        avg = avg + list_of_pts[i];
    }
    avg = avg/((float)list_of_pts.size());
    return avg;
}

// Function to return the smallest value from a vector of points
double find_min(vector<double> list_of_pts)
{
    double minimum_value = 99999;
    for (int i = 0; i < list_of_pts.size(); i++)
    {
        if (list_of_pts[i] < minimum_value)
        {
            minimum_value = list_of_pts[i];
        }
    }
    return minimum_value;
}

// Function to return the largest value from a vector of points
double find_max(vector<double> list_of_pts)
{
    double maximum_value = -99999;
    for (int i = 0; i < list_of_pts.size(); i++)
    {
        if (list_of_pts[i] > maximum_value)
        {
            maximum_value = list_of_pts[i];
        }
    }
    return maximum_value;
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    //if here, then a new LIDAR scan has been received
    // get the transform from LIDAR frame to world frame
    tf::StampedTransform stfLidar2World;
    //specialized for lidar_wobbler; more generally, use scan_in->header.frame_id
    g_listener_ptr->lookupTransform("world", "lidar_link", ros::Time(0), stfLidar2World);
    //extract transform from transformStamped:
    tf::Transform tf = xformUtils.get_tf_from_stamped_tf(stfLidar2World);
    //stfLidar2World is only the pose of the LIDAR at the LAST ping...
    //better would be to consider separate transforms for each ping
    //using the above transform for all points is adequate approx if LIDAR is wobbling slowly enough
    Eigen::Affine3d affine_tf,affine_tf_inv; //can use an Eigen type "affine" object for transformations
    //convert transform to Eigen::Affine3d
    affine_tf = xformUtils.transformTFToAffine3d(tf); //can use this to transform points to world frame
    affine_tf_inv = affine_tf.inverse();
    vector <float> ranges = scan_in->ranges; //extract all the radius values from scan
    int npts = ranges.size(); //see how many pings there are in the scan; expect 181 for wobbler model
    g_pt_vecs_wrt_lidar_frame.clear();
    g_pt_vecs_wrt_world_frame.clear();

    //ROS_INFO("received %d ranges: ", npts);
    double start_ang = scan_in->angle_min; //get start and end angles from scan message
    double end_ang = scan_in->angle_max;   //should be -90 deg to +90 deg
    double d_ang = (end_ang - start_ang) / (npts - 1); //samples are at this angular increment
    //ROS_INFO("d_ang = %f", d_ang);
    Eigen::Vector3d vec; //var to hold one point at a time
    vec[2] = 0.0; //all pings in the LIDAR frame are in x-y plane, so z-component is 0

    double ang;
    for (int i = 0; i < npts; i++) {
        if (ranges[i] < 5.0) { //only transform points within 5m
            //if range is too long, LIDAR is nearly parallel to the ground plane, so skip this ping
            ang = start_ang + i*d_ang; //polar angle of this ping
            vec[0] = ranges[i] * cos(ang); //convert polar coords to Cartesian coords
            vec[1] = ranges[i] * sin(ang);
            g_pt_vecs_wrt_lidar_frame.push_back(vec); //save the valid 3d points
        }
    }
    int npts3d = g_pt_vecs_wrt_lidar_frame.size(); //this many points got converted
    //ROS_INFO("computed %d 3-D pts w/rt LIDAR frame", npts3d);
    g_pt_vecs_wrt_world_frame.resize(npts3d);

    //transform the points to world frame:
    //do this one point at a time; alternatively, could have listed all points
    //as column vectors in a single matrix, then do a single multiply to convert the
    //entire matrix of points to the world frame
    for (int i = 0; i < npts3d; i++) {
        g_pt_vecs_wrt_world_frame[i] = affine_tf * g_pt_vecs_wrt_lidar_frame[i];
    }

    //the points in g_pt_vecs_wrt_world_frame are now in Cartesian coordinates
    // points in this frame are easier to interpret

    //can now analyze these points to interpret shape of objects on the ground plane
    //but for this example, simply display the z values w/rt world frame:


    // <---------THIS IS WHERE TRENTS CODE STARTS AND OLD CODE ENDS. ---------------------->


    // An array of point slices in the Z direction.
    vector<vector<Eigen::Vector3d> > array_of_point_slices;

    // A single slice of the array of points slices. This represents a single scan of the
    //  LIDAR scanner between two Z bounds (upper and lower).
    vector<Eigen::Vector3d> point_slice;

    // Z slice parameters
    double delta_z = 0.1;
    double upper_z;

    // Divide the scan into slices (or statifications) in the Z direction.
    for(upper_z = delta_z; upper_z < 1; upper_z = upper_z + delta_z)
    {
        // Iterate over all points to put it into one of the slices.
        for (int i = 0; i < npts3d; i++)
        {
            // Load the vector.
            vec = g_pt_vecs_wrt_world_frame[i]; //consider the i'th point

            // Add the point vector to the slice if its between the bounds.
            if(vec[2] < upper_z && vec[2] > upper_z-delta_z)
            {
                point_slice.push_back(vec);
            }
        }
        // BUT, don't add the slice if it has too few points in it.
        //      We don't want noise getting in the way.
        if (point_slice.size() > 5)
        {
            array_of_point_slices.push_back(point_slice);
        }
        point_slice.clear();
    }

    // If the number of dense (has more than a few points) slices is more than one, then
    //   we know that the first one (i = 0) is the floor plane, while the second one (i = 1)
    //   is the scan of the block.
    //  Thus, we want to load this line scan of points into the full list of
    //    all points that we know are aprt of the block scan (global across these scan callbacks).
    if (array_of_point_slices.size() > 1)
    {
        for (int n = 0; n < array_of_point_slices[1].size(); n++)
        {
            g_block_points.push_back(array_of_point_slices[1][n]);
        }
    }

    // The following series of "if" statements are the formation of
    //     a rudimentary state machine, that will latch onto the SECOND full block we see
    //   (because the first one seen might/will be a partial scan only)
    // Wait around until we see the start of the block.
    if (array_of_point_slices.size() > 1)
    {
        block_detected = true;
        //cout << "Block detected" << endl;
    }

    // Wait until we see the end of the block.
    if (block_detected == true)
    {
        if(array_of_point_slices.size() == 1)
        {
            block_finished = true;
            block_detected = false;
            //cout << "Block finished" << endl;
        }
    }

    // Once the first partial block started to scan, wait until we finish scanning it.
    if (block_finished == true)
    {
        wait_for_block_again = true;
        block_finished = false;
        //cout << "Block finished" << endl;
    }

    // Now that we are done with the first (propably partial) block scan, we
    //     are going to wait until we see the block again to start scanning.
    if (wait_for_block_again == true)
    {
        if (array_of_point_slices.size() > 1)
        {
            block_detected_again = true;
            wait_for_block_again = false;
            //cout << "Block detected again" << endl;
        }
    }

    // Once we have hit the block again, wait until we STOP seeing the block again to know
    //   that we are done with the full scan.
    if (block_detected_again == true)
    {
        if (array_of_point_slices.size() == 1)
        {
            block_detected_again = false;
            block_finished_again = true;
            //cout << "Block finished again" << endl;
        }
    }

    // If we finally have a full block scan done, so proceed with computing things
    //    about the full scan.
    if (block_finished_again == true)
    {
        block_finished_again = false;

        // Declare various statistic variables.
        double com_x, com_y, com_z;
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;
        double len_x, len_y, len_z;
        double avg_x, avg_y, avg_z;
        double cor1_x, cor1_y, cor1_z;
        double cor2_x, cor2_y, cor2_z;
        double cor3_x, cor3_y, cor3_z;
        double cor4_x, cor4_y, cor4_z;
        vector<double> point_vector;

        // Compute statistics for the X coordinates of the block points.
        for (int i = 0; i < g_block_points.size(); i++)
        {
            point_vector.push_back(g_block_points[i][0]);
        }
        min_x = find_min(point_vector);
        max_x = find_max(point_vector);
        avg_x = find_avg(point_vector);
        point_vector.clear();

        // Compute statistics for the Y coordinates of the block points.
        for (int i = 0; i < g_block_points.size(); i++)
        {
            point_vector.push_back(g_block_points[i][1]);
        }
        min_y = find_min(point_vector);
        max_y = find_max(point_vector);
        avg_y = find_avg(point_vector);
        point_vector.clear();

        // Compute statistics for the Z coordinates of the block points.
        for (int i = 0; i < g_block_points.size(); i++)
        {
            point_vector.push_back(g_block_points[i][2]);
        }
        min_z = find_min(point_vector);
        max_z = find_max(point_vector);
        avg_z = find_avg(point_vector);
        point_vector.clear();

        // Compute the lengths of the scan area. Only the x and y are actually for the block itself,
        //    the Z is just the range of the z points collected.
        len_x = max_x - min_x;
        len_y = max_y - min_y;
        len_z = max_z - min_z;

        // Find the "center of mass" of the block by finding the midway points between the extrema points of the scan.
        com_x = (max_x + min_x)/2;
        com_y = (max_y + min_y)/2;
        com_z = (max_z + min_z)/2;

        // Compute the locations of the four corners of the top of the block
        cor1_x = min_x;
        cor1_y = min_y;
        cor1_z = avg_z;

        cor2_x = min_x;
        cor2_y = max_y;
        cor2_z = avg_z;

        cor3_x = max_x;
        cor3_y = min_y;
        cor3_z = avg_z;

        cor4_x = max_x;
        cor4_y = max_y;
        cor4_z = avg_z;

        // Print block info for user
        cout << "< x, y, z>" << endl;
        cout << "Number of block scanned points total:" << g_block_points.size() << "." << endl;
        cout << "Mins are: " << min_x << " " << min_y << " " << min_z << "." << endl;
        cout << "Maxs are: " << max_x << " " << max_y << " " << max_z << "." << endl;
        cout << "Lens are: " << len_x << ", " << len_y << ", " << len_z << "." << endl;
        cout << "Halfway points are: " << com_x << ", " << com_y << ", " << com_z << "." << endl;
        cout << "Bulk averages are: " << avg_x << ", " << avg_y << ", " << avg_z << "." << endl;
        cout << "Corner 1:" << cor1_x << ", " << cor1_y << ", " << cor1_z << "." << endl;
        cout << "Corner 2:" << cor2_x << ", " << cor2_y << ", " << cor2_z << "." << endl;
        cout << "Corner 3:" << cor3_x << ", " << cor3_y << ", " << cor3_z << "." << endl;
        cout << "Corner 4:" << cor4_x << ", " << cor4_y << ", " << cor4_z << "." << endl;
        cout << endl;

        // Done with processing the block, so clear the array.
        g_block_points.clear();
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_wobbler_transformer"); //node name
    ros::NodeHandle nh;

    // SOME OF MY CODE HERE.
    // Declare block state machine latching booleans as all false (will be set true in sequence then reset).
    block_detected = false;
    block_finished = false;
    wait_for_block_again = false;
    block_detected_again = false;
    block_finished_again = false;

    g_listener_ptr = new tf::TransformListener;
    tf::StampedTransform stfLidar2World;
    bool tferr = true;
    ROS_INFO("trying to get tf of lidar_link w/rt world: ");
    //topic /scan has lidar data in frame_id: lidar_link
    while (tferr) {
        tferr = false;
        try {
            g_listener_ptr->lookupTransform("world", "lidar_link", ros::Time(0), stfLidar2World);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    //ROS_INFO("transform received; ready to process lidar scans");
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, scanCallback);
    ros::spin(); //let the callback do all the work

    return 0;
}
