// moving persons detector using lidar data
// written by O. Aycard

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "std_msgs/Bool.h"

//used for clustering
#define cluster_threshold 0.2 //threshold for clustering

//used for detection of motion
#define detection_threshold 0.2 //threshold for motion detection
#define dynamic_threshold 75 //to decide if a cluster is static or dynamic

//used for detection of moving legs
#define leg_size_min 0.10
#define leg_size_max 0.25

//used for detection of fmoving persons
#define legs_distance_max 0.7

using namespace std;

class moving_persons_detector {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_robot_moving;

    ros::Publisher pub_moving_persons_detector;
    ros::Publisher pub_moving_persons_detector_marker;

    // to store, process and display laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000];
    geometry_msgs::Point current_scan[1000];

    //to perform detection of motion
    float background[1000];//to store the background
    bool dynamic[1000];//to store if the current is dynamic or not

    //to perform clustering
    int nb_cluster;// number of cluster
    int cluster[1000]; //to store for each hit, the cluster it belongs to: for instance, cluster[6] = 1 => hit 6 belongs to cluster 1
    float cluster_size[1000];// to store the size of each cluster
    geometry_msgs::Point cluster_middle[1000];// to store the middle of each cluster
    int cluster_dynamic[1000];// to store the percentage of the cluster that is dynamic
    int cluster_start[1000], cluster_end[1000];//to store the hit corresponding to the start/end of a cluster: for instance, cluster_start[1] = 10 => hit 10 is the start of cluster 1

    //to perform detection of legs and to store them
    int nb_legs_detected;
    geometry_msgs::Point legs_detected[1000];// to store the middle of each leg
    bool leg_dynamic[1000]; // to store if a leg is dynamic or static

    //to perform detection of moving person and store them
    int nb_persons_detected;
    geometry_msgs::Point persons_detected[1000];// to store the middle of each person
    bool person_dynamic[1000]; // to store if a person is dynamic or static

    // to know if a moving person has been detected or not
    bool moving_person_detected;

    //to store the goal to reach that we will be published
    geometry_msgs::Point goal_to_reach;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

    //to check if the robot is moving or not
    bool previous_robot_moving;
    bool current_robot_moving;

    bool init_laser;//to check if new data of laser is available or not
    bool init_robot;//to check if new data of robot_moving is available or not

    bool display_laser;
    bool display_robot;

public:

moving_persons_detector() {

    sub_scan = n.subscribe("scan", 1, &moving_persons_detector::scanCallback, this);
    sub_robot_moving = n.subscribe("robot_moving", 1, &moving_persons_detector::robot_movingCallback, this);

    pub_moving_persons_detector_marker = n.advertise<visualization_msgs::Marker>("moving_person_detector", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    pub_moving_persons_detector = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.

    previous_robot_moving = true;
    init_laser = false;
    init_robot = false;
    display_laser = false;
    display_robot = false;

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing of laser data and robot_moving
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( init_laser && init_robot ) {
        nb_pts = 0;

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");

        nb_pts = 0;
        //if the robot is not moving then we can perform moving persons detection
        if ( !current_robot_moving ) {
            // = false;
            ROS_INFO("robot is not moving");
                // if the robot was moving previously and now it is not moving now then we store the background
            if ( previous_robot_moving && !current_robot_moving )
                store_background();

            //we search for moving persons in 4 steps
            detect_motion();//to classify each hit of the laser as dynamic or not
            perform_clustering();//to perform clustering
            detect_legs();
            detect_persons();

            //to publish the goal_to_reach
            if ( nb_persons_detected )
                pub_moving_persons_detector.publish(goal_to_reach);
        }
        else{
            ROS_INFO("robot is moving");
            //moving_person_detected = true;
            //we search for moving persons in 4 steps
            detect_motion();//to classify each hit of the laser as dynamic or not
            perform_clustering();//to perform clustering
            detect_legs();
            detect_persons();
        }
        previous_robot_moving = current_robot_moving;
        //graphical display of the results
        populateMarkerTopic();

    }
    if ( !display_laser && !init_laser ) {
        ROS_INFO("wait for laser data");
        display_laser = true;
    }
    if ( display_laser && init_laser )  {
        ROS_INFO("laser data are ok");
        display_laser = false;
    }
    if ( !display_robot && !init_robot ) {
        ROS_INFO("wait for robot_moving_node");
        display_robot = true;
    }
    if ( display_robot && init_robot ) {
        ROS_INFO("robot_moving_node is ok");
        display_robot = false;
    }

}// update

// DETECTION OF MOTION
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background() {
// store all the hits of the laser in the background table

    ROS_INFO("storing background");

    for (int loop=0; loop<nb_beams; loop++)
        background[loop] = range[loop];

    ROS_INFO("background stored");

}//init_background

void detect_motion() {

    ROS_INFO("detecting motion");

    for (int loop=0; loop<nb_beams; loop++) {
        if (abs(background[loop] - range[loop]) > detection_threshold) {
            dynamic[loop] = 1;
        } else {
            dynamic[loop] = 0;
        }
    }

    ROS_INFO("motion detected");

}//detect_motion

// CLUSTERING
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit and the current one is lower than "cluster_threshold"
//then the current hit belongs to the current cluster
//else we start a new cluster with the current hit and end the current cluster

    ROS_INFO("performing clustering");

    nb_cluster = 0;//to count the number of cluster

    //initialization of the first cluster
    cluster_start[0] = 0;// the first hit is the start of the first cluster
    cluster[0] = 0;// the first hit belongs to the first cluster
    int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

    //graphical display of the start of the current cluster in green
    // display[nb_pts].x = current_scan[cluster_start[nb_cluster]].x;
    // display[nb_pts].y = current_scan[cluster_start[nb_cluster]].y;
    // display[nb_pts].z = current_scan[cluster_start[nb_cluster]].z;

    // colors[nb_pts].r = 0;
    // colors[nb_pts].g = 1;
    // colors[nb_pts].b = 0;
    // colors[nb_pts].a = 1.0;
    //nb_pts++;

    for (int loop=1; loop<nb_beams; loop++) {
        if (abs(range[loop-1] - range[loop]) < cluster_threshold) {
            //the current hit belongs to the current cluster
            cluster[loop] = nb_cluster;
            if (dynamic[loop]){
                nb_dynamic++;
            }
        } else { //the current hit doesnt belong to the same hit
            cluster_end[nb_cluster] = loop - 1;
            cluster_size[nb_cluster] = sqrt(
                pow(current_scan[cluster_start[nb_cluster]].x - current_scan[cluster_end[nb_cluster]].x, 2) +
                pow(current_scan[cluster_start[nb_cluster]].y - current_scan[cluster_end[nb_cluster]].y, 2) +
                pow(current_scan[cluster_start[nb_cluster]].z - current_scan[cluster_end[nb_cluster]].z, 2));
            cluster_dynamic[nb_cluster] = 100 * nb_dynamic / (cluster_end[nb_cluster] - cluster_start[nb_cluster] + 1);
            for (int i = cluster_start[nb_cluster]; i < cluster_end[nb_cluster]; i++) {
                cluster_middle[nb_cluster].x += current_scan[i].x;
                cluster_middle[nb_cluster].y += current_scan[i].y;
                cluster_middle[nb_cluster].z += current_scan[i].z;
            }
            cluster_middle[nb_cluster].x /= (cluster_end[nb_cluster] - cluster_start[nb_cluster] + 1);
            cluster_middle[nb_cluster].y /= (cluster_end[nb_cluster] - cluster_start[nb_cluster] + 1);
            cluster_middle[nb_cluster].z /= (cluster_end[nb_cluster] - cluster_start[nb_cluster] + 1);

            //graphical display of the end of the current cluster in red
            // display[nb_pts].x = current_scan[cluster_end[nb_cluster]].x;
            // display[nb_pts].y = current_scan[cluster_end[nb_cluster]].y;
            // display[nb_pts].z = current_scan[cluster_end[nb_cluster]].z;

            // colors[nb_pts].r = 1;
            // colors[nb_pts].g = 0;
            // colors[nb_pts].b = 0;
            // colors[nb_pts].a = 1.0;
            //nb_pts++;
            int isLeg = 0;
            if (cluster_size[nb_cluster] > leg_size_min && cluster_size[nb_cluster] < leg_size_max){isLeg = 1;}
            //textual display
            ROS_INFO("cluster[%i]: [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, isLeg: %i, middle: (%f, %f)", nb_cluster, cluster_start[nb_cluster], current_scan[cluster_start[nb_cluster]].x, current_scan[cluster_start[nb_cluster]].y, cluster_end[nb_cluster], current_scan[cluster_end[nb_cluster]].x, current_scan[cluster_end[nb_cluster]].y, cluster_size[nb_cluster], cluster_dynamic[nb_cluster], isLeg, cluster_middle[nb_cluster].x, cluster_middle[nb_cluster].y);

            // we start a new cluster with the current hit
            nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic
            nb_cluster++;
            cluster_start[nb_cluster] = loop;
            cluster[loop] = nb_cluster;
            if ( dynamic[loop] )
                nb_dynamic++;

            //graphical display of the start of the current cluster in green
            // display[nb_pts].x = current_scan[cluster_start[nb_cluster]].x;
            // display[nb_pts].y = current_scan[cluster_start[nb_cluster]].y;
            // display[nb_pts].z = current_scan[cluster_start[nb_cluster]].z;

            // colors[nb_pts].r = 0;
            // colors[nb_pts].g = 1;
            // colors[nb_pts].b = 0;
            // colors[nb_pts].a = 1.0;
            //nb_pts++;
        }
    }

    nb_cluster++;

    ROS_INFO("clustering performed");

}//perfor_clustering

// DETECTION OF MOVING PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void detect_legs() {
// a moving leg is a cluster:
// - with a size higher than "leg_size_min";
// - with a size lower than "leg_size_max;
// - more than "dynamic_threshold"% of its hits are dynamic (see, cluster_dynamic table)

    ROS_INFO("detecting moving legs");
    nb_legs_detected = 0;

    for (int loop=0; loop<nb_cluster; loop++) {
        ROS_INFO("SIZE: [%f], DYNAMIC: [%i]", cluster_size[loop], cluster_dynamic[loop]);
        if (cluster_size[loop] > leg_size_min &&
            cluster_size[loop] < leg_size_max) {
                // robair stops - stop to detect all the persons
                if (!moving_person_detected){
                    //detect a moving person
                    if (cluster_dynamic[loop] > dynamic_threshold){
                        // we update the legs_detected table to store the middle of the moving leg
                        legs_detected[nb_legs_detected] = cluster_middle[loop];

                        //we mark current leg as dynamic
                        leg_dynamic[nb_legs_detected] = 1;

                        //textual display
                        ROS_INFO("moving leg detected[%i]: cluster[%i]", nb_legs_detected, loop);
                        nb_legs_detected++;

                        //graphical display
                        for(int loop2=cluster_start[loop]; loop2<=cluster_end[loop]; loop2++) {
                            // moving legs are white
                            display[nb_pts].x = current_scan[loop2].x;
                            display[nb_pts].y = current_scan[loop2].y;
                            display[nb_pts].z = current_scan[loop2].z;

                            colors[nb_pts].r = 1;
                            colors[nb_pts].g = 1;
                            colors[nb_pts].b = 1;
                            colors[nb_pts].a = 1.0;

                            nb_pts++;
                        }
                    }
                }
                //robair is moving
                else{
                    // we update the legs_detected table to store the middle of all legs
                    legs_detected[nb_legs_detected] = cluster_middle[loop];

                    if (cluster_dynamic[loop] > dynamic_threshold){
                            //we mark current leg as dynamic
                        leg_dynamic[nb_legs_detected] = 1;
                    }
                    else {
                        leg_dynamic[nb_legs_detected] = 0;
                    }

                    //textual display
                    ROS_INFO("leg detected[%i]: cluster[%i]", nb_legs_detected, loop);
                    nb_legs_detected++;
                    //graphical display
                    for(int loop2=cluster_start[loop]; loop2<=cluster_end[loop]; loop2++) {
                        
                        display[nb_pts].x = current_scan[loop2].x;
                        display[nb_pts].y = current_scan[loop2].y;
                        display[nb_pts].z = current_scan[loop2].z;
                        //moving legs are white
                        colors[nb_pts].r = 1;
                        colors[nb_pts].g = 1;
                        colors[nb_pts].b = 1;
                        colors[nb_pts].a = 1.0;

                        nb_pts++;
                    }
                }
            }
    }

    if ( nb_legs_detected )
        ROS_INFO("%d legs have been detected.\n", nb_legs_detected);

    ROS_INFO("legs are detected");

}//detect_legs

void detect_persons() {
// a moving person has two moving legs located at less than "legs_distance_max" one from the other

    ROS_INFO("detecting persons");
    nb_persons_detected = 0;

    float loop_leg_dist;

    for (int loop_leg1=0; loop_leg1<nb_legs_detected; loop_leg1++) {//loop over all the legs
        for (int loop_leg2=loop_leg1+1; loop_leg2<nb_legs_detected; loop_leg2++) {//loop over all the legs
            loop_leg_dist = sqrt(
            pow(legs_detected[loop_leg1].x - legs_detected[loop_leg2].x, 2) +
            pow(legs_detected[loop_leg1].y - legs_detected[loop_leg2].y, 2) +
            pow(legs_detected[loop_leg1].z - legs_detected[loop_leg2].z, 2));

            if (loop_leg_dist > legs_distance_max)
                continue;

            if (leg_dynamic[loop_leg1] == 1 && leg_dynamic[loop_leg2] == 1){
                // we update the persons_detected table to store the middle of the moving person
                persons_detected[nb_persons_detected].x = (legs_detected[loop_leg1].x + legs_detected[loop_leg2].x) / 2;
                persons_detected[nb_persons_detected].y = (legs_detected[loop_leg1].y + legs_detected[loop_leg2].y) / 2;
                persons_detected[nb_persons_detected].z = (legs_detected[loop_leg1].z + legs_detected[loop_leg2].z) / 2;
                ROS_INFO("moving person detected[%i]: leg[%i]+leg[%i] -> (%f, %f)", nb_persons_detected, loop_leg1, loop_leg2, persons_detected[nb_persons_detected].x, persons_detected[nb_persons_detected].y);

                // the moving persons are green
                // display[nb_pts].x = persons_detected[nb_persons_detected].x;
                // display[nb_pts].y = persons_detected[nb_persons_detected].y;
                // display[nb_pts].z = persons_detected[nb_persons_detected].z;

                // colors[nb_pts].r = 0;
                // colors[nb_pts].g = 1;
                // colors[nb_pts].b = 0;
                // colors[nb_pts].a = 1.0;

                // nb_pts++;

                person_dynamic[nb_persons_detected] = 1;

                goal_to_reach.x = persons_detected[nb_persons_detected].x;
                goal_to_reach.y = persons_detected[nb_persons_detected].y;

                // pub_moving_persons_detector.publish(goal_to_reach);
                nb_persons_detected++;
                break;
            }
            else if (leg_dynamic[loop_leg1] == 0 && leg_dynamic[loop_leg2] == 0){
                // we update the persons_detected table to store the middle of the moving person
                persons_detected[nb_persons_detected].x = (legs_detected[loop_leg1].x + legs_detected[loop_leg2].x) / 2;
                persons_detected[nb_persons_detected].y = (legs_detected[loop_leg1].y + legs_detected[loop_leg2].y) / 2;
                persons_detected[nb_persons_detected].z = (legs_detected[loop_leg1].z + legs_detected[loop_leg2].z) / 2;
                ROS_INFO("person detected[%i]: leg[%i]+leg[%i] -> (%f, %f)", nb_persons_detected, loop_leg1, loop_leg2, persons_detected[nb_persons_detected].x, persons_detected[nb_persons_detected].y);

                // the moving persons are orange
                // display[nb_pts].x = persons_detected[nb_persons_detected].x;
                // display[nb_pts].y = persons_detected[nb_persons_detected].y;
                // display[nb_pts].z = persons_detected[nb_persons_detected].z;

                // colors[nb_pts].r = 1;
                // colors[nb_pts].g = 0;
                // colors[nb_pts].b = 0;
                // colors[nb_pts].a = 1.0;

                // nb_pts++;

                person_dynamic[nb_persons_detected] = 0;

                //goal_to_reach.x = persons_detected[nb_persons_detected].x;
                //goal_to_reach.y = persons_detected[nb_persons_detected].y;

                // pub_moving_persons_detector.publish(goal_to_reach);
                nb_persons_detected++;
                break;
            }
        }
    }

    if ( nb_persons_detected )
        ROS_INFO("%d moving persons have been detected.\n", nb_persons_detected);

    ROS_INFO("moving persons detected");


}//detect_persons

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    init_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            range[loop] = scan->ranges[loop];
        else
            range[loop] = range_max;

        //transform the scan in cartesian framework
        current_scan[loop].x = range[loop] * cos(beam_angle);
        current_scan[loop].y = range[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }

}//scanCallback

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

    init_robot = true;
    //previous_robot_moving = current_robot_moving;
    current_robot_moving = state->data;

    moving_person_detected = current_robot_moving;

}//robot_movingCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

// Draw the field of view and other references
void populateMarkerReference() {

    visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "example";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x =  0.02 * cos(-2.356194);
    v.y =  0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  5.6 * cos(-2.356194);
    v.y =  5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i=0 ; i< 723; i++, beam_angle += 0.006136){
        v.x =  5.6 * cos(beam_angle);
        v.y =  5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x =  5.6 * cos(2.092350);
    v.y =  5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  0.02 * cos(2.092350);
    v.y =  0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    pub_moving_persons_detector_marker.publish(references);

}

void populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    //ROS_INFO("%i points to display", nb_pts);
    for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            //ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

    pub_moving_persons_detector_marker.publish(marker);
    populateMarkerReference();

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "moving_persons_detector");

    moving_persons_detector bsObject;

    ros::spin();

    return 0;
}
