#include <ros/ros.h>

#include "indooruav_waypoint/waypoint_tracker.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_tracker_node");
    indooruav_waypoint::WaypointTracker waypoint_tracker;
    ros::spin();
    return 0;
}
