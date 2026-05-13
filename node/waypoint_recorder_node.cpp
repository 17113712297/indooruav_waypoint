#include <ros/ros.h>

#include "indooruav_waypoint/waypoint_recorder.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_recorder_node");
    indooruav_waypoint::WaypointRecorder waypoint_recorder;
    ros::spin();
    return 0;
}
