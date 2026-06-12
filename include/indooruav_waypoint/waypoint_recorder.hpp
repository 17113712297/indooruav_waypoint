// 订阅实时里程计，缓存最新一帧；
// 通过 std_srvs/Trigger 服务接收人工触发，把当前帧的 (x, y, z, yaw) 追加到内存列表；
// 收到 save 触发时，将内存列表整文件覆盖写入 YAML（yaw 以度为单位写入）。
//
// 录制的坐标系应与 waypoint_tracker 订阅的实时里程计话题一致，
// 否则录到的航点跟踪时会到不了正确位置。

#ifndef INDOORUAV_WAYPOINT_WAYPOINT_RECORDER_HPP_
#define INDOORUAV_WAYPOINT_WAYPOINT_RECORDER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace indooruav_waypoint {

class WaypointRecorder {
public:
    WaypointRecorder();
    ~WaypointRecorder();

    WaypointRecorder(const WaypointRecorder&) = delete;
    WaypointRecorder& operator=(const WaypointRecorder&) = delete;

private:
    struct Waypoint {
        double x_m = 0.0;
        double y_m = 0.0;
        double z_m = 0.0;
        double yaw_deg = 0.0;  // 文件以度存储，这里直接保存度
        bool stop = false;     // true=停留点, false=穿越点
    };

    struct Config {
        std::string mode;  // "direct" or "path_relay"
        std::string current_odometry_topic;
        std::string global_path_topic;
        std::string odometry_path_topic;
        std::string record_service_name;
        std::string save_service_name;
        std::string clear_service_name;
        std::string auto_record_start_service_name;
        std::string auto_record_stop_service_name;
        std::string waypoints_package_name;
        std::string waypoints_file_path;
        bool load_existing = false;
        double auto_record_interval_s = 3.0;
    };

    void LoadParameters(ros::NodeHandle& pnh);
    void SetupRosInterfaces();

    bool ResolveWaypointFilePath(std::string* resolved_path) const;
    bool LoadExistingWaypoints();
    bool SaveWaypointsToFile(std::string* error_message);

    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void GlobalPathCallback(const nav_msgs::Path::ConstPtr& msg);

    bool RecordServiceCallback(std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res);
    bool SaveServiceCallback(std_srvs::Trigger::Request& req,
                             std_srvs::Trigger::Response& res);
    bool ClearServiceCallback(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res);
    bool AutoRecordStartServiceCallback(std_srvs::Trigger::Request& req,
                                        std_srvs::Trigger::Response& res);
    bool AutoRecordStopServiceCallback(std_srvs::Trigger::Request& req,
                                       std_srvs::Trigger::Response& res);
    void AutoRecordTimerCallback(const ros::TimerEvent& event);

    static double GetYawFromQuaternion(const geometry_msgs::Quaternion& q);
    static double RadToDeg(double rad);
    static bool EnsureParentDirectory(const std::string& file_path,
                                      std::string* error_message);
    static std::string CurrentTimestampString();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odometry_subscriber_;
    ros::Subscriber global_path_subscriber_;
    ros::Publisher odometry_path_publisher_;
    ros::ServiceServer record_service_;
    ros::ServiceServer save_service_;
    ros::ServiceServer clear_service_;
    ros::ServiceServer auto_record_start_service_;
    ros::ServiceServer auto_record_stop_service_;
    ros::Timer auto_record_timer_;

    Config config_;
    std::mutex mutex_;
    std::vector<Waypoint> waypoints_;
    nav_msgs::Odometry latest_odometry_;
    bool has_odometry_ = false;
    bool auto_recording_active_ = false;
};

}  // namespace indooruav_waypoint

#endif  // INDOORUAV_WAYPOINT_WAYPOINT_RECORDER_HPP_
