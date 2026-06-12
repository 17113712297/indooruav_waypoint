#include "indooruav_waypoint/waypoint_recorder.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/package.h>
#include <yaml-cpp/yaml.h>

namespace indooruav_waypoint {

namespace {

constexpr double kRadToDegConst = 180.0 / 3.14159265358979323846;

}  // namespace

// =============================================================================
// 构造与析构
// =============================================================================

WaypointRecorder::WaypointRecorder()
    : nh_(),
      pnh_("~") {
    LoadParameters(pnh_);
    SetupRosInterfaces();

    if (config_.load_existing) {
        if (LoadExistingWaypoints()) {
            ROS_INFO("[WaypointRecorder] Loaded %zu existing waypoints (resume mode).",
                     waypoints_.size());
        } else {
            ROS_INFO(
                "[WaypointRecorder] No existing waypoints loaded "
                "(starting empty).");
        }
    } else {
        ROS_INFO(
            "[WaypointRecorder] Starting from empty waypoint list "
            "(load_existing=false).");
    }

    if (config_.mode == "path_relay") {
        ROS_INFO_STREAM("[WaypointRecorder] mode=path_relay, subscribing path from: "
                        << config_.global_path_topic
                        << ", publishing odometry to: "
                        << config_.odometry_path_topic);
    } else {
        ROS_INFO_STREAM("[WaypointRecorder] mode=direct, subscribing odometry from: "
                        << config_.current_odometry_topic);
    }
    ROS_INFO_STREAM("[WaypointRecorder] services: record="
                    << config_.record_service_name
                    << ", save=" << config_.save_service_name
                    << ", clear=" << config_.clear_service_name);
}

WaypointRecorder::~WaypointRecorder() = default;

// =============================================================================
// 参数加载
// =============================================================================

void WaypointRecorder::LoadParameters(ros::NodeHandle& pnh) {
    (void)pnh;  // 全部参数走绝对路径，从全局命名空间读取
    const std::string p = "/indooruav_waypoint/waypoint_recorder/";

    nh_.param<std::string>(p + "mode",
                           config_.mode,
                           std::string("direct"));

    nh_.param<std::string>(p + "topics/current_odometry",
                           config_.current_odometry_topic,
                           std::string("/Odometry_global_rotate"));
    nh_.param<std::string>(p + "topics/global_path",
                           config_.global_path_topic,
                           std::string("/global_path"));
    nh_.param<std::string>(p + "topics/odometry_path",
                           config_.odometry_path_topic,
                           std::string("/Odometry_path"));

    nh_.param<std::string>(p + "services/record",
                           config_.record_service_name,
                           std::string("indooruav_controller/waypoint_recorder/record"));
    nh_.param<std::string>(p + "services/save",
                           config_.save_service_name,
                           std::string("indooruav_controller/waypoint_recorder/save"));
    nh_.param<std::string>(p + "services/clear",
                           config_.clear_service_name,
                           std::string("indooruav_controller/waypoint_recorder/clear"));
    nh_.param<std::string>(p + "services/auto_record_start",
                           config_.auto_record_start_service_name,
                           std::string("indooruav_controller/waypoint_recorder/auto_record_start"));
    nh_.param<std::string>(p + "services/auto_record_stop",
                           config_.auto_record_stop_service_name,
                           std::string("indooruav_controller/waypoint_recorder/auto_record_stop"));

    nh_.param<std::string>(p + "waypoints_package_name",
                           config_.waypoints_package_name,
                           std::string("indooruav_waypoint"));
    nh_.param<std::string>(p + "waypoints_file_path",
                           config_.waypoints_file_path,
                           std::string("config/waypoints.yaml"));

    nh_.param<bool>(p + "parameters/load_existing",
                    config_.load_existing, false);
    nh_.param<double>(p + "parameters/auto_record_interval_s",
                      config_.auto_record_interval_s, 3.0);
    config_.auto_record_interval_s = std::max(config_.auto_record_interval_s, 0.5);
}

// =============================================================================
// ROS 接口
// =============================================================================

void WaypointRecorder::SetupRosInterfaces() {
    if (config_.mode == "path_relay") {
        global_path_subscriber_ = nh_.subscribe(
            config_.global_path_topic, 20,
            &WaypointRecorder::GlobalPathCallback, this);
        odometry_path_publisher_ = nh_.advertise<nav_msgs::Odometry>(
            config_.odometry_path_topic, 20);
    } else {
        odometry_subscriber_ = nh_.subscribe(
            config_.current_odometry_topic, 20,
            &WaypointRecorder::OdometryCallback, this);
    }

    record_service_ = nh_.advertiseService(
        config_.record_service_name,
        &WaypointRecorder::RecordServiceCallback, this);
    save_service_ = nh_.advertiseService(
        config_.save_service_name,
        &WaypointRecorder::SaveServiceCallback, this);
    clear_service_ = nh_.advertiseService(
        config_.clear_service_name,
        &WaypointRecorder::ClearServiceCallback, this);
    auto_record_start_service_ = nh_.advertiseService(
        config_.auto_record_start_service_name,
        &WaypointRecorder::AutoRecordStartServiceCallback, this);
    auto_record_stop_service_ = nh_.advertiseService(
        config_.auto_record_stop_service_name,
        &WaypointRecorder::AutoRecordStopServiceCallback, this);

    // 创建自动录制定时器，初始停止
    auto_record_timer_ = nh_.createTimer(
        ros::Duration(config_.auto_record_interval_s),
        &WaypointRecorder::AutoRecordTimerCallback, this);
    auto_record_timer_.stop();
}

void WaypointRecorder::OdometryCallback(
    const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odometry_ = *msg;
    has_odometry_ = true;
}

void WaypointRecorder::GlobalPathCallback(
    const nav_msgs::Path::ConstPtr& msg) {
    if (msg->poses.empty()) {
        return;
    }

    const auto& last_pose = msg->poses.back();

    nav_msgs::Odometry odom;
    odom.header.stamp = last_pose.header.stamp;
    odom.header.frame_id = msg->header.frame_id;
    odom.child_frame_id = "body";
    odom.pose.pose = last_pose.pose;

    odometry_path_publisher_.publish(odom);

    std::lock_guard<std::mutex> lock(mutex_);
    latest_odometry_ = odom;
    has_odometry_ = true;
}

// =============================================================================
// 文件路径解析
// =============================================================================

bool WaypointRecorder::ResolveWaypointFilePath(
    std::string* resolved_path) const {
    if (resolved_path == nullptr) {
        return false;
    }

    if (config_.waypoints_file_path.empty()) {
        ROS_ERROR("[WaypointRecorder] waypoints_file_path is empty.");
        return false;
    }

    if (config_.waypoints_file_path.front() == '/') {
        *resolved_path = config_.waypoints_file_path;
        return true;
    }

    if (config_.waypoints_package_name.empty()) {
        ROS_ERROR_STREAM(
            "[WaypointRecorder] Relative waypoints_file_path requires a "
            "non-empty waypoints_package_name. path="
            << config_.waypoints_file_path);
        return false;
    }

    const std::string package_path =
        ros::package::getPath(config_.waypoints_package_name);
    if (package_path.empty()) {
        ROS_ERROR_STREAM("[WaypointRecorder] Cannot resolve package: "
                         << config_.waypoints_package_name);
        return false;
    }

    *resolved_path = package_path + "/" + config_.waypoints_file_path;
    return true;
}

// =============================================================================
// 服务回调
// =============================================================================

bool WaypointRecorder::RecordServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res) {
    (void)req;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_odometry_) {
        const std::string source = (config_.mode == "path_relay")
            ? config_.global_path_topic : config_.current_odometry_topic;
        res.success = false;
        res.message = "No odometry received yet from " + source;
        ROS_WARN_STREAM("[WaypointRecorder] " << res.message);
        return true;
    }

    Waypoint wp;
    wp.x_m = latest_odometry_.pose.pose.position.x;
    wp.y_m = latest_odometry_.pose.pose.position.y;
    wp.z_m = latest_odometry_.pose.pose.position.z;
    const double yaw_rad =
        GetYawFromQuaternion(latest_odometry_.pose.pose.orientation);
    wp.yaw_deg = RadToDeg(yaw_rad);
    wp.stop = true;
    waypoints_.push_back(wp);

    std::ostringstream oss;
    oss << "[Stop] Recorded waypoint #" << waypoints_.size()
        << ": x=" << wp.x_m
        << ", y=" << wp.y_m
        << ", z=" << wp.z_m
        << ", yaw_deg=" << wp.yaw_deg;
    res.success = true;
    res.message = oss.str();
    ROS_INFO_STREAM("[WaypointRecorder] " << res.message);
    return true;
}

bool WaypointRecorder::SaveServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res) {
    (void)req;
    std::lock_guard<std::mutex> lock(mutex_);

    std::string error_message;
    if (!SaveWaypointsToFile(&error_message)) {
        res.success = false;
        res.message = error_message;
        ROS_ERROR_STREAM("[WaypointRecorder] Save failed: " << error_message);
        return true;
    }

    std::ostringstream oss;
    oss << "Saved " << waypoints_.size() << " waypoints to "
        << config_.waypoints_file_path;
    res.success = true;
    res.message = oss.str();
    ROS_INFO_STREAM("[WaypointRecorder] " << res.message);
    return true;
}

bool WaypointRecorder::ClearServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res) {
    (void)req;
    std::lock_guard<std::mutex> lock(mutex_);

    const std::size_t old_size = waypoints_.size();
    waypoints_.clear();
    std::ostringstream oss;
    oss << "Cleared in-memory waypoint list (was " << old_size
        << " entries; file unchanged).";
    res.success = true;
    res.message = oss.str();
    ROS_INFO_STREAM("[WaypointRecorder] " << res.message);
    return true;
}

// =============================================================================
// 自动录制服务
// =============================================================================

bool WaypointRecorder::AutoRecordStartServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res) {
    (void)req;
    std::lock_guard<std::mutex> lock(mutex_);

    if (auto_recording_active_) {
        res.success = true;
        res.message = "Auto recording already active.";
        ROS_WARN("[WaypointRecorder] %s", res.message.c_str());
        return true;
    }

    if (!has_odometry_) {
        const std::string source = (config_.mode == "path_relay")
            ? config_.global_path_topic : config_.current_odometry_topic;
        res.success = false;
        res.message = "No odometry received yet from " + source;
        ROS_WARN_STREAM("[WaypointRecorder] " << res.message);
        return true;
    }

    auto_recording_active_ = true;
    auto_record_timer_.start();

    std::ostringstream oss;
    oss << "Auto recording started (interval="
        << config_.auto_record_interval_s << "s).";
    res.success = true;
    res.message = oss.str();
    ROS_INFO_STREAM("[WaypointRecorder] " << res.message);
    return true;
}

bool WaypointRecorder::AutoRecordStopServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res) {
    (void)req;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!auto_recording_active_) {
        res.success = true;
        res.message = "Auto recording already stopped.";
        ROS_WARN("[WaypointRecorder] %s", res.message.c_str());
        return true;
    }

    auto_recording_active_ = false;
    auto_record_timer_.stop();

    std::ostringstream oss;
    oss << "Auto recording stopped (recorded "
        << waypoints_.size() << " waypoints total).";
    res.success = true;
    res.message = oss.str();
    ROS_INFO_STREAM("[WaypointRecorder] " << res.message);
    return true;
}

void WaypointRecorder::AutoRecordTimerCallback(const ros::TimerEvent& event) {
    (void)event;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_odometry_) {
        ROS_WARN_THROTTLE(2.0, "[WaypointRecorder] [Auto] No odometry, skipping.");
        return;
    }

    Waypoint wp;
    wp.x_m = latest_odometry_.pose.pose.position.x;
    wp.y_m = latest_odometry_.pose.pose.position.y;
    wp.z_m = latest_odometry_.pose.pose.position.z;
    const double yaw_rad =
        GetYawFromQuaternion(latest_odometry_.pose.pose.orientation);
    wp.yaw_deg = RadToDeg(yaw_rad);
    wp.stop = false;
    waypoints_.push_back(wp);

    ROS_INFO("[WaypointRecorder] [Auto] Recorded waypoint #%zu: "
             "x=%.3f, y=%.3f, z=%.3f, yaw_deg=%.1f",
             waypoints_.size(), wp.x_m, wp.y_m, wp.z_m, wp.yaw_deg);
}

// =============================================================================
// 加载已有航点（仅在 load_existing=true 时启动期调用）
// =============================================================================

bool WaypointRecorder::LoadExistingWaypoints() {
    std::string resolved_path;
    if (!ResolveWaypointFilePath(&resolved_path)) {
        return false;
    }

    std::ifstream ifs(resolved_path);
    if (!ifs.is_open()) {
        ROS_INFO_STREAM("[WaypointRecorder] No existing waypoint file at: "
                        << resolved_path);
        return false;
    }

    YAML::Node root;
    try {
        root = YAML::Load(ifs);
    } catch (const YAML::Exception& e) {
        ROS_WARN_STREAM("[WaypointRecorder] Existing file YAML parse error: "
                        << e.what());
        return false;
    }

    if (!root["waypoints"] || !root["waypoints"].IsSequence()) {
        ROS_WARN_STREAM(
            "[WaypointRecorder] Existing file has no 'waypoints' sequence: "
            << resolved_path);
        return false;
    }

    waypoints_.clear();
    waypoints_.reserve(root["waypoints"].size());
    try {
        for (std::size_t i = 0; i < root["waypoints"].size(); ++i) {
            const YAML::Node& node = root["waypoints"][i];
            Waypoint wp;
            wp.x_m = node["x"].as<double>();
            wp.y_m = node["y"].as<double>();
            wp.z_m = node["z"].as<double>();
            wp.yaw_deg = node["yaw_deg"].as<double>();
            wp.stop = node["stop"] ? node["stop"].as<bool>() : false;
            waypoints_.push_back(wp);
        }
    } catch (const YAML::Exception& e) {
        ROS_WARN_STREAM("[WaypointRecorder] Malformed existing entry: "
                        << e.what());
        waypoints_.clear();
        return false;
    }

    return true;
}

// =============================================================================
// 保存到 YAML 文件（覆盖写）
// =============================================================================

bool WaypointRecorder::SaveWaypointsToFile(std::string* error_message) {
    std::string resolved_path;
    if (!ResolveWaypointFilePath(&resolved_path)) {
        if (error_message != nullptr) {
            *error_message = "Cannot resolve waypoint file path.";
        }
        return false;
    }

    if (!EnsureParentDirectory(resolved_path, error_message)) {
        return false;
    }

    std::ofstream ofs(resolved_path,
                      std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
        if (error_message != nullptr) {
            *error_message = "Cannot open file for writing: " + resolved_path;
        }
        return false;
    }

    // 头注释
    ofs << "# indooruav_waypoint recorded waypoints\n";
    ofs << "# generated_at: " << CurrentTimestampString() << "\n";
    ofs << "# count: " << waypoints_.size() << "\n";
    ofs << "# fields: x [m], y [m], z [m], yaw_deg [deg], stop [bool]\n";
    ofs << "waypoints:\n";

    if (waypoints_.empty()) {
        ofs << "  []\n";
    } else {
        ofs << std::fixed;
        for (std::size_t i = 0; i < waypoints_.size(); ++i) {
            const Waypoint& wp = waypoints_[i];
            ofs << "  - {"
                << "x: " << std::setprecision(3) << wp.x_m
                << ", y: " << std::setprecision(3) << wp.y_m
                << ", z: " << std::setprecision(3) << wp.z_m
                << ", yaw_deg: " << std::setprecision(1) << wp.yaw_deg
                << ", stop: " << (wp.stop ? "true" : "false")
                << "}\n";
        }
    }

    ofs.flush();
    if (!ofs.good()) {
        if (error_message != nullptr) {
            *error_message = "Write error while saving: " + resolved_path;
        }
        return false;
    }

    ofs.close();
    ROS_INFO_STREAM("[WaypointRecorder] Wrote waypoint file: " << resolved_path);
    return true;
}

// =============================================================================
// 工具函数
// =============================================================================

double WaypointRecorder::GetYawFromQuaternion(
    const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double WaypointRecorder::RadToDeg(double rad) {
    return rad * kRadToDegConst;
}

bool WaypointRecorder::EnsureParentDirectory(const std::string& file_path,
                                             std::string* error_message) {
    // 取父目录
    const std::size_t slash = file_path.find_last_of('/');
    if (slash == std::string::npos) {
        return true;  // 无父路径，按当前工作目录处理
    }
    const std::string dir = file_path.substr(0, slash);
    if (dir.empty()) {
        return true;
    }

    // 递归 mkdir -p
    std::string acc;
    if (dir.front() == '/') {
        acc = "/";
    }
    std::stringstream ss(dir);
    std::string segment;
    while (std::getline(ss, segment, '/')) {
        if (segment.empty()) continue;
        if (acc.empty() || acc.back() != '/') {
            acc.push_back('/');
        }
        acc += segment;

        struct stat st;
        if (stat(acc.c_str(), &st) == 0) {
            if (!S_ISDIR(st.st_mode)) {
                if (error_message != nullptr) {
                    *error_message = "Path component exists but is not a directory: " + acc;
                }
                return false;
            }
            continue;
        }
        // 不存在，尝试创建
        if (mkdir(acc.c_str(), 0755) != 0 && errno != EEXIST) {
            if (error_message != nullptr) {
                *error_message =
                    "mkdir failed for " + acc + ": " + std::strerror(errno);
            }
            return false;
        }
    }
    return true;
}

std::string WaypointRecorder::CurrentTimestampString() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm_value;
#if defined(_WIN32)
    localtime_s(&tm_value, &tt);
#else
    localtime_r(&tt, &tm_value);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_value, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

}  // namespace indooruav_waypoint
