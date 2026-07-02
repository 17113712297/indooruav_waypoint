#include "indooruav_waypoint/waypoint_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/package.h>
#include <yaml-cpp/yaml.h>

namespace indooruav_waypoint {

namespace {

// 将 deg 安全转换到 rad
inline double DegToRad(double deg) {
    return deg * (3.14159265358979323846 / 180.0);
}

// 取符号；零返回 0
inline double SignOf(double v) {
    if (v > 0.0) return 1.0;
    if (v < 0.0) return -1.0;
    return 0.0;
}

}  // namespace

// =============================================================================
// 构造与析构
// =============================================================================

WaypointTracker::WaypointTracker()
    : nh_(),
      pnh_("~") {
    LoadParameters(pnh_);

    // 加载航点 YAML
    if (!LoadWaypointsFromYaml()) {
        ROS_ERROR(
            "[WaypointTracker] Failed to load waypoints. Node will keep "
            "running but tracking is disabled until a valid file is provided "
            "(restart required).");
        // 仍然搭建 ROS 接口，但状态保持 IDLE
    }

    SetupRosInterfaces();

    // 初始化限幅器
    xy_vx_limiter_.SetLimits(config_.xy_limits);
    xy_vy_limiter_.SetLimits(config_.xy_limits);
    z_limiter_.SetLimits(config_.z_limits);
    yaw_limiter_.SetLimits(config_.yaw_limits);

    initialized_ = true;
    state_ = State::IDLE;

    if (config_.autostart && !waypoints_.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        EnterRunningLocked();
        ROS_INFO("[WaypointTracker] autostart=true, entered RUNNING state.");
    } else {
        ROS_INFO(
            "[WaypointTracker] Initialized. Waiting for /start service call. "
            "(loaded waypoints=%zu)",
            waypoints_.size());
    }
}

WaypointTracker::~WaypointTracker() {
    // 析构时尝试发一次零速指令保持安全
    if (velocity_command_publisher_) {
        PublishZeroVelocityCommand();
    }
}

// =============================================================================
// 参数加载
// =============================================================================

void WaypointTracker::LoadParameters(ros::NodeHandle& pnh) {
    (void)pnh;  // 全部参数走绝对路径，从全局命名空间读取
    const std::string p = "/indooruav_waypoint/waypoint_tracker/";

    // topics
    nh_.param<std::string>(p + "topics/current_odometry",
                           config_.current_odometry_topic,
                           std::string("/Odometry_global_rotate"));
    nh_.param<std::string>(p + "topics/desired_odometry",
                           config_.desired_odometry_topic,
                           std::string("indooruav_controller/waypoint_tracker/odom_goal"));
    nh_.param<std::string>(p + "topics/velocity_command",
                           config_.velocity_command_topic,
                           std::string("indooruav_controller/waypoint_tracker/cmd_vel"));

    // services
    nh_.param<std::string>(p + "services/start",
                           config_.start_service_name,
                           std::string("indooruav_controller/waypoint_tracker/start"));
    nh_.param<std::string>(p + "services/pause",
                           config_.pause_service_name,
                           std::string("indooruav_controller/waypoint_tracker/pause"));
    nh_.param<std::string>(p + "services/stop",
                           config_.stop_service_name,
                           std::string("indooruav_controller/waypoint_tracker/stop"));
    nh_.param<std::string>(p + "services/cruise_complete",
                           config_.cruise_complete_service_name,
                           std::string("indooruav_core/state_machine_event/cruise_complete"));

    // file
    nh_.param<std::string>(p + "waypoints_package_name",
                           config_.waypoints_package_name,
                           std::string("indooruav_waypoint"));
    nh_.param<std::string>(p + "waypoints_file_path",
                           config_.waypoints_file_path,
                           std::string("waypoints/waypoints.yaml"));

    // parameters
    nh_.param<double>(p + "parameters/control_rate_hz",
                      config_.control_rate_hz, 30.0);
    nh_.param<double>(p + "parameters/odometry_timeout_s",
                      config_.odometry_timeout_s, 0.2);
    nh_.param<bool>(p + "parameters/autostart",
                    config_.autostart, false);

    nh_.param<double>(p + "parameters/xy_speed_mps",
                      config_.xy_speed_mps, 0.5);
    nh_.param<double>(p + "parameters/z_speed_mps",
                      config_.z_speed_mps, 0.3);

    double yaw_speed_dps = 30.0;
    nh_.param<double>(p + "parameters/yaw_speed_dps", yaw_speed_dps, 30.0);
    config_.yaw_speed_radps = DegToRad(yaw_speed_dps);

    // XY 限幅
    config_.xy_limits.v_max = config_.xy_speed_mps;
    nh_.param<double>(p + "parameters/xy_accel_max_mps2",
                      config_.xy_limits.a_max, 0.5);
    nh_.param<double>(p + "parameters/xy_jerk_max_mps3",
                      config_.xy_limits.j_max, 2.0);
    nh_.param<double>(p + "parameters/xy_snap_max_mps4",
                      config_.xy_limits.s_max, 1.0e6);

    // Z 限幅
    config_.z_limits.v_max = config_.z_speed_mps;
    nh_.param<double>(p + "parameters/z_accel_max_mps2",
                      config_.z_limits.a_max, 0.4);
    nh_.param<double>(p + "parameters/z_jerk_max_mps3",
                      config_.z_limits.j_max, 2.0);
    nh_.param<double>(p + "parameters/z_snap_max_mps4",
                      config_.z_limits.s_max, 1.0e6);

    // Yaw 限幅（参数读取为度，内部转弧度）
    config_.yaw_limits.v_max = config_.yaw_speed_radps;

    double yaw_accel_dps2 = 60.0;
    double yaw_jerk_dps3 = 300.0;
    double yaw_snap_dps4 = 1.0e6;
    nh_.param<double>(p + "parameters/yaw_accel_max_dps2", yaw_accel_dps2, 60.0);
    nh_.param<double>(p + "parameters/yaw_jerk_max_dps3", yaw_jerk_dps3, 300.0);
    nh_.param<double>(p + "parameters/yaw_snap_max_dps4", yaw_snap_dps4, 1.0e6);
    config_.yaw_limits.a_max = DegToRad(yaw_accel_dps2);
    config_.yaw_limits.j_max = DegToRad(yaw_jerk_dps3);
    config_.yaw_limits.s_max = DegToRad(yaw_snap_dps4);

    // 位置 P 反馈增益
    nh_.param<double>(p + "parameters/kp_xy", config_.kp_xy, 2.0);
    nh_.param<double>(p + "parameters/kp_z", config_.kp_z, 1.5);
    nh_.param<double>(p + "parameters/kp_yaw_rotate",
                      config_.kp_yaw_rotate, 3.0);
    nh_.param<double>(p + "parameters/kp_yaw_hold",
                      config_.kp_yaw_hold, 0.5);

    // 到达 / 切换阈值
    nh_.param<double>(p + "parameters/xy_arrival_tolerance_m",
                      config_.xy_arrival_tolerance_m, 0.15);
    nh_.param<double>(p + "parameters/z_arrival_tolerance_m",
                      config_.z_arrival_tolerance_m, 0.15);
    nh_.param<double>(p + "parameters/xy_switch_radius_m",
                      config_.xy_switch_radius_m, 0.3);

    double yaw_rotation_tol_deg = 5.0;
    double yaw_arrival_tol_deg = 5.0;
    nh_.param<double>(p + "parameters/yaw_rotation_tolerance_deg",
                      yaw_rotation_tol_deg, 5.0);
    nh_.param<double>(p + "parameters/yaw_arrival_tolerance_deg",
                      yaw_arrival_tol_deg, 5.0);
    config_.yaw_rotation_tolerance_rad = DegToRad(yaw_rotation_tol_deg);
    config_.yaw_arrival_tolerance_rad = DegToRad(yaw_arrival_tol_deg);

    nh_.param<bool>(p + "parameters/ignore_arrival_yaw",
                    config_.ignore_arrival_yaw, true);

    // 数值兜底：增益必须非负
    config_.kp_xy = std::max(config_.kp_xy, 0.0);
    config_.kp_z = std::max(config_.kp_z, 0.0);
    config_.kp_yaw_rotate = std::max(config_.kp_yaw_rotate, 0.0);
    config_.kp_yaw_hold = std::max(config_.kp_yaw_hold, 0.0);
    // 数值兜底
    config_.control_rate_hz = std::max(config_.control_rate_hz, 1.0);
    config_.odometry_timeout_s = std::max(config_.odometry_timeout_s, 0.01);

    ROS_INFO_STREAM("[WaypointTracker] current_odometry_topic: "
                    << config_.current_odometry_topic);
    ROS_INFO_STREAM("[WaypointTracker] desired_odometry_topic: "
                    << config_.desired_odometry_topic);
    ROS_INFO_STREAM("[WaypointTracker] velocity_command_topic: "
                    << config_.velocity_command_topic);
    ROS_INFO("[WaypointTracker] xy_speed=%.2f m/s, z_speed=%.2f m/s, yaw_speed=%.1f deg/s",
             config_.xy_speed_mps, config_.z_speed_mps, yaw_speed_dps);
    ROS_INFO("[WaypointTracker] arrival tol: xy=%.2fm, z=%.2fm, "
             "yaw_rotation=%.1fdeg, yaw_arrival=%.1fdeg, ignore_arrival_yaw=%s",
             config_.xy_arrival_tolerance_m,
             config_.z_arrival_tolerance_m,
             yaw_rotation_tol_deg,
             yaw_arrival_tol_deg,
             config_.ignore_arrival_yaw ? "true" : "false");
    ROS_INFO("[WaypointTracker] Kp xy=%.2f, z=%.2f, yaw_rotate=%.2f, yaw_hold=%.2f (1/s)",
             config_.kp_xy, config_.kp_z,
             config_.kp_yaw_rotate, config_.kp_yaw_hold);
    ROS_INFO("[WaypointTracker] control_rate=%.1f Hz, odom_timeout=%.3f s, autostart=%s",
             config_.control_rate_hz,
             config_.odometry_timeout_s,
             config_.autostart ? "true" : "false");
}

// =============================================================================
// 航点 YAML 加载
// =============================================================================

bool WaypointTracker::ResolveWaypointFilePath(std::string* resolved_path) const {
    if (resolved_path == nullptr) {
        return false;
    }

    if (config_.waypoints_file_path.empty()) {
        ROS_ERROR("[WaypointTracker] waypoints_file_path is empty.");
        return false;
    }

    if (config_.waypoints_file_path.front() == '/') {
        *resolved_path = config_.waypoints_file_path;
        return true;
    }

    if (config_.waypoints_package_name.empty()) {
        ROS_ERROR_STREAM(
            "[WaypointTracker] Relative waypoints_file_path requires a "
            "non-empty waypoints_package_name. path="
            << config_.waypoints_file_path);
        return false;
    }

    const std::string package_path =
        ros::package::getPath(config_.waypoints_package_name);
    if (package_path.empty()) {
        ROS_ERROR_STREAM("[WaypointTracker] Cannot resolve package: "
                         << config_.waypoints_package_name);
        return false;
    }

    *resolved_path = package_path + "/" + config_.waypoints_file_path;
    return true;
}

bool WaypointTracker::LoadWaypointsFromYaml() {
    std::string resolved_path;
    if (!ResolveWaypointFilePath(&resolved_path)) {
        return false;
    }

    std::ifstream ifs(resolved_path);
    if (!ifs.is_open()) {
        ROS_ERROR_STREAM("[WaypointTracker] Cannot open waypoint file: "
                         << resolved_path);
        return false;
    }

    YAML::Node root;
    try {
        root = YAML::Load(ifs);
    } catch (const YAML::Exception& e) {
        ROS_ERROR_STREAM("[WaypointTracker] YAML parse error in "
                         << resolved_path << ": " << e.what());
        return false;
    }

    if (!root["waypoints"] || !root["waypoints"].IsSequence()) {
        ROS_ERROR_STREAM(
            "[WaypointTracker] Missing or non-sequence 'waypoints' field in: "
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
            const double yaw_deg = node["yaw_deg"].as<double>();
            wp.yaw_rad = NormalizeAngleRad(DegToRad(yaw_deg));
            wp.stop = node["stop"] ? node["stop"].as<bool>() : false;
            waypoints_.push_back(wp);
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR_STREAM("[WaypointTracker] Malformed waypoint entry: "
                         << e.what());
        waypoints_.clear();
        return false;
    }

    if (waypoints_.empty()) {
        ROS_WARN_STREAM("[WaypointTracker] Waypoint list is empty in: "
                        << resolved_path);
    } else {
        // 找到第一个和最后一个停留点
        int first_stop = -1;
        int last_stop = -1;
        for (int i = 0; i < static_cast<int>(waypoints_.size()); ++i) {
            if (!waypoints_[i].stop) continue;
            if (first_stop < 0) first_stop = i;
            last_stop = i;
        }

        // 没有停留点 → 全部为穿越点，不执行跟踪
        if (last_stop < 0) {
            ROS_WARN("[WaypointTracker] No stop waypoint found. All waypoints are pass-through, nothing to track.");
            waypoints_.clear();
            return true;
        }

        // 截掉第一个停留点之前的穿越点（对称于后端的 resize）
        if (first_stop > 0) {
            waypoints_.erase(waypoints_.begin(), waypoints_.begin() + first_stop);
            last_stop -= first_stop;
            ROS_INFO("[WaypointTracker] Trimmed %d pass-through waypoints before first stop.",
                     first_stop);
            first_stop = 0;
        }

        // 截掉最后一个停留点之后的穿越点
        if (last_stop + 1 < static_cast<int>(waypoints_.size())) {
            const std::size_t removed = waypoints_.size() - (last_stop + 1);
            waypoints_.resize(last_stop + 1);
            ROS_INFO("[WaypointTracker] Trimmed %zu pass-through waypoints after last stop.",
                     removed);
        }

        // 屏蔽停留点 1m 半径内的穿越点
        constexpr double kStopShieldRadius = 1.0;
        std::vector<Waypoint> filtered;
        filtered.reserve(waypoints_.size());
        for (const auto& wp : waypoints_) {
            if (wp.stop) {
                filtered.push_back(wp);
                continue;
            }
            bool near_stop = false;
            for (const auto& sw : waypoints_) {
                if (!sw.stop) continue;
                const double ddx = wp.x_m - sw.x_m;
                const double ddy = wp.y_m - sw.y_m;
                if (std::sqrt(ddx * ddx + ddy * ddy) < kStopShieldRadius) {
                    near_stop = true;
                    break;
                }
            }
            if (!near_stop) {
                filtered.push_back(wp);
            }
        }
        if (filtered.size() < waypoints_.size()) {
            ROS_INFO("[WaypointTracker] Filtered %zu pass-through waypoints near stop waypoints.",
                     waypoints_.size() - filtered.size());
            waypoints_ = std::move(filtered);
        }
        ROS_INFO_STREAM("[WaypointTracker] Loaded " << waypoints_.size()
                        << " waypoints from " << resolved_path);
    }
    return true;
}

// =============================================================================
// ROS 接口
// =============================================================================

void WaypointTracker::SetupRosInterfaces() {
    odometry_subscriber_ = nh_.subscribe(
        config_.current_odometry_topic, 20,
        &WaypointTracker::OdometryCallback, this);

    velocity_command_publisher_ =
        nh_.advertise<geometry_msgs::Twist>(config_.velocity_command_topic, 5, false);
    desired_odometry_publisher_ =
        nh_.advertise<nav_msgs::Odometry>(config_.desired_odometry_topic, 5, false);

    start_service_ = nh_.advertiseService(
        config_.start_service_name,
        &WaypointTracker::StartServiceCallback, this);
    pause_service_ = nh_.advertiseService(
        config_.pause_service_name,
        &WaypointTracker::PauseServiceCallback, this);
    stop_service_ = nh_.advertiseService(
        config_.stop_service_name,
        &WaypointTracker::StopServiceCallback, this);

    cruise_complete_client_ = nh_.serviceClient<std_srvs::Empty>(
        config_.cruise_complete_service_name);

    cruise_complete_timer_ = nh_.createTimer(
        ros::Duration(0.001),
        &WaypointTracker::CruiseCompleteTimerCallback,
        this, true /* oneshot */);
    cruise_complete_timer_.stop();

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / config_.control_rate_hz),
        &WaypointTracker::ControlTimerCallback, this);
}

void WaypointTracker::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odometry_ = *msg;
    latest_odometry_stamp_ = ros::Time::now();
    has_odometry_ = true;
    stale_zero_command_sent_ = false;
}

// =============================================================================
// 服务回调
// =============================================================================

bool WaypointTracker::StartServiceCallback(std_srvs::Empty::Request& req,
                                           std_srvs::Empty::Response& res) {
    (void)req;
    (void)res;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_ || waypoints_.empty()) {
        ROS_WARN("[WaypointTracker] Start rejected: not initialized or no waypoints.");
        return true;
    }

    if (!has_odometry_) {
        ROS_WARN("[WaypointTracker] Start rejected: no odometry received yet on %s",
                 config_.current_odometry_topic.c_str());
        return true;
    }

    switch (state_) {
        case State::RUNNING:
            ROS_INFO("[WaypointTracker] Already RUNNING, no change.");
            return true;
        case State::PAUSED:
            EnterRunningLocked();
            ROS_INFO_STREAM("[WaypointTracker] Resumed from PAUSED at index "
                            << current_index_);
            return true;
        case State::FINISHED:
            current_index_ = 0;
            EnterRunningLocked();
            ROS_INFO("[WaypointTracker] Restarted from index 0 (was FINISHED).");
            return true;
        case State::IDLE:
        default:
            current_index_ = 0;
            EnterRunningLocked();
            ROS_INFO("[WaypointTracker] Started from index 0.");
            return true;
    }
}

bool WaypointTracker::PauseServiceCallback(std_srvs::Empty::Request& req,
                                           std_srvs::Empty::Response& res) {
    (void)req;
    (void)res;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        ROS_WARN("[WaypointTracker] Pause rejected: not initialized.");
        return true;
    }

    if (state_ == State::PAUSED) {
        ROS_INFO("[WaypointTracker] Already PAUSED.");
        return true;
    }

    EnterPausedLocked();
    ROS_INFO_STREAM("[WaypointTracker] Paused at index " << current_index_);
    return true;
}

bool WaypointTracker::StopServiceCallback(std_srvs::Empty::Request& req,
                                          std_srvs::Empty::Response& res) {
    (void)req;
    (void)res;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        ROS_WARN("[WaypointTracker] Stop rejected: not initialized.");
        return true;
    }

    EnterIdleLocked();
    current_index_ = 0;
    ROS_INFO("[WaypointTracker] Stopped and reset to index 0.");
    return true;
}

// =============================================================================
// 状态机内部转换（locked 版本，调用方需持有 mutex_）
// =============================================================================

void WaypointTracker::EnterIdleLocked() {
    state_ = State::IDLE;
    phase_ = Phase::PHASE_ROTATE;
    ResetLimiters();
    last_control_time_ = ros::Time();
}

void WaypointTracker::EnterRunningLocked() {
    state_ = State::RUNNING;
    // 每次进入 RUNNING 都从当前航点的“原地旋转”阶段开始；
    // 限幅器一律清零，保证旋转从静止起步，避免 pause/stop 残留状态污染。
    phase_ = Phase::PHASE_ROTATE;
    ResetLimiters();
    last_control_time_ = ros::Time();
}

void WaypointTracker::EnterPausedLocked() {
    state_ = State::PAUSED;
    phase_ = Phase::PHASE_ROTATE;
    ResetLimiters();
    last_control_time_ = ros::Time();
}

void WaypointTracker::EnterFinishedLocked() {
    state_ = State::FINISHED;
    phase_ = Phase::PHASE_ROTATE;
    ResetLimiters();
    last_control_time_ = ros::Time();
    ROS_INFO("[WaypointTracker] All waypoints completed. Holding position.");
}

void WaypointTracker::ResetLimiters() {
    xy_vx_limiter_.Reset();
    xy_vy_limiter_.Reset();
    z_limiter_.Reset();
    yaw_limiter_.Reset();
}

// =============================================================================
// 主控制循环
// =============================================================================

void WaypointTracker::ControlTimerCallback(const ros::TimerEvent& event) {
    (void)event;
    if (!initialized_) {
        return;
    }
    RunControlStep(ros::Time::now());

    // 锁外：检查巡航完成标志，触发 oneshot timer
    if (cruise_complete_pending_) {
        cruise_complete_pending_ = false;
        cruise_complete_timer_.start();
    }
}

void WaypointTracker::CruiseCompleteTimerCallback(const ros::TimerEvent&) {
    // [锁外] 等待服务就绪 + 调用，避免阻塞其他回调
    if (!cruise_complete_client_.waitForExistence(ros::Duration(2.0))) {
        ++cruise_complete_retry_count_;
        if (cruise_complete_retry_count_ >= kMaxCruiseCompleteRetries) {
            ROS_ERROR("[WaypointTracker] cruise_complete service unavailable after %d retries, giving up.",
                      cruise_complete_retry_count_);
            cruise_complete_retry_count_ = 0;
            // 仍然重置 tracker 状态，避免节点卡死
            std::lock_guard<std::mutex> lock(mutex_);
            EnterIdleLocked();
            current_index_ = 0;
            return;
        }
        ROS_WARN_THROTTLE(1.0,
            "[WaypointTracker] cruise_complete service not available, retry %d/%d...",
            cruise_complete_retry_count_, kMaxCruiseCompleteRetries);
        cruise_complete_pending_ = true;  // 触发下一个 tick 重试
        return;
    }

    std_srvs::Empty srv;
    if (!cruise_complete_client_.call(srv)) {
        ++cruise_complete_retry_count_;
        if (cruise_complete_retry_count_ >= kMaxCruiseCompleteRetries) {
            ROS_ERROR("[WaypointTracker] cruise_complete service call failed after %d retries, giving up.",
                      cruise_complete_retry_count_);
            cruise_complete_retry_count_ = 0;
            std::lock_guard<std::mutex> lock(mutex_);
            EnterIdleLocked();
            current_index_ = 0;
            return;
        }
        ROS_WARN_THROTTLE(1.0,
            "[WaypointTracker] cruise_complete service call failed, retry %d/%d...",
            cruise_complete_retry_count_, kMaxCruiseCompleteRetries);
        cruise_complete_pending_ = true;  // 触发下一个 tick 重试
        return;
    }

    // [锁内] 服务调用成功，重置 tracker 状态
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EnterIdleLocked();
        current_index_ = 0;
        ROS_INFO("[WaypointTracker] Called cruise_complete event service successfully. Self-reset to IDLE.");
    }
    cruise_complete_retry_count_ = 0;
}

void WaypointTracker::RunControlStep(const ros::Time& now) {
    geometry_msgs::Twist out_cmd;
    bool publish_zero_once = false;
    bool publish_cmd = false;
    bool publish_desired_step = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);

        // 非 RUNNING 状态：维持悬停
        if (state_ != State::RUNNING) {
            // 持续以低频发零速指令，避免飞控失控保护
            // （IDLE/PAUSED/FINISHED 都走这里）
            ResetLimiters();
            last_control_time_ = ros::Time();
            // 直接构造零速指令（默认全 0）
            publish_cmd = true;
            // 发布期望里程计（当前目标航点），方便 RViz 观察
            publish_desired_step = !waypoints_.empty();
            // 检测 FINISHED 状态，设置巡航完成标志
            if (state_ == State::FINISHED && !cruise_complete_pending_) {
                cruise_complete_pending_ = true;
            }
        } else {
            // RUNNING：执行追踪逻辑

            if (!has_odometry_) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[WaypointTracker] RUNNING but no odometry received on %s",
                    config_.current_odometry_topic.c_str());
                publish_cmd = true;  // 发零速保持安全
                publish_desired_step = !waypoints_.empty();
            } else {
                const double odometry_age =
                    (now - latest_odometry_stamp_).toSec();
                if (odometry_age > config_.odometry_timeout_s) {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[WaypointTracker] Odometry stale (age=%.3f s > %.3f s), "
                        "holding position.",
                        odometry_age, config_.odometry_timeout_s);
                    ResetLimiters();
                    last_control_time_ = ros::Time();
                    if (!stale_zero_command_sent_) {
                        publish_zero_once = true;
                        stale_zero_command_sent_ = true;
                    }
                    publish_desired_step = !waypoints_.empty();
                } else {
                    stale_zero_command_sent_ = false;

                    // 计算 dt
                    double dt = 0.0;
                    if (!last_control_time_.isZero()) {
                        dt = (now - last_control_time_).toSec();
                    }
                    if (dt < 0.0 || dt > kMaxStepDtSeconds) {
                        ResetLimiters();
                        dt = 0.0;
                    }
                    last_control_time_ = now;

                    // 防御：waypoints_.empty() 已经在状态进入 RUNNING 时被拦截
                    if (waypoints_.empty() ||
                        current_index_ >= waypoints_.size()) {
                        EnterFinishedLocked();
                        publish_cmd = true;
                        publish_desired_step = false;
                    } else {
                        // 计算期望世界系速度并执行限幅
                        double vx_world = 0.0;
                        double vy_world = 0.0;
                        double vz_world = 0.0;
                        double vyaw = 0.0;
                        bool arrived = false;

                        // 提取当前 yaw（用于机体系转换）
                        const double current_yaw = GetYawFromQuaternion(
                            latest_odometry_.pose.pose.orientation);

                        // 限幅器内部对 dt<=0 已做防御（保持 v_cmd 不变）
                        ComputeDesiredWorldVelocities(
                            dt, current_yaw,
                            &vx_world, &vy_world, &vz_world, &vyaw,
                            &arrived);

                        if (arrived) {
                            const std::size_t old_index = current_index_;
                            const bool current_is_stop = waypoints_[current_index_].stop;
                            ++current_index_;

                            if (current_is_stop) {
                                ROS_INFO("[WaypointTracker] Arrived at waypoint %zu/%zu (stop)",
                                         old_index,
                                         waypoints_.empty() ? 0 : (waypoints_.size() - 1));
                            } else {
                                ROS_DEBUG("[WaypointTracker] Arrived at waypoint %zu/%zu (pass-through)",
                                          old_index,
                                          waypoints_.empty() ? 0 : (waypoints_.size() - 1));
                            }

                            if (current_index_ >= waypoints_.size()) {
                                EnterFinishedLocked();
                                publish_cmd = true;
                                publish_desired_step = false;
                            } else if (current_is_stop) {
                                // 停留点到达：走原有流程 - ResetLimiters, PHASE_ROTATE
                                phase_ = Phase::PHASE_ROTATE;
                                ResetLimiters();
                                out_cmd.linear.x = 0.0;
                                out_cmd.linear.y = 0.0;
                                out_cmd.linear.z = 0.0;
                                out_cmd.angular.z = 0.0;
                                publish_cmd = true;
                                publish_desired_step = true;
                            } else {
                                // 穿越点到达：保持 PHASE_MOVE，速度无缝衔接
                                phase_ = Phase::PHASE_MOVE;
                                double vx_body = 0.0;
                                double vy_body = 0.0;
                                TransformWorldFluVelocityToBodyFlu(
                                    vx_world, vy_world, current_yaw,
                                    &vx_body, &vy_body);
                                out_cmd.linear.x = vx_body;
                                out_cmd.linear.y = vy_body;
                                out_cmd.linear.z = vz_world;
                                out_cmd.angular.z = vyaw;
                                publish_cmd = true;
                                publish_desired_step = true;
                            }
                        } else {
                            double vx_body = 0.0;
                            double vy_body = 0.0;
                            TransformWorldFluVelocityToBodyFlu(
                                vx_world, vy_world, current_yaw,
                                &vx_body, &vy_body);
                            out_cmd.linear.x = vx_body;
                            out_cmd.linear.y = vy_body;
                            out_cmd.linear.z = vz_world;
                            out_cmd.angular.z = vyaw;
                            publish_cmd = true;
                            publish_desired_step = true;
                        }
                    }
                }
            }
        }
    }  // unlock

    if (publish_zero_once) {
        PublishZeroVelocityCommand();
    } else if (publish_cmd) {
        velocity_command_publisher_.publish(out_cmd);
    }

    if (publish_desired_step) {
        PublishDesiredOdometryStep();
    }

    if (publish_cmd) {
        ROS_INFO_THROTTLE(
            1.0,
            "[WaypointTracker] cmd_vel body=[%.2f, %.2f, %.2f], yaw_rate=%.2f rad/s",
            out_cmd.linear.x, out_cmd.linear.y, out_cmd.linear.z,
            out_cmd.angular.z);
    }
}

// 计算当前 tick 的期望世界系速度（执行限幅器 Update）
// 在 mutex_ 已锁定的前提下调用
//
// 两阶段流程：
//   PHASE_ROTATE：仅 yaw 用大 P 旋转到目标 yaw，xyz 速度恒为 0；
//                 |yaw_err| ≤ yaw_rotation_tolerance_rad 时切换到 PHASE_MOVE。
//                 切换时不重置 yaw 限幅器，让其残余角速度通过下一阶段的小 P 自然衰减。
//   PHASE_MOVE  ：xy/z 同时收敛到目标位置；yaw 用小 P 保持目标 yaw；
//                 到达判定基于 xy 与 z 容差（ignore_arrival_yaw=false 时还要求 yaw 容差）。
//                 到达后所有速度置零、所有限幅器 Reset、返回 arrived=true。
void WaypointTracker::ComputeDesiredWorldVelocities(double dt,
                                                    double current_yaw_rad,
                                                    double* vx_world,
                                                    double* vy_world,
                                                    double* vz_world,
                                                    double* vyaw,
                                                    bool* arrived) {
    *vx_world = 0.0;
    *vy_world = 0.0;
    *vz_world = 0.0;
    *vyaw = 0.0;
    *arrived = false;

    const Waypoint& target = waypoints_[current_index_];
    const auto& current_pos = latest_odometry_.pose.pose.position;

    const double yaw_err = NormalizeAngleRad(target.yaw_rad - current_yaw_rad);
    const double abs_yaw_err = std::fabs(yaw_err);

    // ---------------- 阶段 1：原地旋转 ----------------
    if (phase_ == Phase::PHASE_ROTATE) {
        if (abs_yaw_err <= config_.yaw_rotation_tolerance_rad) {
            // 已在旋转容差内，直接切到移动阶段，并继续走 PHASE_MOVE 分支
            phase_ = Phase::PHASE_MOVE;
            // 不 return，继续 fall-through 进入 PHASE_MOVE 计算
        } else {
            // 仅旋转 yaw：xy 和 z 速度保持 0
            const double v_stop_yaw =
                std::sqrt(2.0 * config_.yaw_limits.a_max * abs_yaw_err);
            const double v_p_yaw = config_.kp_yaw_rotate * abs_yaw_err;
            const double yaw_speed_mag = std::min(
                {config_.yaw_speed_radps, v_stop_yaw, v_p_yaw});
            const double yaw_speed_target = SignOf(yaw_err) * yaw_speed_mag;

            *vyaw = yaw_limiter_.Update(yaw_speed_target, dt);
            // xy/z 限幅器不更新，保持上次 Reset 之后的 0 状态
            *vx_world = 0.0;
            *vy_world = 0.0;
            *vz_world = 0.0;
            return;
        }
    }

    // ---------------- 阶段 2：xy + z 移动，yaw 保持 ----------------
    const bool is_stop = target.stop;

    // XY — 所有航点逻辑一致
    const double dx = target.x_m - current_pos.x;
    const double dy = target.y_m - current_pos.y;
    const double d_xy = std::sqrt(dx * dx + dy * dy);

    // 停留点：末端减速；穿越点：全程巡航速度
    const double v_stop_xy =
        std::sqrt(2.0 * config_.xy_limits.a_max * std::max(0.0, d_xy));
    const double v_p_xy = config_.kp_xy * d_xy;
    const double xy_speed_target_mag = is_stop
        ? std::min({config_.xy_speed_mps, v_stop_xy, v_p_xy})
        : config_.xy_speed_mps;

    double target_vx_world = 0.0;
    double target_vy_world = 0.0;
    if (d_xy > 1e-6) {
        const double inv_d = 1.0 / d_xy;
        target_vx_world = xy_speed_target_mag * dx * inv_d;
        target_vy_world = xy_speed_target_mag * dy * inv_d;
    }
    *vx_world = xy_vx_limiter_.Update(target_vx_world, dt);
    *vy_world = xy_vy_limiter_.Update(target_vy_world, dt);

    // Z — 停留点与 XY 同步，穿越点仅 P 控制趋势
    const double dz = target.z_m - current_pos.z;
    const double abs_dz = std::fabs(dz);

    double z_speed_target = 0.0;
    if (is_stop) {
        if (d_xy > 1e-6) {
            const double z_speed_synced = xy_speed_target_mag * (dz / d_xy);
            z_speed_target = std::max(-config_.z_speed_mps,
                                      std::min(z_speed_synced, config_.z_speed_mps));
        } else {
            const double v_p_z = config_.kp_z * abs_dz;
            const double z_speed_mag = std::min(config_.z_speed_mps, v_p_z);
            z_speed_target = SignOf(dz) * z_speed_mag;
        }
    } else {
        // 穿越点：独立 P 控制，不依赖 XY 进度
        const double v_p_z = config_.kp_z * abs_dz;
        const double z_speed_mag = std::min(config_.z_speed_mps, v_p_z);
        z_speed_target = SignOf(dz) * z_speed_mag;
    }
    *vz_world = z_limiter_.Update(z_speed_target, dt);

    // Yaw 保持：所有航点一致
    const double v_p_yaw_hold = config_.kp_yaw_hold * abs_yaw_err;
    const double yaw_hold_mag =
        std::min(config_.yaw_speed_radps, v_p_yaw_hold);
    const double yaw_hold_target = SignOf(yaw_err) * yaw_hold_mag;
    *vyaw = yaw_limiter_.Update(yaw_hold_target, dt);

    // 到达判定：停留点用严格容差，穿越点只看 XY 距离
    const double radius = is_stop ? config_.xy_arrival_tolerance_m
                                  : config_.xy_switch_radius_m;
    const bool xy_ok = (d_xy <= radius);
    const bool z_ok = is_stop ? (abs_dz <= config_.z_arrival_tolerance_m) : true;
    const bool yaw_ok = !is_stop || config_.ignore_arrival_yaw ||
                        (abs_yaw_err <= config_.yaw_arrival_tolerance_rad);
    if (xy_ok && z_ok && yaw_ok) {
        if (is_stop) {
            // 停留点：速度置零，限幅器 Reset
            xy_vx_limiter_.Reset();
            xy_vy_limiter_.Reset();
            z_limiter_.Reset();
            yaw_limiter_.Reset();
            *vx_world = 0.0;
            *vy_world = 0.0;
            *vz_world = 0.0;
            *vyaw = 0.0;
        }
        // 穿越点：保留本 tick 已算好的速度，限幅器状态原样带入下一个航点
        *arrived = true;
    }
}

// =============================================================================
// 发布
// =============================================================================

void WaypointTracker::PublishZeroVelocityCommand() {
    geometry_msgs::Twist zero;
    velocity_command_publisher_.publish(zero);
}

void WaypointTracker::PublishVelocityCommandWorldFlu(double vx_world,
                                                     double vy_world,
                                                     double vz_world,
                                                     double vyaw,
                                                     double current_yaw_rad) {
    double vx_body = 0.0;
    double vy_body = 0.0;
    TransformWorldFluVelocityToBodyFlu(vx_world, vy_world, current_yaw_rad,
                                       &vx_body, &vy_body);
    geometry_msgs::Twist cmd;
    cmd.linear.x = vx_body;
    cmd.linear.y = vy_body;
    cmd.linear.z = vz_world;
    cmd.angular.z = vyaw;
    velocity_command_publisher_.publish(cmd);
}

void WaypointTracker::PublishDesiredOdometryStep() {
    if (waypoints_.empty()) {
        return;
    }
    std::size_t idx = current_index_;
    if (idx >= waypoints_.size()) {
        idx = waypoints_.size() - 1;
    }
    const Waypoint& wp = waypoints_[idx];

    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id =
        latest_odometry_.header.frame_id.empty()
            ? "world"
            : latest_odometry_.header.frame_id;
    msg.child_frame_id = latest_odometry_.child_frame_id;
    msg.pose.pose.position.x = wp.x_m;
    msg.pose.pose.position.y = wp.y_m;
    msg.pose.pose.position.z = wp.z_m;

    // 从 yaw 构造仅含 yaw 的四元数
    const double half = wp.yaw_rad * 0.5;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(half);
    msg.pose.pose.orientation.w = std::cos(half);

    desired_odometry_publisher_.publish(msg);
}

// =============================================================================
// 数学工具（与 position_controller.hpp 保持一致）
// =============================================================================

double WaypointTracker::GetYawFromQuaternion(
    const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double WaypointTracker::NormalizeAngleRad(double angle) {
    while (angle <= -kPi) {
        angle += 2.0 * kPi;
    }
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    return angle;
}

void WaypointTracker::TransformWorldFluVelocityToBodyFlu(double vx_world,
                                                          double vy_world,
                                                          double current_yaw_rad,
                                                          double* vx_body,
                                                          double* vy_body) {
    const double cy = std::cos(current_yaw_rad);
    const double sy = std::sin(current_yaw_rad);
    *vx_body = cy * vx_world + sy * vy_world;
    *vy_body = -sy * vx_world + cy * vy_world;
}

}  // namespace indooruav_waypoint
