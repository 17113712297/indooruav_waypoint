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

    // file
    nh_.param<std::string>(p + "waypoints_package_name",
                           config_.waypoints_package_name,
                           std::string("indooruav_waypoint"));
    nh_.param<std::string>(p + "waypoints_file_path",
                           config_.waypoints_file_path,
                           std::string("config/waypoints.yaml"));

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
    nh_.param<double>(p + "parameters/kp_yaw", config_.kp_yaw, 1.5);

    // 中间航点 lookahead 切换半径
    nh_.param<double>(p + "parameters/lookahead_radius_m",
                      config_.lookahead_radius_m, 0.30);

    // 到达阈值（仅末点强制判定）
    nh_.param<double>(p + "parameters/xy_arrival_tolerance_m",
                      config_.xy_arrival_tolerance_m, 0.15);
    nh_.param<double>(p + "parameters/z_arrival_tolerance_m",
                      config_.z_arrival_tolerance_m, 0.15);

    double yaw_tol_deg = 5.0;
    nh_.param<double>(p + "parameters/yaw_arrival_tolerance_deg",
                      yaw_tol_deg, 5.0);
    config_.yaw_arrival_tolerance_rad = DegToRad(yaw_tol_deg);

    // 数值兜底：增益必须非负
    config_.kp_xy = std::max(config_.kp_xy, 0.0);
    config_.kp_z = std::max(config_.kp_z, 0.0);
    config_.kp_yaw = std::max(config_.kp_yaw, 0.0);
    config_.lookahead_radius_m = std::max(config_.lookahead_radius_m, 0.0);

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
    ROS_INFO("[WaypointTracker] arrival tol: xy=%.2fm, z=%.2fm, yaw=%.1fdeg",
             config_.xy_arrival_tolerance_m,
             config_.z_arrival_tolerance_m,
             yaw_tol_deg);
    ROS_INFO("[WaypointTracker] Kp xy=%.2f, z=%.2f, yaw=%.2f (1/s); lookahead=%.2fm",
             config_.kp_xy, config_.kp_z, config_.kp_yaw,
             config_.lookahead_radius_m);
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

bool WaypointTracker::StartServiceCallback(std_srvs::Trigger::Request& req,
                                           std_srvs::Trigger::Response& res) {
    (void)req;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_ || waypoints_.empty()) {
        res.success = false;
        res.message = "Tracker not initialized or waypoint list is empty.";
        return true;
    }

    if (!has_odometry_) {
        res.success = false;
        res.message = "No odometry received yet on " +
                      config_.current_odometry_topic;
        return true;
    }

    switch (state_) {
        case State::RUNNING:
            res.success = true;
            res.message = "Already RUNNING, no change.";
            return true;
        case State::PAUSED:
            EnterRunningLocked();
            res.success = true;
            res.message = "Resumed from PAUSED at index " +
                          std::to_string(current_index_);
            ROS_INFO_STREAM("[WaypointTracker] " << res.message);
            return true;
        case State::FINISHED:
            current_index_ = 0;
            EnterRunningLocked();
            res.success = true;
            res.message = "Restarted from index 0 (was FINISHED).";
            ROS_INFO_STREAM("[WaypointTracker] " << res.message);
            return true;
        case State::IDLE:
        default:
            current_index_ = 0;
            EnterRunningLocked();
            res.success = true;
            res.message = "Started from index 0.";
            ROS_INFO_STREAM("[WaypointTracker] " << res.message);
            return true;
    }
}

bool WaypointTracker::PauseServiceCallback(std_srvs::Trigger::Request& req,
                                           std_srvs::Trigger::Response& res) {
    (void)req;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        res.success = false;
        res.message = "Tracker not initialized.";
        return true;
    }

    if (state_ == State::PAUSED) {
        res.success = true;
        res.message = "Already PAUSED.";
        return true;
    }

    EnterPausedLocked();
    res.success = true;
    res.message = "Paused at index " + std::to_string(current_index_);
    ROS_INFO_STREAM("[WaypointTracker] " << res.message);
    return true;
}

bool WaypointTracker::StopServiceCallback(std_srvs::Trigger::Request& req,
                                          std_srvs::Trigger::Response& res) {
    (void)req;
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        res.success = false;
        res.message = "Tracker not initialized.";
        return true;
    }

    EnterIdleLocked();
    current_index_ = 0;
    res.success = true;
    res.message = "Stopped and reset to index 0.";
    ROS_INFO_STREAM("[WaypointTracker] " << res.message);
    return true;
}

// =============================================================================
// 状态机内部转换（locked 版本，调用方需持有 mutex_）
// =============================================================================

void WaypointTracker::EnterIdleLocked() {
    state_ = State::IDLE;
    ResetLimiters();
    last_control_time_ = ros::Time();
}

void WaypointTracker::EnterRunningLocked() {
    state_ = State::RUNNING;
    // 保留限幅器历史状态以保证速度连续；
    // 但若 last_control_time_ 已陈旧（pause 期间），强制下个 tick 当作 dt=0 起步
    last_control_time_ = ros::Time();
}

void WaypointTracker::EnterPausedLocked() {
    state_ = State::PAUSED;
    ResetLimiters();
    last_control_time_ = ros::Time();
}

void WaypointTracker::EnterFinishedLocked() {
    state_ = State::FINISHED;
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
                            // 到达当前目标航点
                            const std::size_t old_index = current_index_;
                            ++current_index_;
                            ROS_INFO("[WaypointTracker] Arrived at waypoint %zu/%zu",
                                     old_index,
                                     waypoints_.empty() ? 0 : (waypoints_.size() - 1));

                            if (current_index_ >= waypoints_.size()) {
                                EnterFinishedLocked();
                                publish_cmd = true;  // 发零速
                                publish_desired_step = false;
                            } else {
                                // 切到下一个目标，本周期发刚算出的速度
                                // （限幅器状态保留以实现 fly-through）
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

    const bool is_last_waypoint =
        (current_index_ + 1 >= waypoints_.size());

    // ---------------- XY ----------------
    const double dx = target.x_m - current_pos.x;
    const double dy = target.y_m - current_pos.y;
    const double d_xy = std::sqrt(dx * dx + dy * dy);

    // 速度幅值合成：min(v_max, sqrt(2·a·d), Kp·d)
    // 预制动保证物理可停，P 反馈保证近距离自然减速。所有航点都启用预制动。
    const double v_stop_xy =
        std::sqrt(2.0 * config_.xy_limits.a_max * std::max(0.0, d_xy));
    const double v_p_xy = config_.kp_xy * d_xy;
    const double xy_speed_target =
        std::min({config_.xy_speed_mps, v_stop_xy, v_p_xy});

    double target_vx_world = 0.0;
    double target_vy_world = 0.0;
    if (d_xy > 1e-6) {
        const double inv_d = 1.0 / d_xy;
        target_vx_world = xy_speed_target * dx * inv_d;
        target_vy_world = xy_speed_target * dy * inv_d;
    }

    *vx_world = xy_vx_limiter_.Update(target_vx_world, dt);
    *vy_world = xy_vy_limiter_.Update(target_vy_world, dt);

    // ---------------- Z ----------------
    const double dz = target.z_m - current_pos.z;
    const double abs_dz = std::fabs(dz);
    const double v_stop_z =
        std::sqrt(2.0 * config_.z_limits.a_max * abs_dz);
    const double v_p_z = config_.kp_z * abs_dz;
    const double z_speed_mag =
        std::min({config_.z_speed_mps, v_stop_z, v_p_z});
    const double z_speed_target = SignOf(dz) * z_speed_mag;
    *vz_world = z_limiter_.Update(z_speed_target, dt);

    // ---------------- Yaw ----------------
    const double yaw_err = NormalizeAngleRad(target.yaw_rad - current_yaw_rad);
    const double abs_yaw_err = std::fabs(yaw_err);
    const double v_stop_yaw =
        std::sqrt(2.0 * config_.yaw_limits.a_max * abs_yaw_err);
    const double v_p_yaw = config_.kp_yaw * abs_yaw_err;
    const double yaw_speed_mag =
        std::min({config_.yaw_speed_radps, v_stop_yaw, v_p_yaw});
    const double yaw_speed_target = SignOf(yaw_err) * yaw_speed_mag;
    *vyaw = yaw_limiter_.Update(yaw_speed_target, dt);

    // ---------------- 到达判定 ----------------
    // 中间航点：d_xy < lookahead_radius 即切换（fly-through 平滑切弯）
    // 末点：xy/z/yaw 三项严格容差全部命中
    if (is_last_waypoint) {
        *arrived = IsArrivedAtCurrentTarget(d_xy, abs_dz, abs_yaw_err);
    } else {
        *arrived = (d_xy <= config_.lookahead_radius_m);
    }
}

bool WaypointTracker::IsArrivedAtCurrentTarget(
    double xy_err, double z_err, double yaw_err_abs) const {
    return xy_err <= config_.xy_arrival_tolerance_m &&
           z_err <= config_.z_arrival_tolerance_m &&
           yaw_err_abs <= config_.yaw_arrival_tolerance_rad;
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
