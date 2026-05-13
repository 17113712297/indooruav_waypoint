// 加载 YAML 航点文件，订阅实时里程计；
// 通过定时器以预设巡航速度跟踪航点，输出经三阶限幅平滑后的速度指令到 /drone/cmd_vel。
//
// 中间航点采用 fly-through 策略（直通），仅最后一个航点启用预制动 v_stop = sqrt(2 a d)；
// 到达判定要求 XY 距离、Z 距离、yaw 误差三项同时落入阈值内。
//
// 坐标转换严格沿用 position_controller.hpp 的做法：
//   - 误差与限幅在世界系 FLU 中计算
//   - 通过 TransformWorldFluVelocityToBodyFlu 转到机体系 FLU 后下发
//   - yaw 速率本身就是机体系标量

#ifndef INDOORUAV_WAYPOINT_WAYPOINT_TRACKER_HPP_
#define INDOORUAV_WAYPOINT_WAYPOINT_TRACKER_HPP_

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "indooruav_waypoint/smooth_velocity_limiter.hpp"

namespace indooruav_waypoint {

class WaypointTracker {
public:
    WaypointTracker();
    ~WaypointTracker();

    // 不可拷贝
    WaypointTracker(const WaypointTracker&) = delete;
    WaypointTracker& operator=(const WaypointTracker&) = delete;

private:
    // 节点内部状态机
    enum class State {
        IDLE,       // 已初始化但未开始：发零速悬停
        RUNNING,    // 正在追踪航点
        PAUSED,     // 暂停：发零速悬停，保留 current_index_
        FINISHED,   // 已完成最后一个航点：发零速悬停
    };

    // 内存中的航点（位置 + yaw，单位米/弧度）
    struct Waypoint {
        double x_m = 0.0;
        double y_m = 0.0;
        double z_m = 0.0;
        double yaw_rad = 0.0;
    };

    // 配置
    struct Config {
        // topics
        std::string current_odometry_topic;
        std::string desired_odometry_topic;
        std::string velocity_command_topic;

        // services
        std::string start_service_name;
        std::string pause_service_name;
        std::string stop_service_name;

        // file
        std::string waypoints_package_name;
        std::string waypoints_file_path;

        // parameters
        double control_rate_hz = 30.0;
        double odometry_timeout_s = 0.2;
        bool autostart = false;

        // 巡航速度（内部统一 SI）
        double xy_speed_mps = 0.5;
        double z_speed_mps = 0.3;
        double yaw_speed_radps = 0.0;  // 由 dps 转换

        // 限幅参数
        SmoothVelocityLimiter::Limits xy_limits;
        SmoothVelocityLimiter::Limits z_limits;
        SmoothVelocityLimiter::Limits yaw_limits;

        // 到达阈值
        double xy_arrival_tolerance_m = 0.15;
        double z_arrival_tolerance_m = 0.15;
        double yaw_arrival_tolerance_rad = 0.0;  // 由 deg 转换
    };

    // 初始化流程
    void LoadParameters(ros::NodeHandle& pnh);
    bool LoadWaypointsFromYaml();
    bool ResolveWaypointFilePath(std::string* resolved_path) const;
    void SetupRosInterfaces();

    // ROS 回调
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void ControlTimerCallback(const ros::TimerEvent& event);

    bool StartServiceCallback(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res);
    bool PauseServiceCallback(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res);
    bool StopServiceCallback(std_srvs::Trigger::Request& req,
                             std_srvs::Trigger::Response& res);

    // 控制核心
    void RunControlStep(const ros::Time& now);
    void ComputeDesiredWorldVelocities(double dt,
                                       double current_yaw_rad,
                                       double* vx_world,
                                       double* vy_world,
                                       double* vz_world,
                                       double* vyaw,
                                       bool* arrived_at_current_target);

    // 行为辅助
    void PublishZeroVelocityCommand();
    void PublishVelocityCommandWorldFlu(double vx_world,
                                        double vy_world,
                                        double vz_world,
                                        double vyaw,
                                        double current_yaw_rad);
    void PublishDesiredOdometryStep();
    void ResetLimiters();
    void EnterIdleLocked();
    void EnterRunningLocked();
    void EnterPausedLocked();
    void EnterFinishedLocked();
    bool IsArrivedAtCurrentTarget(double xy_err, double z_err, double yaw_err) const;

    // 数学工具（与 position_controller.hpp 保持一致）
    static double GetYawFromQuaternion(const geometry_msgs::Quaternion& q);
    static double NormalizeAngleRad(double angle);
    static void TransformWorldFluVelocityToBodyFlu(double vx_world,
                                                   double vy_world,
                                                   double current_yaw_rad,
                                                   double* vx_body,
                                                   double* vy_body);

    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kMaxStepDtSeconds = 1.0;  // 大于此值视为异常，清零限幅器

    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odometry_subscriber_;
    ros::Publisher velocity_command_publisher_;
    ros::Publisher desired_odometry_publisher_;
    ros::ServiceServer start_service_;
    ros::ServiceServer pause_service_;
    ros::ServiceServer stop_service_;
    ros::Timer control_timer_;

    // 配置
    Config config_;

    // 数据保护
    std::mutex mutex_;

    // 航点
    std::vector<Waypoint> waypoints_;
    std::size_t current_index_ = 0;

    // 实时里程计
    nav_msgs::Odometry latest_odometry_;
    ros::Time latest_odometry_stamp_;
    bool has_odometry_ = false;

    // 状态机
    State state_ = State::IDLE;

    // 限幅器：XY 在世界系下分两个轴跑独立限幅器，
    // 这样航点切换时方向跳变可被限幅器自然处理为平滑过渡（即"切弯"）。
    SmoothVelocityLimiter xy_vx_limiter_;
    SmoothVelocityLimiter xy_vy_limiter_;
    SmoothVelocityLimiter z_limiter_;
    SmoothVelocityLimiter yaw_limiter_;

    // 控制周期管理
    ros::Time last_control_time_;

    // 超时悬停防刷屏标志
    bool stale_zero_command_sent_ = false;

    bool initialized_ = false;
};

}  // namespace indooruav_waypoint

#endif  // INDOORUAV_WAYPOINT_WAYPOINT_TRACKER_HPP_
