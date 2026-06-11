// 加载 YAML 航点文件，订阅实时里程计；
// 通过定时器以预设巡航速度跟踪航点，输出经三阶限幅平滑后的速度指令到 /drone/cmd_vel。
//
// 每个航点采用两阶段流程：
//   PHASE_ROTATE：原地用较大 P 增益把机头转到该航点的目标 yaw（xyz 速度恒为 0）
//   PHASE_MOVE  ：同时收敛 XY 与 Z；yaw 用较小 P 增益保持，避免长距离漂航
// 到达判定基于 XY/Z 容差；yaw 是否参与到达判定由 ignore_arrival_yaw 控制（默认 true）。
// 到达后立即切换下一个航点并重置为 PHASE_ROTATE；若已是最后一个航点则进入 FINISHED 持续发零速。
//
// 坐标转换严格沿用 position_controller.hpp 的做法：
//   - 误差与限幅在世界系 FLU 中计算
//   - 通过 TransformWorldFluVelocityToBodyFlu 转到机体系 FLU 后下发
//   - yaw 速率本身就是机体系标量

#ifndef INDOORUAV_WAYPOINT_WAYPOINT_TRACKER_HPP_
#define INDOORUAV_WAYPOINT_WAYPOINT_TRACKER_HPP_

#include <atomic>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

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

    // 每个航点的两阶段顺序流程（已进入 MOVE 后不会回退到 ROTATE）
    enum class Phase {
        PHASE_ROTATE,  // 阶段1：原地把 yaw 旋转到目标航点 yaw
        PHASE_MOVE,    // 阶段2：xy 与 z 同时移动；yaw 小 P 保持目标 yaw
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
        std::string cruise_complete_service_name;

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

        // 位置 P 反馈增益
        double kp_xy = 2.0;           // 1/s, XY 误差
        double kp_z = 1.5;            // 1/s, Z 误差
        double kp_yaw_rotate = 3.0;   // 1/s, 阶段1（大 P，快速旋转到目标 yaw）
        double kp_yaw_hold = 0.5;     // 1/s, 阶段2（小 P，移动中保持目标 yaw）

        // 到达 / 切换阈值
        double xy_arrival_tolerance_m = 0.15;
        double z_arrival_tolerance_m = 0.15;
        double yaw_rotation_tolerance_rad = 0.0;  // 阶段1→阶段2 阈值（由 deg 转换）
        double yaw_arrival_tolerance_rad = 0.0;   // 到达 yaw 容差（由 deg 转换）
        bool ignore_arrival_yaw = true;           // 到达判定是否忽略 yaw
    };

    // 初始化流程
    void LoadParameters(ros::NodeHandle& pnh);
    bool LoadWaypointsFromYaml();
    bool ResolveWaypointFilePath(std::string* resolved_path) const;
    void SetupRosInterfaces();

    // ROS 回调
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void ControlTimerCallback(const ros::TimerEvent& event);
    void CruiseCompleteTimerCallback(const ros::TimerEvent& event);

    bool StartServiceCallback(std_srvs::Empty::Request& req,
                              std_srvs::Empty::Response& res);
    bool PauseServiceCallback(std_srvs::Empty::Request& req,
                              std_srvs::Empty::Response& res);
    bool StopServiceCallback(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& res);

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
    ros::Timer cruise_complete_timer_;
    ros::ServiceClient cruise_complete_client_;
    std::atomic<bool> cruise_complete_pending_{false};
    int cruise_complete_retry_count_ = 0;
    static constexpr int kMaxCruiseCompleteRetries = 5;

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
    Phase phase_ = Phase::PHASE_ROTATE;

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
