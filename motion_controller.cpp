#include "fangan_core/motion_controller.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "fangan_core/logic.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace fangan_core {

namespace {

enum class RecoveryPhase {
  IDLE,          // 未处于恢复流程。
  STOP,          // 恢复前短暂停车，持续时间单位秒。
  BACKUP,        // 避障/禁区后退，速度单位 m/s。
  LEFT_TURN,     // 后退后左转脱困，角速度单位 rad/s。
  SEARCH_YELLOW  // 黄区内恢复后重新搜索黄色通道。
};

enum class ObstacleBypassPhase {
  IDLE,
  AVOIDING,
  CLEARING,
  RETURNING
};

class MotionControllerNode : public rclcpp::Node {
 public:
  // 构造函数：声明控制参数、订阅感知/任务话题，并创建 /cmd_vel 控制定时器。
  MotionControllerNode() : Node("motion_controller") {
    RCLCPP_INFO(get_logger(), "motion_controller constructor entered");
    // 基础行驶速度参数，单位 m/s。
    declare_parameter<double>("follow_speed", 0.55);
    declare_parameter<double>("approach_speed", 0.20);
    // QR 识别停车参数：hold/wait 单位秒，stop_distance 单位米。
    declare_parameter<double>("qr_capture_hold_seconds", 0.35);
    declare_parameter<double>("qr_stop_distance", 0.60);
    declare_parameter<double>("qr_decode_wait_seconds", 1.20);
    // QR 视觉锁定帧数：连续满足面积条件多少帧后认为已接近二维码。
    declare_parameter<int>("qr_visual_lock_ticks_required", 3);
    // 看见二维码后提前降速/对准：激光对薄立牌常无效(front=10m)，须靠 YOLO 面积与对准。
    declare_parameter<double>("qr_approach_brake_start_area", 10000.0);
    declare_parameter<double>("qr_approach_creep_speed", 0.16);
    declare_parameter<double>("qr_align_start_area", 5000.0);
    declare_parameter<double>("qr_line_blend", 0.35);
    // 黄通道入口开环转向参数：speed 单位 m/s，seconds 单位秒，angular 单位 rad/s。
    declare_parameter<bool>("yellow_entry_turn_left", true);
    // 进黄区 TURN 段与巡线融合权重：0=纯开环转，1=纯跟线(地图为「沿虚线进通道」时建议 0.6~0.9)
    declare_parameter<double>("yellow_entry_turn_line_weight", 0.70);
    declare_parameter<double>("yellow_entry_speed", 0.20);
    declare_parameter<double>("yellow_entry_pause_seconds", 0.30);
    declare_parameter<double>("yellow_entry_turn_seconds", 2.40);
    declare_parameter<double>("yellow_entry_turn_angular", 0.95);
    // 二维码识别后退动作参数：duration 单位秒，speed 单位 m/s。
    declare_parameter<double>("qr_backup_duration", 1.2);
    declare_parameter<double>("qr_backup_speed", 0.10);
    // 阿克曼转向参数：linear_speed 单位 m/s，angular_z 单位 rad/s，duration 单位秒。
    declare_parameter<double>("ackermann_turn_linear_speed", 0.10);
    declare_parameter<double>("turn_angular_z", 0.6);
    declare_parameter<double>("turn_duration", 1.5);
    declare_parameter<double>("qr_turn_linear_speed", 0.06);
    declare_parameter<double>("qr_turn_angular_speed", 0.60);
    declare_parameter<double>("qr_forward_speed", 0.08);
    declare_parameter<double>("qr_forward_small_left_angular", 0.10);
    // 无后退二维码入口：固定左转、定距直行、固定右转，角度由里程计/IMU闭环。
    declare_parameter<double>("turn1_angle_deg", 170.42);
    declare_parameter<double>("entry_advance_distance", 0.834);
    declare_parameter<double>("entry_advance_speed", 0.12);
    declare_parameter<double>("turn2_angle_deg", 82.80);
    declare_parameter<double>("qr_turn_yaw_tolerance_deg", 4.0);
    declare_parameter<double>("qr_turn_settle_seconds", 0.20);
    declare_parameter<double>("qr_turn_calib_linear_x", 0.14);
    declare_parameter<double>("qr_turn_calib_angular_z", 0.72);
    declare_parameter<double>("qr_hardcode_backup_speed", -0.12);
    declare_parameter<double>("qr_hardcode_backup_duration", 6.0);
    declare_parameter<double>("qr_hardcode_turn_speed", 0.08);
    declare_parameter<double>("qr_hardcode_turn_angular_z", 0.55);
    declare_parameter<double>("qr_hardcode_turn_duration", 12.5);
    declare_parameter<double>("qr_hardcode_forward_speed", 0.08);
    declare_parameter<double>("qr_hardcode_forward_duration", 9.0);
    declare_parameter<bool>("qr_hardcode_full_entry", true);
    declare_parameter<double>("qr_hardcode_entry_speed", 0.15);
    declare_parameter<double>("qr_hardcode_entry_right_angular_z", -0.56);
    declare_parameter<double>("qr_hardcode_entry_right1_duration", 2.0);
    declare_parameter<double>("qr_hardcode_entry_straight_duration", 4.0);
    declare_parameter<double>("qr_hardcode_entry_right2_duration", 2.4);
    declare_parameter<double>("qr_hardcode_entry_final_straight_duration", 2.0);
    declare_parameter<double>("qr_hardcode_yellow_grace_duration", 3.0);
    declare_parameter<bool>("qr_hardcode_disable_forbidden_recovery", true);
    declare_parameter<double>("qr_hardcode_yellow_entry_force_ratio", 0.50);
    declare_parameter<double>("qr_hardcode_yellow_lap_grace_duration", 25.0);
    declare_parameter<bool>("debug_stop_after_enter_yellow", true);
    // 从 QR 点转向黄区门口的开环动作参数，角速度单位 rad/s，时间单位秒。
    declare_parameter<double>("qr_backup_stop_time", 0.0);
    declare_parameter<double>("turn_to_gate_angular_z", 0.60);
    declare_parameter<double>("turn_to_gate_duration", 1.5);
    declare_parameter<double>("turn_to_gate_stop_time", 0.0);
    declare_parameter<double>("turn_min_visual_stop_seconds", 1.6);
    declare_parameter<double>("forward_to_gate_speed", 0.14);
    // 黄区入口检测阈值：面积比例为 0~1，pixel_area 单位像素。
    declare_parameter<double>("yellow_gate_threshold", 0.15);
    declare_parameter<double>("yellow_roi_pixel_area", 138240.0);
    declare_parameter<double>("min_forward_to_gate_time", 0.5);
    declare_parameter<double>("forward_to_gate_timeout_seconds", 10.0);
    declare_parameter<int>("yellow_entry_detect_ticks_required", 4);
    declare_parameter<int>("yellow_valid_frames_required", 6);
    // 黄区搜索/入口确认参数：时间单位秒，面积单位像素或面积比例，角速度单位 rad/s。
    declare_parameter<double>("yellow_recovery_search_min_seconds", 0.8);
    declare_parameter<double>("yellow_search_min_seconds", 2.0);
    declare_parameter<double>("yellow_track_min_area_ratio", 0.25);
    declare_parameter<double>("yellow_entry_min_area", 2500.0);
    declare_parameter<double>("yellow_entry_forward_seconds", 4.20);
    declare_parameter<double>("yellow_entry_forward_speed", 0.28);
    declare_parameter<double>("yellow_entry_search_speed", 0.18);
    declare_parameter<double>("yellow_entry_search_angular", 0.55);
    declare_parameter<double>("yellow_entry_track_speed", 0.20);
    declare_parameter<double>("yellow_entry_center_tolerance", 55.0);
    declare_parameter<int>("yellow_entry_line_lock_ticks", 6);
    declare_parameter<double>("yellow_entry_search_timeout_seconds", 3.50);
    declare_parameter<double>("yellow_entry_gate_lock_tolerance", 85.0);
    declare_parameter<int>("yellow_entry_gate_lock_ticks", 4);
    declare_parameter<double>("yellow_entry_approach_max_angular", 0.10);
    declare_parameter<double>("yellow_entry_grid_reject_tolerance", 115.0);
    declare_parameter<double>("yellow_entry_grid_avoid_angular", 0.18);
    declare_parameter<double>("yellow_entry_lane_width_min", 180.0);
    declare_parameter<double>("yellow_entry_grid_max_ratio", 0.03);
    declare_parameter<double>("forward_to_yellow_max_time", 4.0);
    declare_parameter<double>("forward_to_yellow_min_time", 0.5);
    declare_parameter<double>("yellow_enter_area_threshold", 0.18);
    declare_parameter<int>("yellow_enter_confirm_frames", 5);
    declare_parameter<double>("yellow_center_kp", 0.003);
    // 引导线已结束、大厅至黄「通道」之间无引导线段：用定角开环直进(阿克曼为弧)，可微调
    declare_parameter<double>("yellow_entry_blind_angular", 0.0);
    declare_parameter<double>("yellow_loop_speed", 0.24);
    // 黄区环道开环路线参数：速度单位 m/s，角速度单位 rad/s，各段时间单位秒。
    declare_parameter<double>("yellow_loop_turn_speed", 0.18);
    declare_parameter<double>("yellow_loop_turn_angular", 0.72);
    declare_parameter<double>("yellow_loop_entry_seconds", 0.70);
    declare_parameter<double>("yellow_loop_branch_turn_seconds", 1.10);
    declare_parameter<double>("yellow_loop_first_straight_seconds", 1.70);
    declare_parameter<double>("yellow_loop_corner_turn_seconds", 0.95);
    declare_parameter<double>("yellow_loop_side_seconds", 1.80);
    declare_parameter<double>("yellow_loop_top_seconds", 2.10);
    declare_parameter<double>("yellow_loop_second_side_seconds", 1.80);
    declare_parameter<double>("yellow_loop_bottom_return_seconds", 1.70);
    declare_parameter<double>("yellow_loop_exit_turn_seconds", 1.05);
    declare_parameter<double>("yellow_loop_exit_seconds", 1.60);
    declare_parameter<double>("return_speed", 0.45);
    declare_parameter<double>("park_speed", 0.14);
    declare_parameter<double>("line_kp", 0.0035);
    // 巡线/雷达/二维码/P 点对齐比例系数：像素或距离误差转换为角速度。
    declare_parameter<double>("lidar_center_kp", 0.8);
    declare_parameter<double>("qr_align_kp", 0.003);
    declare_parameter<double>("park_align_kp", 0.003);
    declare_parameter<double>("front_stop_distance", 0.35);
    // 前方安全距离：单位米，stop 为急停距离，slow 为降速起点。
    declare_parameter<double>("front_slow_distance", 0.60);
    declare_parameter<double>("qr_stop_area", 68000.0);
    declare_parameter<double>("qr_fallback_area", 22000.0);
    declare_parameter<double>("human_trigger_area", 26000.0);
    declare_parameter<double>("parking_trigger_area", 28000.0);
    declare_parameter<double>("obstacle_trigger_area", 18000.0);
    // 障碍物避让参数：面积单位像素，速度 m/s，角速度 rad/s，时间秒。
    declare_parameter<double>("obstacle_confirm_front_distance", 0.75);
    declare_parameter<double>("obstacle_image_height", 480.0);
    declare_parameter<double>("obstacle_near_bottom_ratio", 0.72);
    declare_parameter<double>("obstacle_path_tolerance", 95.0);
    declare_parameter<double>("obstacle_lidar_side_margin", 0.15);
    declare_parameter<double>("obstacle_avoid_angular", 0.35);
    declare_parameter<double>("obstacle_avoid_speed", 0.10);
    declare_parameter<double>("obstacle_line_weight", 0.15);
    declare_parameter<double>("obstacle_hold_seconds", 0.0);
    declare_parameter<bool>("obstacle_return_enabled", true);
    declare_parameter<double>("obstacle_avoid_min_seconds", 0.45);
    declare_parameter<double>("obstacle_return_seconds", 0.75);
    declare_parameter<double>("obstacle_return_angular", 0.22);
    declare_parameter<double>("obstacle_return_line_weight", 0.85);
    declare_parameter<double>("obstacle_clear_tail_seconds", 0.0);
    declare_parameter<double>("obstacle_clear_tail_angular_scale", 0.35);
    declare_parameter<double>("obstacle_return_force_seconds", 0.90);
    declare_parameter<double>("obstacle_reacquire_error", 80.0);
    declare_parameter<int>("obstacle_reacquire_ticks", 4);
    declare_parameter<double>("obstacle_stop_duration", 0.2);
    declare_parameter<double>("obstacle_backup_duration", 0.7);
    declare_parameter<double>("obstacle_backup_speed", 0.10);
    declare_parameter<double>("obstacle_turn_duration", 1.0);
    declare_parameter<double>("obstacle_turn_speed", 0.35);
    declare_parameter<int>("obstacle_recovery_max_count", 3);
    declare_parameter<int>("obstacle_recovery_detect_frames", 4);
    // 禁入区恢复参数：黄白格/绿色禁区触发后停车、后退、转向，时间秒，速度 m/s。
    declare_parameter<double>("forbidden_stop_duration", 0.2);
    declare_parameter<double>("forbidden_backup_duration", 1.2);
    declare_parameter<double>("forbidden_backup_speed", 0.10);
    declare_parameter<double>("forbidden_turn_duration", 1.0);
    declare_parameter<double>("forbidden_turn_speed", 0.35);
    declare_parameter<double>("green_forbidden_area_ratio", 0.12);
    declare_parameter<double>("checker_forbidden_area_ratio", 0.10);
    declare_parameter<double>("checker_backup_speed", 0.08);
    declare_parameter<double>("checker_backup_duration", 1.0);
    // 白色/黄白格比例阈值：ROI 中白色或格子面积占比，0~1。
    declare_parameter<double>("white_forbidden_roi_start_ratio", 0.60);
    declare_parameter<double>("white_forbidden_threshold", 0.42);
    declare_parameter<double>("white_seen_roi_start_ratio", 0.35);
    declare_parameter<double>("white_seen_roi_end_ratio", 0.95);
    declare_parameter<double>("white_seen_threshold", 0.42);
    declare_parameter<double>("white_danger_roi_start_ratio", 0.88);
    declare_parameter<double>("white_danger_roi_end_ratio", 1.00);
    declare_parameter<double>("white_danger_threshold", 0.70);
    declare_parameter<double>("forced_left_linear_speed", 0.04);
    declare_parameter<double>("forced_left_angular_z", 0.55);
    declare_parameter<double>("forced_left_turn_duration", 6.0);
    declare_parameter<double>("approach_yellow_speed", 0.08);
    declare_parameter<double>("approach_yellow_angular_z", 0.15);
    declare_parameter<double>("approach_yellow_duration", 3.0);
    // 入口近距离危险阈值：白色/格子面积比例，超过后触发禁入恢复。
    declare_parameter<double>("white_emergency_threshold", 0.97);
    declare_parameter<double>("entry_close_grid_threshold", 0.30);
    declare_parameter<double>("entry_close_grid_soft_threshold", 0.18);
    declare_parameter<double>("post_approach_candidate_grace_duration", 1.5);
    declare_parameter<double>("post_approach_candidate_area_threshold", 0.10);
    declare_parameter<double>("post_approach_hard_grid_threshold", 0.45);
    declare_parameter<double>("forbidden_adjust_linear_speed", 0.03);
    declare_parameter<double>("forbidden_adjust_left_angular_z", 0.06);
    declare_parameter<double>("forbidden_adjust_duration", 0.5);
    declare_parameter<double>("yellow_soft_seen_threshold", 0.03);
    declare_parameter<double>("yellow_entry_turn_sign", -1.0);
    declare_parameter<double>("yellow_entry_kp", 0.003);
    declare_parameter<double>("yellow_entry_max_angular_z", 0.18);
    declare_parameter<double>("yellow_entry_area_threshold", 0.18);
    declare_parameter<double>("yellow_entry_total_timeout", 35.0);
    // QR 后禁止原地右打方向：避免在黄白格或白色区域附近向禁区转入。
    declare_parameter<bool>("block_static_right_steer_after_qr", true);
    declare_parameter<double>("static_right_linear_threshold", 0.02);
    declare_parameter<int>("yellow_entry_confirm_frames", 5);
    declare_parameter<double>("line_lost_speed", 0.0);
    declare_parameter<double>("line_lost_search_angular", 0.0);
    declare_parameter<int>("parking_required_ticks", 4);
    declare_parameter<double>("parking_approach_after_seen_seconds", 2.5);
    declare_parameter<double>("parking_approach_speed", 0.10);
    declare_parameter<double>("loop_hold_seconds", 3.5);
    declare_parameter<double>("qr_approach_timeout_seconds", 2.5);
    declare_parameter<double>("return_to_p_seconds", 8.0);
    declare_parameter<double>("image_center_x", 320.0);
    // 雷达/深度安全参数：距离单位米，depth_unit_scale 将深度图单位换算为米。
    declare_parameter<bool>("use_lidar_safety", true);
    declare_parameter<bool>("use_depth_safety", true);
    declare_parameter<double>("depth_stop_distance", 0.35);
    declare_parameter<double>("depth_slow_distance", 0.60);
    declare_parameter<double>("depth_unit_scale", 0.001);
    declare_parameter<int>("depth_roi_width", 96);
    declare_parameter<int>("depth_roi_height", 72);
    // 进黄区：无引导线段时用 /yellow_boundary_center 的黄色质心横坐标闭环（地布分色，非虚线）
    declare_parameter<bool>("use_yellow_color_guidance", true);
    declare_parameter<double>("yellow_boundary_kp", 0.0028);
    // 进黄区：线模型在蓝地仍会持续出点，与真实通道错开。黄区信度够时提高黄色权重(0.7=70% 黄+30% 线)
    declare_parameter<double>("yellow_entry_yellow_line_blend", 0.7);
    declare_parameter<double>("yellow_min_area_for_guidance", 3500.0);
    // 进黄 DRIVE 段：有巡线但尚未满足 HSV 黄(信度)时，巡线满输出会乱拉。按此系数缩小 line_angular(0=完全不跟假线)
    declare_parameter<double>("yellow_entry_line_kp_scale", 0.32);
    declare_parameter<bool>("direct_yellow_entry_mode", false);
    // 直接进黄区锁定参数：面积单位像素，中心容差单位像素，ticks 为连续帧数。
    declare_parameter<double>("direct_yellow_entry_min_area", 9000.0);
    declare_parameter<double>("direct_yellow_entry_center_tolerance", 65.0);
    declare_parameter<int>("direct_yellow_entry_lock_ticks", 8);
    declare_parameter<bool>("yellow_loop_closed_loop", false);
    // 黄区闭环绕行参数：时间秒，速度 m/s，绿色/黄色 Kp 将像素误差转换为角速度。
    declare_parameter<double>("yellow_loop_closed_loop_seconds", 18.0);
    declare_parameter<double>("yellow_loop_closed_loop_speed", 0.20);
    // 刚进入黄通道后先用短开环序列摆正，再交给 HSV 闭环绕圈。
    declare_parameter<double>("yellow_loop_entry_right1_duration", 2.5);
    declare_parameter<double>("yellow_loop_entry_straight_duration", 5.0);
    declare_parameter<double>("yellow_loop_entry_right2_duration", 3.5);
    declare_parameter<double>("yellow_loop_entry_speed", 0.12);
    declare_parameter<double>("yellow_loop_entry_right_angular_z", -0.45);
    // 黄圈转角硬编码：检测到一边绿+对侧黄时，往黄色方向开环转弯，绕过绿区后继续巡线。
    // 看到普通巡线（has_recent_track）即判定为出口。
    declare_parameter<bool>("yellow_corner_hardcode_enabled", true);
    declare_parameter<double>("yellow_corner_green_threshold", 0.06);
    declare_parameter<double>("yellow_corner_yellow_threshold", 0.45);
    declare_parameter<double>("yellow_corner_hc_turn_duration", 1.80);
    declare_parameter<double>("yellow_corner_hc_turn_angular", 0.70);
    declare_parameter<double>("yellow_corner_hc_turn_speed", 0.10);
    declare_parameter<double>("yellow_corner_cooldown_seconds", 4.50);
    declare_parameter<double>("yellow_loop_green_kp", 0.0022);
    declare_parameter<double>("yellow_loop_yellow_kp", 0.0024);
    declare_parameter<double>("yellow_loop_green_target_offset", 135.0);
    declare_parameter<double>("yellow_loop_green_min_area", 1800.0);
    declare_parameter<double>("yellow_loop_yellow_min_area", 2500.0);
    declare_parameter<double>("yellow_loop_fallback_angular", 0.28);
    declare_parameter<double>("yellow_loop_green_weight", 0.75);
    declare_parameter<double>("yellow_loop_green_danger_area", 35000.0);
    declare_parameter<double>("yellow_loop_green_danger_tolerance", 95.0);
    declare_parameter<double>("yellow_loop_green_danger_angular", 0.42);
    declare_parameter<double>("yellow_loop_green_danger_speed_scale", 0.55);
    declare_parameter<double>("yellow_lane_speed", 0.10);
    // 黄区车道跟踪参数：速度 m/s，Kp 为像素误差到角速度系数，max_angular_z 单位 rad/s。
    declare_parameter<double>("yellow_lane_kp", 0.003);
    declare_parameter<double>("yellow_lane_max_angular_z", 0.45);
    // 黄车道中心 Alpha-Beta 滤波器（稳态卡尔曼），消除检测噪声和抖动。
    declare_parameter<double>("yellow_lane_alpha", 0.40);
    declare_parameter<double>("yellow_lane_beta", 0.10);
    declare_parameter<bool>("yellow_corner_boost_enabled", false);
    declare_parameter<double>("yellow_corner_trigger_error", 155.0);
    declare_parameter<double>("yellow_corner_turn_speed", 0.18);
    declare_parameter<double>("yellow_corner_turn_angular", 0.68);
    declare_parameter<double>("yellow_corner_turn_duration", 0.85);
    declare_parameter<double>("yellow_corner_exit_straight_duration", 0.35);
    declare_parameter<double>("yellow_corner_cooldown", 1.6);
    declare_parameter<double>("yellow_exit_corner_inhibit_seconds", 1.2);
    declare_parameter<bool>("yellow_obstacle_allow_green", true);
    declare_parameter<double>("yellow_obstacle_trigger_area", 3500.0);
    declare_parameter<double>("yellow_obstacle_confirm_front_distance", 0.70);
    declare_parameter<double>("yellow_obstacle_path_tolerance", 85.0);
    declare_parameter<double>("yellow_obstacle_avoid_speed", 0.15);
    declare_parameter<double>("yellow_obstacle_avoid_seconds", 0.70);
    declare_parameter<double>("yellow_obstacle_return_seconds", 0.90);
    declare_parameter<double>("yellow_obstacle_return_angular", 0.30);
    declare_parameter<double>("yellow_obstacle_return_line_weight", 0.85);
    declare_parameter<double>("yellow_obstacle_reacquire_error", 100.0);
    declare_parameter<int>("yellow_obstacle_reacquire_ticks", 4);
    declare_parameter<double>("lane_width_min", 60.0);
    declare_parameter<double>("lane_width_max", 500.0);
    declare_parameter<double>("lane_width_estimate_init", 220.0);
    declare_parameter<double>("yellow_min_area_ratio", 0.03);
    declare_parameter<double>("green_forbidden_threshold", 0.08);
    declare_parameter<double>("green_filter_alpha", 0.30);
    declare_parameter<int>("green_danger_confirm_ticks", 4);
    declare_parameter<int>("green_clear_confirm_ticks", 3);
    declare_parameter<double>("yellow_lap_green_recovery_center_ratio", 0.20);
    declare_parameter<double>("yellow_lap_green_recovery_area_ratio", 0.45);
    declare_parameter<double>("yellow_lap_green_recovery_cooldown", 6.0);
    declare_parameter<double>("yellow_lap_green_guard_side_ratio", 0.16);
    declare_parameter<double>("yellow_lap_green_guard_center_ratio", 0.04);
    declare_parameter<double>("yellow_lap_green_guard_strong_side_ratio", 0.30);
    declare_parameter<double>("yellow_lap_green_guard_strong_center_ratio", 0.06);
    declare_parameter<double>("yellow_lap_green_guard_bias_angular", 0.18);
    declare_parameter<double>("yellow_lap_green_guard_force_angular", 0.55);
    declare_parameter<bool>("checker_forbidden_enabled", true);
    declare_parameter<double>("grid_forbidden_threshold", 0.42);
    declare_parameter<double>("yellow_lane_lost_timeout", 0.8);
    declare_parameter<double>("yellow_lane_history_keep_time", 0.5);
    // 黄区车道丢失和格子恢复参数：timeout/duration 单位秒，speed 单位 m/s，angular 单位 rad/s。
    declare_parameter<double>("yellow_lane_safe_speed", 0.04);
    declare_parameter<double>("yellow_lane_escape_angular", 0.35);
    declare_parameter<double>("grid_yellow_ratio_threshold", 0.05);
    declare_parameter<double>("grid_white_ratio_threshold", 0.42);
    declare_parameter<double>("grid_near_roi_start_ratio", 0.60);
    declare_parameter<double>("white_grid_reject_threshold", 0.42);
    declare_parameter<double>("yellow_lane_area_threshold", 0.12);
    declare_parameter<double>("grid_recover_backup_speed", 0.08);
    declare_parameter<double>("grid_recover_backup_duration", 0.6);
    declare_parameter<double>("grid_recover_search_angular_z", 0.25);
    declare_parameter<double>("grid_recover_search_duration", 1.2);
    declare_parameter<int>("grid_recover_max_count", 2);
    declare_parameter<double>("forward_to_yellow_speed", 0.08);
    declare_parameter<double>("yellow_search_linear_speed", 0.04);
    declare_parameter<double>("yellow_search_angular_z", 0.08);
    declare_parameter<double>("yellow_candidate_area_threshold", 0.04);
    // 黄区候选通道前探参数：面积比例阈值，速度 m/s，角速度 rad/s，duration 秒。
    declare_parameter<double>("yellow_candidate_forward_speed", 0.08);
    declare_parameter<double>("yellow_candidate_left_angular_z", 0.03);
    declare_parameter<double>("yellow_candidate_duration", 1.5);
    declare_parameter<bool>("yellow_candidate_allow_right_steer", true);
    declare_parameter<double>("yellow_candidate_max_right_angular_z", 0.12);
    declare_parameter<double>("yellow_candidate_kp", 0.003);
    declare_parameter<double>("forced_approach_gate_speed", 0.10);
    // 强制接近黄区门口参数：速度 m/s，角速度 rad/s，duration 秒。
    declare_parameter<double>("forced_approach_gate_angular_z", 0.05);
    declare_parameter<double>("forced_approach_gate_duration", 4.0);
    declare_parameter<double>("ccw_bias", 0.00);
    declare_parameter<double>("ccw_bias_max", 0.08);
    declare_parameter<double>("yellow_speed", 0.12);
    // 黄区绕圈/出口判断参数：速度 m/s，Kp 转角速度，时间秒，面积比例 0~1。
    declare_parameter<double>("yellow_kp", 0.003);
    declare_parameter<double>("yellow_max_angular_z", 0.5);
    declare_parameter<double>("yellow_lost_threshold", 0.03);
    declare_parameter<double>("yellow_search_speed", 0.04);
    declare_parameter<double>("clockwise_target_ratio", 0.60);
    declare_parameter<double>("counterclockwise_target_ratio", 0.40);
    declare_parameter<double>("min_lap_time", 12.0);
    declare_parameter<double>("expected_lap_time", 90.0);
    declare_parameter<double>("yellow_lap_line_return_min_time", 110.0);
    declare_parameter<int>("yellow_lap_line_return_ticks_required", 10);
    declare_parameter<double>("yellow_exit_green_gap_min_time", 60.0);
    declare_parameter<double>("yellow_exit_green_left_threshold", 0.12);
    declare_parameter<double>("yellow_exit_green_right_clear_threshold", 0.03);
    declare_parameter<double>("yellow_exit_right_yellow_threshold", 0.08);
    declare_parameter<int>("yellow_exit_green_gap_ticks_required", 6);
    declare_parameter<int>("yellow_exit_arm_ticks_required", 8);
    declare_parameter<double>("yellow_exit_arm_inner_green_threshold", 0.08);
    declare_parameter<double>("yellow_exit_side_yellow_min_time", 60.0);
    declare_parameter<double>("yellow_exit_side_yellow_threshold", 0.60);
    declare_parameter<double>("yellow_exit_side_yellow_green_clear", 0.08);
    declare_parameter<double>("yellow_exit_yellow_side_turn_sign", -1.0);
    declare_parameter<double>("exit_gate_yellow_ratio_threshold", 0.06);
    declare_parameter<double>("exit_gate_align_speed", 0.08);
    declare_parameter<double>("exit_gate_align_seconds", 3.0);
    declare_parameter<double>("exit_gate_turn_angular_z", -0.55);
    declare_parameter<double>("exit_gate_visual_timeout_seconds", 12.0);
    declare_parameter<double>("forward_exit_gate_speed", 0.12);
    declare_parameter<double>("min_forward_exit_gate_time", 1.2);
    declare_parameter<double>("back_line_speed", 0.14);
    declare_parameter<double>("min_back_time", 3.0);
    declare_parameter<double>("line_lost_finish_forward_time", 0.4);
    declare_parameter<double>("line_lost_finish_speed", 0.08);

    get_parameter("follow_speed", follow_speed_);
    get_parameter("approach_speed", approach_speed_);
    get_parameter("qr_capture_hold_seconds", qr_capture_hold_seconds_);
    get_parameter("qr_stop_distance", qr_stop_distance_);
    get_parameter("qr_decode_wait_seconds", qr_decode_wait_seconds_);
    get_parameter("qr_visual_lock_ticks_required", qr_visual_lock_ticks_required_);
    get_parameter("qr_approach_brake_start_area", qr_approach_brake_start_area_);
    get_parameter("qr_approach_creep_speed", qr_approach_creep_speed_);
    get_parameter("qr_align_start_area", qr_align_start_area_);
    get_parameter("qr_line_blend", qr_line_blend_);
    get_parameter("yellow_entry_turn_left", yellow_entry_turn_left_);
    get_parameter("yellow_entry_turn_line_weight", yellow_entry_turn_line_weight_);
    get_parameter("yellow_entry_speed", yellow_entry_speed_);
    get_parameter("yellow_entry_pause_seconds", yellow_entry_pause_seconds_);
    get_parameter("yellow_entry_turn_seconds", yellow_entry_turn_seconds_);
    get_parameter("yellow_entry_turn_angular", yellow_entry_turn_angular_);
    get_parameter("qr_backup_duration", qr_backup_duration_);
    get_parameter("qr_backup_speed", qr_backup_speed_);
    get_parameter("ackermann_turn_linear_speed", ackermann_turn_linear_speed_);
    get_parameter("turn_angular_z", turn_angular_z_);
    get_parameter("turn_duration", turn_duration_);
    get_parameter("qr_turn_linear_speed", qr_turn_linear_speed_);
    get_parameter("qr_turn_angular_speed", qr_turn_angular_speed_);
    get_parameter("qr_forward_speed", qr_forward_speed_);
    get_parameter("qr_forward_small_left_angular", qr_forward_small_left_angular_);
    get_parameter("turn1_angle_deg", turn1_angle_deg_);
    get_parameter("entry_advance_distance", entry_advance_distance_);
    get_parameter("entry_advance_speed", entry_advance_speed_);
    get_parameter("turn2_angle_deg", turn2_angle_deg_);
    get_parameter("qr_turn_yaw_tolerance_deg", qr_turn_yaw_tolerance_deg_);
    get_parameter("qr_turn_settle_seconds", qr_turn_settle_seconds_);
    get_parameter("qr_turn_calib_linear_x", qr_turn_calib_linear_x_);
    get_parameter("qr_turn_calib_angular_z", qr_turn_calib_angular_z_);
    get_parameter("qr_hardcode_backup_speed", qr_hardcode_backup_speed_);
    get_parameter("qr_hardcode_backup_duration", qr_hardcode_backup_duration_);
    get_parameter("qr_hardcode_turn_speed", qr_hardcode_turn_speed_);
    get_parameter("qr_hardcode_turn_angular_z", qr_hardcode_turn_angular_z_);
    get_parameter("qr_hardcode_turn_duration", qr_hardcode_turn_duration_);
    get_parameter("qr_hardcode_forward_speed", qr_hardcode_forward_speed_);
    get_parameter("qr_hardcode_forward_duration", qr_hardcode_forward_duration_);
    get_parameter("qr_hardcode_full_entry", qr_hardcode_full_entry_);
    get_parameter("qr_hardcode_entry_speed", qr_hardcode_entry_speed_);
    get_parameter("qr_hardcode_entry_right_angular_z",
                  qr_hardcode_entry_right_angular_z_);
    get_parameter("qr_hardcode_entry_right1_duration",
                  qr_hardcode_entry_right1_duration_);
    get_parameter("qr_hardcode_entry_straight_duration",
                  qr_hardcode_entry_straight_duration_);
    get_parameter("qr_hardcode_entry_right2_duration",
                  qr_hardcode_entry_right2_duration_);
    get_parameter("qr_hardcode_entry_final_straight_duration",
                  qr_hardcode_entry_final_straight_duration_);
    get_parameter("qr_hardcode_yellow_grace_duration",
                  qr_hardcode_yellow_grace_duration_);
    get_parameter("qr_hardcode_disable_forbidden_recovery",
                  qr_hardcode_disable_forbidden_recovery_);
    get_parameter("qr_hardcode_yellow_entry_force_ratio",
                  qr_hardcode_yellow_entry_force_ratio_);
    get_parameter("qr_hardcode_yellow_lap_grace_duration",
                  qr_hardcode_yellow_lap_grace_duration_);
    get_parameter("debug_stop_after_enter_yellow", debug_stop_after_enter_yellow_);
    get_parameter("qr_backup_stop_time", qr_backup_stop_time_);
    get_parameter("turn_to_gate_angular_z", turn_to_gate_angular_z_);
    get_parameter("turn_to_gate_duration", turn_to_gate_duration_);
    get_parameter("turn_to_gate_stop_time", turn_to_gate_stop_time_);
    get_parameter("turn_min_visual_stop_seconds", turn_min_visual_stop_seconds_);
    get_parameter("forward_to_gate_speed", forward_to_gate_speed_);
    get_parameter("yellow_gate_threshold", yellow_gate_threshold_);
    get_parameter("yellow_roi_pixel_area", yellow_roi_pixel_area_);
    get_parameter("min_forward_to_gate_time", min_forward_to_gate_time_);
    get_parameter("forward_to_gate_timeout_seconds", forward_to_gate_timeout_seconds_);
    get_parameter("yellow_entry_detect_ticks_required", yellow_entry_detect_ticks_required_);
    get_parameter("yellow_valid_frames_required", yellow_valid_frames_required_);
    get_parameter("yellow_recovery_search_min_seconds", yellow_recovery_search_min_seconds_);
    get_parameter("yellow_search_min_seconds", yellow_search_min_seconds_);
    get_parameter("yellow_track_min_area_ratio", yellow_track_min_area_ratio_);
    get_parameter("yellow_entry_min_area", yellow_entry_min_area_);
    get_parameter("yellow_entry_forward_seconds", yellow_entry_forward_seconds_);
    get_parameter("yellow_entry_forward_speed", yellow_entry_forward_speed_);
    get_parameter("yellow_entry_search_speed", yellow_entry_search_speed_);
    get_parameter("yellow_entry_search_angular", yellow_entry_search_angular_);
    get_parameter("yellow_entry_track_speed", yellow_entry_track_speed_);
    get_parameter("yellow_entry_center_tolerance", yellow_entry_center_tolerance_);
    get_parameter("yellow_entry_line_lock_ticks", yellow_entry_line_lock_ticks_);
    get_parameter("yellow_entry_search_timeout_seconds", yellow_entry_search_timeout_seconds_);
    get_parameter("yellow_entry_gate_lock_tolerance", yellow_entry_gate_lock_tolerance_);
    get_parameter("yellow_entry_gate_lock_ticks", yellow_entry_gate_lock_ticks_);
    get_parameter("yellow_entry_approach_max_angular", yellow_entry_approach_max_angular_);
    get_parameter("yellow_entry_grid_reject_tolerance", yellow_entry_grid_reject_tolerance_);
    get_parameter("yellow_entry_grid_avoid_angular", yellow_entry_grid_avoid_angular_);
    get_parameter("yellow_entry_lane_width_min", yellow_entry_lane_width_min_);
    get_parameter("yellow_entry_grid_max_ratio", yellow_entry_grid_max_ratio_);
    get_parameter("forward_to_yellow_max_time", forward_to_yellow_max_time_);
    get_parameter("forward_to_yellow_min_time", forward_to_yellow_min_time_);
    get_parameter("yellow_enter_area_threshold", yellow_enter_area_threshold_);
    get_parameter("yellow_enter_confirm_frames", yellow_enter_confirm_frames_);
    get_parameter("yellow_center_kp", yellow_center_kp_);
    get_parameter("yellow_entry_blind_angular", yellow_entry_blind_angular_);
    get_parameter("yellow_loop_speed", yellow_loop_speed_);
    get_parameter("yellow_loop_turn_speed", yellow_loop_turn_speed_);
    get_parameter("yellow_loop_turn_angular", yellow_loop_turn_angular_);
    get_parameter("yellow_loop_entry_seconds", yellow_loop_entry_seconds_);
    get_parameter("yellow_loop_branch_turn_seconds",
                  yellow_loop_branch_turn_seconds_);
    get_parameter("yellow_loop_first_straight_seconds",
                  yellow_loop_first_straight_seconds_);
    get_parameter("yellow_loop_corner_turn_seconds",
                  yellow_loop_corner_turn_seconds_);
    get_parameter("yellow_loop_side_seconds", yellow_loop_side_seconds_);
    get_parameter("yellow_loop_top_seconds", yellow_loop_top_seconds_);
    get_parameter("yellow_loop_second_side_seconds",
                  yellow_loop_second_side_seconds_);
    get_parameter("yellow_loop_bottom_return_seconds",
                  yellow_loop_bottom_return_seconds_);
    get_parameter("yellow_loop_exit_turn_seconds",
                  yellow_loop_exit_turn_seconds_);
    get_parameter("yellow_loop_exit_seconds", yellow_loop_exit_seconds_);
    get_parameter("return_speed", return_speed_);
    get_parameter("park_speed", park_speed_);
    get_parameter("line_kp", line_kp_);
    get_parameter("lidar_center_kp", lidar_center_kp_);
    get_parameter("qr_align_kp", qr_align_kp_);
    get_parameter("park_align_kp", park_align_kp_);
    get_parameter("front_stop_distance", front_stop_distance_);
    get_parameter("front_slow_distance", front_slow_distance_);
    get_parameter("qr_stop_area", qr_stop_area_);
    get_parameter("qr_fallback_area", qr_fallback_area_);
    get_parameter("human_trigger_area", human_trigger_area_);
    get_parameter("parking_trigger_area", parking_trigger_area_);
    get_parameter("obstacle_trigger_area", obstacle_trigger_area_);
    get_parameter("obstacle_confirm_front_distance",
                  obstacle_confirm_front_distance_);
    get_parameter("obstacle_image_height", obstacle_image_height_);
    get_parameter("obstacle_near_bottom_ratio", obstacle_near_bottom_ratio_);
    get_parameter("obstacle_path_tolerance", obstacle_path_tolerance_);
    get_parameter("obstacle_lidar_side_margin", obstacle_lidar_side_margin_);
    get_parameter("obstacle_avoid_angular", obstacle_avoid_angular_);
    get_parameter("obstacle_avoid_speed", obstacle_avoid_speed_);
    get_parameter("obstacle_line_weight", obstacle_line_weight_);
    get_parameter("obstacle_hold_seconds", obstacle_hold_seconds_);
    get_parameter("obstacle_return_enabled", obstacle_return_enabled_);
    get_parameter("obstacle_avoid_min_seconds", obstacle_avoid_min_seconds_);
    get_parameter("obstacle_return_seconds", obstacle_return_seconds_);
    get_parameter("obstacle_return_angular", obstacle_return_angular_);
    get_parameter("obstacle_return_line_weight", obstacle_return_line_weight_);
    get_parameter("obstacle_clear_tail_seconds", obstacle_clear_tail_seconds_);
    get_parameter("obstacle_clear_tail_angular_scale",
                  obstacle_clear_tail_angular_scale_);
    get_parameter("obstacle_return_force_seconds",
                  obstacle_return_force_seconds_);
    get_parameter("obstacle_reacquire_error", obstacle_reacquire_error_);
    get_parameter("obstacle_reacquire_ticks", obstacle_reacquire_ticks_);
    get_parameter("obstacle_stop_duration", obstacle_stop_duration_);
    get_parameter("obstacle_backup_duration", obstacle_backup_duration_);
    get_parameter("obstacle_backup_speed", obstacle_backup_speed_);
    get_parameter("obstacle_turn_duration", obstacle_turn_duration_);
    get_parameter("obstacle_turn_speed", obstacle_turn_speed_);
    get_parameter("obstacle_recovery_max_count", obstacle_recovery_max_count_);
    get_parameter("obstacle_recovery_detect_frames", obstacle_recovery_detect_frames_);
    get_parameter("forbidden_stop_duration", forbidden_stop_duration_);
    get_parameter("forbidden_backup_duration", forbidden_backup_duration_);
    get_parameter("forbidden_backup_speed", forbidden_backup_speed_);
    get_parameter("forbidden_turn_duration", forbidden_turn_duration_);
    get_parameter("forbidden_turn_speed", forbidden_turn_speed_);
    get_parameter("green_forbidden_area_ratio", green_forbidden_area_ratio_);
    get_parameter("checker_forbidden_area_ratio", checker_forbidden_area_ratio_);
    get_parameter("checker_backup_speed", checker_backup_speed_);
    get_parameter("checker_backup_duration", checker_backup_duration_);
    get_parameter("white_forbidden_roi_start_ratio", white_forbidden_roi_start_ratio_);
    get_parameter("white_forbidden_threshold", white_forbidden_threshold_);
    get_parameter("white_seen_roi_start_ratio", white_seen_roi_start_ratio_);
    get_parameter("white_seen_roi_end_ratio", white_seen_roi_end_ratio_);
    get_parameter("white_seen_threshold", white_seen_threshold_);
    get_parameter("white_danger_roi_start_ratio", white_danger_roi_start_ratio_);
    get_parameter("white_danger_roi_end_ratio", white_danger_roi_end_ratio_);
    get_parameter("white_danger_threshold", white_danger_threshold_);
    get_parameter("forced_left_linear_speed", forced_left_linear_speed_);
    get_parameter("forced_left_angular_z", forced_left_angular_z_);
    get_parameter("forced_left_turn_duration", forced_left_turn_duration_);
    get_parameter("approach_yellow_speed", approach_yellow_speed_);
    get_parameter("approach_yellow_angular_z", approach_yellow_angular_z_);
    get_parameter("approach_yellow_duration", approach_yellow_duration_);
    get_parameter("white_emergency_threshold", white_emergency_threshold_);
    get_parameter("entry_close_grid_threshold", entry_close_grid_threshold_);
    get_parameter("entry_close_grid_soft_threshold",
                  entry_close_grid_soft_threshold_);
    get_parameter("post_approach_candidate_grace_duration",
                  post_approach_candidate_grace_duration_);
    get_parameter("post_approach_candidate_area_threshold",
                  post_approach_candidate_area_threshold_);
    get_parameter("post_approach_hard_grid_threshold",
                  post_approach_hard_grid_threshold_);
    get_parameter("forbidden_adjust_linear_speed", forbidden_adjust_linear_speed_);
    get_parameter("forbidden_adjust_left_angular_z", forbidden_adjust_left_angular_z_);
    get_parameter("forbidden_adjust_duration", forbidden_adjust_duration_);
    get_parameter("yellow_soft_seen_threshold", yellow_soft_seen_threshold_);
    get_parameter("yellow_entry_turn_sign", yellow_entry_turn_sign_);
    get_parameter("yellow_entry_kp", yellow_entry_kp_);
    get_parameter("yellow_entry_max_angular_z", yellow_entry_max_angular_z_);
    get_parameter("yellow_entry_area_threshold", yellow_entry_area_threshold_);
    get_parameter("yellow_entry_total_timeout", yellow_entry_total_timeout_);
    get_parameter("block_static_right_steer_after_qr", block_static_right_steer_after_qr_);
    get_parameter("static_right_linear_threshold", static_right_linear_threshold_);
    get_parameter("yellow_entry_confirm_frames", yellow_entry_confirm_frames_);
    get_parameter("line_lost_speed", line_lost_speed_);
    get_parameter("line_lost_search_angular", line_lost_search_angular_);
    get_parameter("parking_required_ticks", parking_required_ticks_);
    get_parameter("parking_approach_after_seen_seconds",
                  parking_approach_after_seen_seconds_);
    get_parameter("parking_approach_speed", parking_approach_speed_);
    get_parameter("loop_hold_seconds", loop_hold_seconds_);
    get_parameter("qr_approach_timeout_seconds", qr_approach_timeout_seconds_);
    get_parameter("return_to_p_seconds", return_to_p_seconds_);
    get_parameter("image_center_x", image_center_x_);
    get_parameter("use_lidar_safety", use_lidar_safety_);
    get_parameter("use_depth_safety", use_depth_safety_);
    get_parameter("depth_stop_distance", depth_stop_distance_);
    get_parameter("depth_slow_distance", depth_slow_distance_);
    get_parameter("depth_unit_scale", depth_unit_scale_);
    get_parameter("depth_roi_width", depth_roi_width_);
    get_parameter("depth_roi_height", depth_roi_height_);
    get_parameter("use_yellow_color_guidance", use_yellow_color_guidance_);
    get_parameter("yellow_boundary_kp", yellow_boundary_kp_);
    get_parameter("yellow_entry_yellow_line_blend", yellow_entry_yellow_line_blend_);
    get_parameter("yellow_min_area_for_guidance", yellow_min_area_for_guidance_);
    get_parameter("yellow_entry_line_kp_scale", yellow_entry_line_kp_scale_);
    get_parameter("direct_yellow_entry_mode", direct_yellow_entry_mode_);
    get_parameter("direct_yellow_entry_min_area", direct_yellow_entry_min_area_);
    get_parameter("direct_yellow_entry_center_tolerance", direct_yellow_entry_center_tolerance_);
    get_parameter("direct_yellow_entry_lock_ticks", direct_yellow_entry_lock_ticks_);
    get_parameter("yellow_loop_closed_loop", yellow_loop_closed_loop_);
    get_parameter("yellow_loop_closed_loop_seconds", yellow_loop_closed_loop_seconds_);
    get_parameter("yellow_loop_closed_loop_speed", yellow_loop_closed_loop_speed_);
    get_parameter("yellow_loop_entry_right1_duration",
                  yellow_loop_entry_right1_duration_);
    get_parameter("yellow_loop_entry_straight_duration",
                  yellow_loop_entry_straight_duration_);
    get_parameter("yellow_loop_entry_right2_duration",
                  yellow_loop_entry_right2_duration_);
    get_parameter("yellow_loop_entry_speed",
                  yellow_loop_entry_speed_);
    get_parameter("yellow_loop_entry_right_angular_z",
                  yellow_loop_entry_right_angular_z_);
    get_parameter("yellow_corner_hardcode_enabled", yellow_corner_hardcode_enabled_);
    get_parameter("yellow_corner_green_threshold", yellow_corner_green_threshold_);
    get_parameter("yellow_corner_yellow_threshold", yellow_corner_yellow_threshold_);
    get_parameter("yellow_corner_hc_turn_duration", yellow_corner_hc_turn_duration_);
    get_parameter("yellow_corner_hc_turn_angular", yellow_corner_hc_turn_angular_);
    get_parameter("yellow_corner_hc_turn_speed", yellow_corner_hc_turn_speed_);
    get_parameter("yellow_corner_cooldown_seconds", yellow_corner_cooldown_seconds_);
    get_parameter("yellow_loop_green_kp", yellow_loop_green_kp_);
    get_parameter("yellow_loop_yellow_kp", yellow_loop_yellow_kp_);
    get_parameter("yellow_loop_green_target_offset", yellow_loop_green_target_offset_);
    get_parameter("yellow_loop_green_min_area", yellow_loop_green_min_area_);
    get_parameter("yellow_loop_yellow_min_area", yellow_loop_yellow_min_area_);
    get_parameter("yellow_loop_fallback_angular", yellow_loop_fallback_angular_);
    get_parameter("yellow_loop_green_weight", yellow_loop_green_weight_);
    get_parameter("yellow_loop_green_danger_area", yellow_loop_green_danger_area_);
    get_parameter("yellow_loop_green_danger_tolerance", yellow_loop_green_danger_tolerance_);
    get_parameter("yellow_loop_green_danger_angular", yellow_loop_green_danger_angular_);
    get_parameter("yellow_loop_green_danger_speed_scale",
                  yellow_loop_green_danger_speed_scale_);
    get_parameter("yellow_lane_speed", yellow_lane_speed_);
    get_parameter("yellow_lane_kp", yellow_lane_kp_);
    get_parameter("yellow_lane_max_angular_z", yellow_lane_max_angular_z_);
    get_parameter("yellow_lane_alpha", yellow_lane_alpha_);
    get_parameter("yellow_lane_beta", yellow_lane_beta_);
    get_parameter("yellow_corner_boost_enabled", yellow_corner_boost_enabled_);
    get_parameter("yellow_corner_trigger_error", yellow_corner_trigger_error_);
    get_parameter("yellow_corner_turn_speed", yellow_corner_turn_speed_);
    get_parameter("yellow_corner_turn_angular", yellow_corner_turn_angular_);
    get_parameter("yellow_corner_turn_duration", yellow_corner_turn_duration_);
    get_parameter("yellow_corner_exit_straight_duration",
                  yellow_corner_exit_straight_duration_);
    get_parameter("yellow_corner_cooldown", yellow_corner_cooldown_);
    get_parameter("yellow_exit_corner_inhibit_seconds",
                  yellow_exit_corner_inhibit_seconds_);
    get_parameter("yellow_obstacle_allow_green", yellow_obstacle_allow_green_);
    get_parameter("yellow_obstacle_trigger_area", yellow_obstacle_trigger_area_);
    get_parameter("yellow_obstacle_confirm_front_distance",
                  yellow_obstacle_confirm_front_distance_);
    get_parameter("yellow_obstacle_path_tolerance",
                  yellow_obstacle_path_tolerance_);
    get_parameter("yellow_obstacle_avoid_speed", yellow_obstacle_avoid_speed_);
    get_parameter("yellow_obstacle_avoid_seconds", yellow_obstacle_avoid_seconds_);
    get_parameter("yellow_obstacle_return_seconds",
                  yellow_obstacle_return_seconds_);
    get_parameter("yellow_obstacle_return_angular",
                  yellow_obstacle_return_angular_);
    get_parameter("yellow_obstacle_return_line_weight",
                  yellow_obstacle_return_line_weight_);
    get_parameter("yellow_obstacle_reacquire_error",
                  yellow_obstacle_reacquire_error_);
    get_parameter("yellow_obstacle_reacquire_ticks",
                  yellow_obstacle_reacquire_ticks_);
    get_parameter("lane_width_min", lane_width_min_);
    get_parameter("lane_width_max", lane_width_max_);
    get_parameter("lane_width_estimate_init", lane_width_estimate_init_);
    get_parameter("yellow_min_area_ratio", yellow_min_area_ratio_);
    get_parameter("green_forbidden_threshold", green_forbidden_threshold_);
    get_parameter("green_filter_alpha", green_filter_alpha_);
    get_parameter("green_danger_confirm_ticks", green_danger_confirm_ticks_);
    get_parameter("green_clear_confirm_ticks", green_clear_confirm_ticks_);
    get_parameter("yellow_lap_green_recovery_center_ratio",
                  yellow_lap_green_recovery_center_ratio_);
    get_parameter("yellow_lap_green_recovery_area_ratio",
                  yellow_lap_green_recovery_area_ratio_);
    get_parameter("yellow_lap_green_recovery_cooldown",
                  yellow_lap_green_recovery_cooldown_);
    get_parameter("yellow_lap_green_guard_side_ratio",
                  yellow_lap_green_guard_side_ratio_);
    get_parameter("yellow_lap_green_guard_center_ratio",
                  yellow_lap_green_guard_center_ratio_);
    get_parameter("yellow_lap_green_guard_strong_side_ratio",
                  yellow_lap_green_guard_strong_side_ratio_);
    get_parameter("yellow_lap_green_guard_strong_center_ratio",
                  yellow_lap_green_guard_strong_center_ratio_);
    get_parameter("yellow_lap_green_guard_bias_angular",
                  yellow_lap_green_guard_bias_angular_);
    get_parameter("yellow_lap_green_guard_force_angular",
                  yellow_lap_green_guard_force_angular_);
    get_parameter("checker_forbidden_enabled", checker_forbidden_enabled_);
    get_parameter("grid_forbidden_threshold", grid_forbidden_threshold_);
    get_parameter("yellow_lane_lost_timeout", yellow_lane_lost_timeout_);
    get_parameter("yellow_lane_history_keep_time", yellow_lane_history_keep_time_);
    get_parameter("yellow_lane_safe_speed", yellow_lane_safe_speed_);
    get_parameter("yellow_lane_escape_angular", yellow_lane_escape_angular_);
    get_parameter("grid_yellow_ratio_threshold", grid_yellow_ratio_threshold_);
    get_parameter("grid_white_ratio_threshold", grid_white_ratio_threshold_);
    get_parameter("grid_near_roi_start_ratio", grid_near_roi_start_ratio_);
    get_parameter("white_grid_reject_threshold", white_grid_reject_threshold_);
    get_parameter("yellow_lane_area_threshold", yellow_lane_area_threshold_);
    get_parameter("grid_recover_backup_speed", grid_recover_backup_speed_);
    get_parameter("grid_recover_backup_duration", grid_recover_backup_duration_);
    get_parameter("grid_recover_search_angular_z", grid_recover_search_angular_z_);
    get_parameter("grid_recover_search_duration", grid_recover_search_duration_);
    get_parameter("grid_recover_max_count", grid_recover_max_count_);
    get_parameter("forward_to_yellow_speed", forward_to_yellow_speed_);
    get_parameter("yellow_search_linear_speed", yellow_search_linear_speed_);
    get_parameter("yellow_search_angular_z", yellow_search_angular_z_);
    get_parameter("yellow_candidate_area_threshold", yellow_candidate_area_threshold_);
    get_parameter("yellow_candidate_forward_speed", yellow_candidate_forward_speed_);
    get_parameter("yellow_candidate_left_angular_z", yellow_candidate_left_angular_z_);
    get_parameter("yellow_candidate_duration", yellow_candidate_duration_);
    get_parameter("yellow_candidate_allow_right_steer", yellow_candidate_allow_right_steer_);
    get_parameter("yellow_candidate_max_right_angular_z", yellow_candidate_max_right_angular_z_);
    get_parameter("yellow_candidate_kp", yellow_candidate_kp_);
    get_parameter("forced_approach_gate_speed", forced_approach_gate_speed_);
    get_parameter("forced_approach_gate_angular_z", forced_approach_gate_angular_z_);
    get_parameter("forced_approach_gate_duration", forced_approach_gate_duration_);
    get_parameter("ccw_bias", ccw_bias_);
    get_parameter("ccw_bias_max", ccw_bias_max_);
    lane_width_estimate_ = std::clamp(lane_width_estimate_init_, lane_width_min_, lane_width_max_);
    get_parameter("yellow_speed", yellow_speed_);
    get_parameter("yellow_kp", yellow_kp_);
    get_parameter("yellow_max_angular_z", yellow_max_angular_z_);
    get_parameter("yellow_lost_threshold", yellow_lost_threshold_);
    get_parameter("yellow_search_speed", yellow_search_speed_);
    get_parameter("clockwise_target_ratio", clockwise_target_ratio_);
    get_parameter("counterclockwise_target_ratio", counterclockwise_target_ratio_);
    get_parameter("min_lap_time", min_lap_time_);
    get_parameter("expected_lap_time", expected_lap_time_);
    get_parameter("yellow_lap_line_return_min_time",
                  yellow_lap_line_return_min_time_);
    get_parameter("yellow_lap_line_return_ticks_required",
                  yellow_lap_line_return_ticks_required_);
    get_parameter("yellow_exit_green_gap_min_time",
                  yellow_exit_green_gap_min_time_);
    get_parameter("yellow_exit_green_left_threshold",
                  yellow_exit_green_left_threshold_);
    get_parameter("yellow_exit_green_right_clear_threshold",
                  yellow_exit_green_right_clear_threshold_);
    get_parameter("yellow_exit_right_yellow_threshold",
                  yellow_exit_right_yellow_threshold_);
    get_parameter("yellow_exit_green_gap_ticks_required",
                  yellow_exit_green_gap_ticks_required_);
    get_parameter("yellow_exit_arm_ticks_required",
                  yellow_exit_arm_ticks_required_);
    get_parameter("yellow_exit_arm_inner_green_threshold",
                  yellow_exit_arm_inner_green_threshold_);
    get_parameter("yellow_exit_side_yellow_min_time",
                  yellow_exit_side_yellow_min_time_);
    get_parameter("yellow_exit_side_yellow_threshold",
                  yellow_exit_side_yellow_threshold_);
    get_parameter("yellow_exit_side_yellow_green_clear",
                  yellow_exit_side_yellow_green_clear_);
    get_parameter("yellow_exit_yellow_side_turn_sign",
                  yellow_exit_yellow_side_turn_sign_);
    get_parameter("exit_gate_yellow_ratio_threshold", exit_gate_yellow_ratio_threshold_);
    get_parameter("exit_gate_align_speed", exit_gate_align_speed_);
    get_parameter("exit_gate_align_seconds", exit_gate_align_seconds_);
    get_parameter("exit_gate_turn_angular_z", exit_gate_turn_angular_z_);
    get_parameter("exit_gate_visual_timeout_seconds", exit_gate_visual_timeout_seconds_);
    get_parameter("forward_exit_gate_speed", forward_exit_gate_speed_);
    get_parameter("min_forward_exit_gate_time", min_forward_exit_gate_time_);
    get_parameter("back_line_speed", back_line_speed_);
    get_parameter("min_back_time", min_back_time_);
    get_parameter("line_lost_finish_forward_time", line_lost_finish_forward_time_);
    get_parameter("line_lost_finish_speed", line_lost_finish_speed_);

    stage_sub_ = create_subscription<std_msgs::msg::String>(
        "/race/stage", 10,
        [this](const std_msgs::msg::String::ConstSharedPtr msg) {
          if (stage_ != msg->data) {
            RCLCPP_INFO(get_logger(), "stage update: %s -> %s",
                        stage_.c_str(), msg->data.c_str());
            if (stage_ == "QR_HARDCODE_ENTRY" &&
                (msg->data == "YELLOW_SEARCH" || msg->data == "YELLOW_LAP")) {
              qr_hardcode_yellow_path_active_ = true;
            } else if (msg->data != "YELLOW_SEARCH" && msg->data != "YELLOW_LAP") {
              qr_hardcode_yellow_path_active_ = false;
            }
            previous_stage_ = stage_;
            stage_enter_time_ = now();
            reset_stage_latches();
            if (msg->data == "LINE_TO_QR" || msg->data == "FOLLOW_TO_QR") {
              qr_text_locked_ = false;
              qr_text_.clear();
            }
            RCLCPP_INFO(get_logger(), "[MOTION] stage=%s", msg->data.c_str());
          }
          stage_ = msg->data;
        });

    direction_sub_ = create_subscription<std_msgs::msg::String>(
        "/race/direction", 10,
        [this](const std_msgs::msg::String::ConstSharedPtr msg) { direction_ = msg->data; });

    qr_text_sub_ = create_subscription<std_msgs::msg::String>(
        "/qr_code_result", 10,
        [this](const std_msgs::msg::String::ConstSharedPtr msg) {
          qr_text_locked_ = !msg->data.empty();
          qr_text_ = msg->data;
          if (qr_text_locked_) {
            RCLCPP_INFO(get_logger(), "[QR] decoded text='%s'", qr_text_.c_str());
          }
        });

    track_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/racing_track_center_point", 10,
        [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          latest_track_x_ = msg->point.x;
          has_track_ = true;
          last_track_time_ = now();
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "track update: x=%.2f, y=%.2f", latest_track_x_, msg->point.y);
        });

    detect_sub_ = create_subscription<std_msgs::msg::String>(
        "/race/detections", 10,
        [this](const std_msgs::msg::String::ConstSharedPtr msg) { consume_simple_detections(msg->data); });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { consume_scan(*msg); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 20,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          update_yaw_from_quaternion(msg->pose.pose.orientation.x,
                                     msg->pose.pose.orientation.y,
                                     msg->pose.pose.orientation.z,
                                     msg->pose.pose.orientation.w,
                                     "odom");
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          update_yaw_from_quaternion(msg->orientation.x, msg->orientation.y,
                                     msg->orientation.z, msg->orientation.w,
                                     "imu");
        });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/aurora/depth/image_raw", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { consume_depth(*msg); });
    // 地布黄区 HSV 质心(包 yellow_boundary) — 无引导线段时对准通道
    yellow_boundary_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/yellow_boundary_center", 10,
        [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          latest_yellow_x_ = msg->point.x;
          latest_yellow_area_ = msg->point.y;
          has_yellow_ = true;
          last_yellow_time_ = now();
        });
    green_boundary_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/green_boundary_center", 10,
        [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          latest_green_x_ = msg->point.x;
          latest_green_area_ = msg->point.y;
          has_green_ = true;
          last_green_time_ = now();
        });
    yellow_lane_info_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/yellow_lane_info", 10,
        [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          consume_yellow_lane_info(*msg);
        });
    hardcode_override_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/hardcode_override_active", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          // override=true：二维码后硬编码节点临时接管控制权，motion_controller 暂停发布 /cmd_vel。
          // override=false：硬编码结束，交还原 HSV/黄绿间隔视觉主控继续发布 /cmd_vel。
          hardcode_override_active_ = msg->data;
        });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    qr_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/race/qr_reached", 10);
    qr_hardcode_done_pub_ =
        create_publisher<std_msgs::msg::Bool>("/race/qr_hardcode_done", 10);
    gate_aligned_pub_ = create_publisher<std_msgs::msg::Bool>("/race/gate_aligned", 10);
    yellow_entry_pub_ = create_publisher<std_msgs::msg::Bool>("/race/yellow_entry_reached", 10);
    loop_complete_pub_ = create_publisher<std_msgs::msg::Bool>("/race/loop_complete", 10);
    park_zone_pub_ = create_publisher<std_msgs::msg::Bool>("/race/park_zone_reached", 10);
    parked_pub_ = create_publisher<std_msgs::msg::Bool>("/race/parked", 10);
    exit_gate_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/race/exit_gate_ready", 10);
    line_reacquired_pub_ = create_publisher<std_msgs::msg::Bool>("/race/line_reacquired", 10);
    return_complete_pub_ = create_publisher<std_msgs::msg::Bool>("/race/return_complete", 10);
    human_result_pub_ = create_publisher<std_msgs::msg::String>("/human/result", 10);

    control_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { step(); });
    RCLCPP_INFO(get_logger(), "motion_controller ready");
  }

 private:
  // 处理激光雷达扫描：取正前、左前、右前三个方向距离，单位米。
  void consume_scan(const sensor_msgs::msg::LaserScan &msg) {
    // 空扫描没有有效避障数据，保持上一次默认安全距离。
    if (msg.ranges.empty()) {
      return;
    }

    const auto mid = static_cast<int>(msg.ranges.size() / 2);
    front_range_ = clamp_range_or_default(msg.ranges[mid]);
    left_range_ = clamp_range_or_default(
        msg.ranges[std::min(static_cast<int>(msg.ranges.size()) - 1, mid + 30)]);
    right_range_ = clamp_range_or_default(msg.ranges[std::max(0, mid - 30)]);
  }

  // 处理深度图：读取中心 ROI 的有效深度，换算为米后用于近距离安全停车。
  void consume_depth(const sensor_msgs::msg::Image &msg) {
    // 深度安全开关关闭时，不更新深度避障距离。
    if (!use_depth_safety_) {
      return;
    }

    // 只支持 16 位单通道深度图；step 不足时认为数据不完整。
    if ((msg.encoding != "16UC1" && msg.encoding != "mono16") ||
        msg.width == 0 || msg.height == 0 || msg.step < msg.width * 2) {
      return;
    }

    const size_t pixel_count =
        static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    std::vector<uint16_t> depth_values(pixel_count, 0);

    for (uint32_t y = 0; y < msg.height; ++y) {
      const size_t row_offset = static_cast<size_t>(y) * msg.step;
      for (uint32_t x = 0; x < msg.width; ++x) {
        const size_t byte_index = row_offset + static_cast<size_t>(x) * 2;
        // 防止异常 step 或截断数据导致越界读取。
        if (byte_index + 1 >= msg.data.size()) {
          break;
        }

        // 按 ROS 图像端序解析 16 位深度值。
        const uint16_t value = !msg.is_bigendian
                                   ? static_cast<uint16_t>(
                                         static_cast<uint16_t>(msg.data[byte_index]) |
                                         (static_cast<uint16_t>(msg.data[byte_index + 1]) << 8))
                                   : static_cast<uint16_t>(
                                         (static_cast<uint16_t>(msg.data[byte_index]) << 8) |
                                         static_cast<uint16_t>(msg.data[byte_index + 1]));
        depth_values[static_cast<size_t>(y) * msg.width + x] = value;
      }
    }

    latest_depth_range_ = extract_center_depth_meters(
        depth_values,
        static_cast<int>(msg.width),
        static_cast<int>(msg.height),
        depth_roi_width_,
        depth_roi_height_,
        depth_unit_scale_);
    last_depth_time_ = now();
  }

  // 解析 /race/detections 简单字符串：更新 QR、P 点、行人、障碍物、黄区入口检测状态。
  void consume_simple_detections(const std::string &payload) {
    // 每帧先清空旧检测，避免感知丢帧时误用上一帧目标。
    qr_seen_ = false;
    qr_area_ = 0.0;
    human_seen_ = false;
    human_area_ = 0.0;
    parking_seen_ = false;
    parking_area_ = 0.0;
    obstacle_seen_ = false;
    obstacle_area_ = 0.0;
    obstacle_width_ = 0.0;
    obstacle_height_ = 0.0;
    obstacle_bottom_y_ = 0.0;
    yellow_entry_seen_ = false;

    // 空 payload 表示当前没有检测框。
    if (payload.empty()) {
      return;
    }

    std::stringstream detections(payload);
    std::string detection;
    while (std::getline(detections, detection, ';')) {
      if (detection.empty()) {
        continue;
      }

      std::stringstream fields(detection);
      std::vector<std::string> parts;
      std::string part;
      while (std::getline(fields, part, ',')) {
        parts.push_back(part);
      }
      if (parts.size() < 6) {
        continue;
      }

      const std::string &type = parts[0];
      const double x = std::stod(parts[1]);
      const double y = std::stod(parts[2]);
      const double width = std::stod(parts[3]);
      const double height = std::stod(parts[4]);
      const double area = width * height;
      const double center_x = x + width / 2.0;
      const double center_y = y + height / 2.0;

      switch (classify_detection_label(type)) {
      case DetectionRole::QR:
        // 二维码检测框：记录中心 x 和面积，用于靠近、对准和停车。
        qr_seen_ = true;
        qr_center_x_ = center_x;
        qr_area_ = area;
        break;
      case DetectionRole::P_POINT:
        // P 点检测框：返程停车区识别。
        parking_seen_ = true;
        parking_center_x_ = center_x;
        parking_center_y_ = center_y;
        parking_area_ = area;
        break;
      case DetectionRole::HUMAN:
        // 行人检测框：黄区绕行中触发结果上报。
        human_seen_ = true;
        human_label_ = type;
        human_area_ = area;
        break;
      case DetectionRole::OBSTACLE:
        // 障碍物检测框：面积超过阈值后进入短时保持，避免单帧丢失。
        obstacle_seen_ = true;
        obstacle_center_x_ = center_x;
        obstacle_width_ = width;
        obstacle_height_ = height;
        obstacle_bottom_y_ = y + height;
        obstacle_area_ = area;
        if (obstacle_candidate_confirmed(obstacle_trigger_area_, false) ||
            obstacle_candidate_confirmed(yellow_obstacle_trigger_area_, true)) {
          last_obstacle_time_ = now();
        }
        break;
      case DetectionRole::YELLOW_ENTRY:
        // 黄区入口类别：作为入口可见的辅助信号。
        yellow_entry_seen_ = true;
        break;
      case DetectionRole::UNKNOWN:
        break;
      }
    }
  }

  // 读取 yellow_boundary 发布的数组：黄线边界、绿色禁区和黄白格子比例。
  void consume_yellow_lane_info(const std_msgs::msg::Float64MultiArray &msg) {
    // 前 10 个字段是必需字段，缺失时丢弃该帧。
    if (msg.data.size() < 10) {
      return;
    }
    latest_lane_center_x_ = msg.data[0];
    latest_yellow_left_x_ = msg.data[1];
    latest_yellow_right_x_ = msg.data[2];
    latest_lane_width_ = msg.data[3];
    latest_yellow_area_ratio_ = msg.data[4];
    forbidden_grid_detected_ = msg.data[7] >= 0.5;
    latest_lane_yellow_visible_ = msg.data[8] >= 0.5;
    latest_lane_green_visible_ = msg.data[9] >= 0.5;
    const double raw_green_left_ratio = msg.data[5];
    const double raw_green_right_ratio = msg.data[6];
    const double raw_green_area_ratio = msg.data.size() > 10 ? msg.data[10] : 0.0;
    latest_white_near_ratio_ = msg.data.size() > 11 ? msg.data[11] : 0.0;
    latest_grid_area_ratio_ = latest_white_near_ratio_;
    const double raw_green_center_ratio = msg.data.size() > 12 ? msg.data[12] : 0.0;
    update_green_filter(raw_green_left_ratio, raw_green_right_ratio,
                        raw_green_center_ratio, raw_green_area_ratio);
    latest_grid_left_ratio_ = msg.data.size() > 13 ? msg.data[13] : 0.0;
    latest_grid_right_ratio_ = msg.data.size() > 14 ? msg.data[14] : 0.0;
    latest_grid_center_ratio_ = msg.data.size() > 15 ? msg.data[15] : 0.0;
    latest_grid_max_ratio_ =
        msg.data.size() > 16 ? msg.data[16] : latest_grid_area_ratio_;
    latest_yellow_near_ratio_ =
        msg.data.size() > 17 ? msg.data[17] : latest_yellow_area_ratio_;
    latest_yellow_left_ratio_ = msg.data.size() > 18 ? msg.data[18] : 0.0;
    latest_yellow_right_ratio_ = msg.data.size() > 19 ? msg.data[19] : 0.0;
    has_yellow_lane_info_ = true;
    last_yellow_lane_info_time_ = now();

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[GRID] yellow_near=%.3f white_near=%.3f detected=%d",
        latest_yellow_near_ratio_, latest_white_near_ratio_,
        forbidden_grid_detected_now());
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[YELLOW] yellow_area=%.3f pure=%d",
        latest_yellow_area_ratio_, pure_yellow_lane_detected());

    if (latest_lane_width_ >= lane_width_min_ &&
        latest_lane_width_ <= lane_width_max_) {
      // 用 EMA 维护通道宽度估计，单边黄线可见时用来补全中心线。
      lane_width_estimate_ = 0.8 * lane_width_estimate_ + 0.2 * latest_lane_width_;
    }
  }

  void update_green_filter(double left_ratio, double right_ratio,
                           double center_ratio, double area_ratio) {
    const double alpha = std::clamp(green_filter_alpha_, 0.0, 1.0);
    if (!green_filter_initialized_) {
      latest_green_left_ratio_ = left_ratio;
      latest_green_right_ratio_ = right_ratio;
      latest_green_center_ratio_ = center_ratio;
      latest_green_area_ratio_ = area_ratio;
      green_filter_initialized_ = true;
    } else {
      latest_green_left_ratio_ =
          alpha * left_ratio + (1.0 - alpha) * latest_green_left_ratio_;
      latest_green_right_ratio_ =
          alpha * right_ratio + (1.0 - alpha) * latest_green_right_ratio_;
      latest_green_center_ratio_ =
          alpha * center_ratio + (1.0 - alpha) * latest_green_center_ratio_;
      latest_green_area_ratio_ =
          alpha * area_ratio + (1.0 - alpha) * latest_green_area_ratio_;
    }

    if (green_forbidden_threshold_crossed()) {
      ++green_danger_ticks_;
      green_clear_ticks_ = 0;
      if (green_danger_ticks_ >= std::max(1, green_danger_confirm_ticks_)) {
        green_danger_confirmed_ = true;
      }
    } else {
      green_danger_ticks_ = 0;
      ++green_clear_ticks_;
      if (green_clear_ticks_ >= std::max(1, green_clear_confirm_ticks_)) {
        green_danger_confirmed_ = false;
      }
    }
  }

  // 普通巡线角速度：将图像中心与赛道中心的像素误差转换为 angular.z。
  double line_angular() const {
    // 巡线点超时则不输出角速度，避免使用过期感知。
    if (!has_recent_track()) {
      return 0.0;
    }

    return std::clamp((image_center_x_ - latest_track_x_) * line_kp_, -0.8, 0.8);
  }

  double line_error() const {
    return image_center_x_ - latest_track_x_;
  }

  const char *obstacle_bypass_phase_name(ObstacleBypassPhase phase) const {
    switch (phase) {
      case ObstacleBypassPhase::IDLE:
        return "IDLE";
      case ObstacleBypassPhase::AVOIDING:
        return "AVOIDING";
      case ObstacleBypassPhase::CLEARING:
        return "CLEARING";
      case ObstacleBypassPhase::RETURNING:
        return "RETURNING";
    }
    return "IDLE";
  }

  void reset_obstacle_bypass_state() {
    line_obstacle_phase_ = ObstacleBypassPhase::IDLE;
    line_obstacle_sign_ = 0.0;
    line_obstacle_reacquire_ticks_ = 0;
    line_obstacle_phase_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    yellow_obstacle_phase_ = ObstacleBypassPhase::IDLE;
    yellow_obstacle_sign_ = 0.0;
    yellow_obstacle_reacquire_count_ = 0;
    yellow_obstacle_phase_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void start_line_obstacle_avoidance() {
    line_obstacle_phase_ = ObstacleBypassPhase::AVOIDING;
    line_obstacle_sign_ = obstacle_avoidance_sign();
    line_obstacle_phase_start_time_ = now();
    line_obstacle_reacquire_ticks_ = 0;
  }

  double line_obstacle_bypass_angular(double lane_angular) {
    if (!obstacle_return_enabled_) {
      return std::clamp(obstacle_line_weight_ * lane_angular + obstacle_avoidance(),
                        -0.55, 0.55);
    }

    const bool active = obstacle_active();
    if (active && line_obstacle_phase_ == ObstacleBypassPhase::IDLE) {
      start_line_obstacle_avoidance();
    }

    if (line_obstacle_phase_ == ObstacleBypassPhase::AVOIDING) {
      const double elapsed =
          (now() - line_obstacle_phase_start_time_).seconds();
      if (!active && elapsed >= obstacle_avoid_min_seconds_) {
        line_obstacle_phase_ = obstacle_clear_tail_seconds_ > 0.0
                                   ? ObstacleBypassPhase::CLEARING
                                   : ObstacleBypassPhase::RETURNING;
        line_obstacle_phase_start_time_ = now();
        line_obstacle_reacquire_ticks_ = 0;
      } else {
        return std::clamp(obstacle_line_weight_ * lane_angular +
                              line_obstacle_sign_ * obstacle_avoid_angular_,
                          -0.55, 0.55);
      }
    }

    if (line_obstacle_phase_ == ObstacleBypassPhase::CLEARING) {
      if (active) {
        start_line_obstacle_avoidance();
        return std::clamp(obstacle_line_weight_ * lane_angular +
                              line_obstacle_sign_ * obstacle_avoid_angular_,
                          -0.55, 0.55);
      }

      const double elapsed =
          (now() - line_obstacle_phase_start_time_).seconds();
      if (elapsed >= obstacle_clear_tail_seconds_) {
        line_obstacle_phase_ = ObstacleBypassPhase::RETURNING;
        line_obstacle_phase_start_time_ = now();
        line_obstacle_reacquire_ticks_ = 0;
      } else {
        return std::clamp(obstacle_line_weight_ * lane_angular +
                              line_obstacle_sign_ * obstacle_avoid_angular_ *
                                  obstacle_clear_tail_angular_scale_,
                          -0.55, 0.55);
      }
    }

    if (line_obstacle_phase_ == ObstacleBypassPhase::RETURNING) {
      if (active) {
        start_line_obstacle_avoidance();
        return std::clamp(obstacle_line_weight_ * lane_angular +
                              line_obstacle_sign_ * obstacle_avoid_angular_,
                          -0.55, 0.55);
      }

      if (has_recent_track() &&
          std::abs(line_error()) <= obstacle_reacquire_error_) {
        ++line_obstacle_reacquire_ticks_;
      } else {
        line_obstacle_reacquire_ticks_ = 0;
      }

      const double elapsed =
          (now() - line_obstacle_phase_start_time_).seconds();
      const bool line_reacquired =
          elapsed >= obstacle_return_seconds_ &&
          line_obstacle_reacquire_ticks_ >=
          std::max(1, obstacle_reacquire_ticks_);
      if (line_reacquired) {
        line_obstacle_phase_ = ObstacleBypassPhase::IDLE;
        line_obstacle_reacquire_ticks_ = 0;
        return lane_angular;
      }

      const double return_bias =
          elapsed < obstacle_return_seconds_ ? obstacle_return_angular_
                                             : obstacle_return_angular_ * 0.75;
      if (elapsed < obstacle_return_force_seconds_) {
        return std::clamp(-line_obstacle_sign_ * return_bias, -0.55, 0.55);
      }
      return std::clamp(obstacle_return_line_weight_ * lane_angular -
                            line_obstacle_sign_ * return_bias,
                        -0.55, 0.55);
    }

    return lane_angular;
  }

  void start_yellow_obstacle_avoidance() {
    yellow_obstacle_phase_ = ObstacleBypassPhase::AVOIDING;
    yellow_obstacle_sign_ = obstacle_avoidance_sign();
    yellow_obstacle_phase_start_time_ = now();
    yellow_obstacle_reacquire_count_ = 0;
  }

  bool yellow_obstacle_active_or_returning() const {
    return yellow_obstacle_phase_ != ObstacleBypassPhase::IDLE;
  }

  bool yellow_obstacle_green_override_active() const {
    return yellow_obstacle_allow_green_ &&
           yellow_obstacle_phase_ == ObstacleBypassPhase::AVOIDING &&
           yellow_obstacle_active();
  }

  bool apply_yellow_obstacle_bypass(geometry_msgs::msg::Twist &cmd,
                                    double lane_angular, double lane_error,
                                    double safety,
                                    std::string &avoid_direction) {
    const bool active = yellow_obstacle_active();
    if (active && yellow_obstacle_phase_ == ObstacleBypassPhase::IDLE) {
      start_yellow_obstacle_avoidance();
    }

    if (yellow_obstacle_phase_ == ObstacleBypassPhase::IDLE) {
      return false;
    }

    if (yellow_obstacle_phase_ == ObstacleBypassPhase::AVOIDING) {
      const double elapsed =
          (now() - yellow_obstacle_phase_start_time_).seconds();
      if (active && elapsed < yellow_obstacle_avoid_seconds_) {
        cmd.linear.x = yellow_obstacle_avoid_speed_ * safety;
        cmd.angular.z =
            std::clamp(0.35 * lane_angular +
                           yellow_obstacle_sign_ * obstacle_avoid_angular_,
                       -0.60, 0.60);
        avoid_direction =
            yellow_obstacle_sign_ > 0.0 ? "yellow_obstacle_left"
                                        : "yellow_obstacle_right";
        return true;
      }

      yellow_obstacle_phase_ = ObstacleBypassPhase::RETURNING;
      yellow_obstacle_phase_start_time_ = now();
      yellow_obstacle_reacquire_count_ = 0;
    }

    if (yellow_obstacle_phase_ == ObstacleBypassPhase::RETURNING) {
      if (active &&
          (now() - yellow_obstacle_phase_start_time_).seconds() >
              yellow_obstacle_return_seconds_ * 0.5) {
        start_yellow_obstacle_avoidance();
        cmd.linear.x = yellow_obstacle_avoid_speed_ * safety;
        cmd.angular.z =
            std::clamp(0.35 * lane_angular +
                           yellow_obstacle_sign_ * obstacle_avoid_angular_,
                       -0.60, 0.60);
        avoid_direction =
            yellow_obstacle_sign_ > 0.0 ? "yellow_obstacle_left"
                                        : "yellow_obstacle_right";
        return true;
      }

      if (std::abs(lane_error) <= yellow_obstacle_reacquire_error_) {
        ++yellow_obstacle_reacquire_count_;
      } else {
        yellow_obstacle_reacquire_count_ = 0;
      }

      const double elapsed =
          (now() - yellow_obstacle_phase_start_time_).seconds();
      const bool lane_reacquired =
          latest_lane_yellow_visible_ &&
          !green_forbidden_threshold_crossed() &&
          yellow_obstacle_reacquire_count_ >=
              std::max(1, yellow_obstacle_reacquire_ticks_);
      if (lane_reacquired ||
          elapsed >= yellow_obstacle_return_seconds_) {
        yellow_obstacle_phase_ = ObstacleBypassPhase::IDLE;
        yellow_obstacle_reacquire_count_ = 0;
        return false;
      }

      cmd.linear.x = yellow_lane_speed_ * safety;
      cmd.angular.z =
          std::clamp(yellow_obstacle_return_line_weight_ * lane_angular -
                         yellow_obstacle_sign_ *
                             yellow_obstacle_return_angular_,
                     -0.60, 0.60);
      avoid_direction = "yellow_obstacle_return";
      return true;
    }

    return false;
  }

  // 判断黄色质心是否新鲜，超时时间 0.4 秒。
  bool has_recent_yellow() const {
    if (!has_yellow_) {
      return false;
    }
    return (now() - last_yellow_time_).seconds() < 0.4;
  }

  // 判断绿色质心是否新鲜，超时时间 0.4 秒。
  bool has_recent_green() const {
    if (!has_green_) {
      return false;
    }
    return (now() - last_green_time_).seconds() < 0.4;
  }

  // 黄色质心闭环角速度：用于无引导线段对准黄通道，角速度单位 rad/s。
  double yellow_color_angular() const {
    // 未启用、黄色过期或面积不足时不使用黄色闭环。
    if (!use_yellow_color_guidance_ || !has_recent_yellow() ||
        latest_yellow_area_ < yellow_min_area_for_guidance_) {
      return 0.0;
    }
    return std::clamp((image_center_x_ - latest_yellow_x_) * yellow_boundary_kp_, -0.85, 0.85);
  }

  // 进黄区专用：线模型在「无/弱引导线」的蓝色开阔区常仍输出点，会拉着车乱跑；与黄色质心做加权融合
  double yellow_entry_merged_angular() const {
    const bool y_ok =
        use_yellow_color_guidance_ && has_recent_yellow() &&
        latest_yellow_area_ >= yellow_min_area_for_guidance_;
    const bool line_ok = has_recent_track();
    const double wy = std::clamp(yellow_entry_yellow_line_blend_, 0.0, 1.0);
    // 黄块和巡线都有效：按 yellow_entry_yellow_line_blend 融合，黄通道优先。
    if (y_ok && line_ok) {
      return wy * yellow_color_angular() + (1.0 - wy) * line_angular();
    }
    // 只有巡线有效：按比例缩小巡线输出，并叠加盲走角速度补偿。
    if (line_ok && !y_ok) {
      const double s = std::clamp(yellow_entry_line_kp_scale_, 0.0, 1.0);
      return s * line_angular() + (1.0 - s) * yellow_entry_blind_angular_;
    }
    // 只有黄色有效：完全使用黄色质心闭环。
    if (y_ok) {
      return yellow_color_angular();
    }
    // 都无效：执行配置的盲走角速度。
    return yellow_entry_blind_angular_;
  }

  // 进黄区 TURN：横向闭环与开环转弯叠加；无巡线/无黄块时横向为 0（不混入 blind，由 open 段负责转向）
  double yellow_entry_turn_closed_loop() const {
    const bool y_ok =
        use_yellow_color_guidance_ && has_recent_yellow() &&
        latest_yellow_area_ >= yellow_min_area_for_guidance_;
    const bool line_ok = has_recent_track();
    const double wy = std::clamp(yellow_entry_yellow_line_blend_, 0.0, 1.0);
    // 转向阶段可同时参考黄色质心和巡线，避免开环转过头。
    if (y_ok && line_ok) {
      return wy * yellow_color_angular() + (1.0 - wy) * line_angular();
    }
    // 无黄色时只保留缩小后的巡线角速度，不加入盲走项。
    if (line_ok && !y_ok) {
      const double s = std::clamp(yellow_entry_line_kp_scale_, 0.0, 1.0);
      return s * line_angular();
    }
    if (y_ok) {
      return yellow_color_angular();
    }
    return 0.0;
  }

  // 直接进黄锁定判断：黄色面积、中心偏差和连续帧数都满足后返回 true。
  bool direct_yellow_entry_locked() {
    const bool y_ok =
        use_yellow_color_guidance_ && has_recent_yellow() &&
        latest_yellow_area_ >= direct_yellow_entry_min_area_ &&
        std::abs(latest_yellow_x_ - image_center_x_) <= direct_yellow_entry_center_tolerance_;
    if (y_ok) {
      ++direct_yellow_entry_lock_count_;
    } else {
      // 任一帧不满足即清零，要求连续稳定锁定。
      direct_yellow_entry_lock_count_ = 0;
    }
    return direct_yellow_entry_lock_count_ >= direct_yellow_entry_lock_ticks_;
  }

  // 黄区入口中心是否已对准：黄色质心面积足够且 x 偏差在像素容差内。
  bool yellow_entry_center_detected() const {
    return has_recent_yellow() && latest_yellow_area_ >= yellow_entry_min_area_ &&
           std::abs(latest_yellow_x_ - image_center_x_) <= yellow_entry_center_tolerance_;
  }

  // 前进穿过黄区门口前的中心锁定判断。
  bool yellow_gate_centered_for_forward() const {
    return use_yellow_color_guidance_ && has_recent_yellow() &&
           latest_yellow_area_ >= yellow_min_area_for_guidance_ &&
           std::abs(latest_yellow_x_ - image_center_x_) <=
               yellow_entry_gate_lock_tolerance_;
  }

  // 黄区门口是否可见：黄色质心或 yellow_lane_info 任一满足即认为可继续前进。
  bool yellow_gate_visible_for_forward() const {
    const bool boundary_visible =
        use_yellow_color_guidance_ && has_recent_yellow() &&
        latest_yellow_area_ >= yellow_min_area_for_guidance_;
    const bool lane_visible =
        has_recent_yellow_lane_info() &&
        latest_yellow_area_ratio_ >= std::max(0.02, yellow_gate_threshold_ * 0.5) &&
        latest_lane_width_ >= yellow_entry_lane_width_min_ * 0.6;
    return boundary_visible || lane_visible;
  }

  // 转向阶段是否已经看见可停止转弯的黄区入口，过滤中心黄白格子干扰。
  bool yellow_gate_visible_to_stop_turn() const {
    const bool grid_center_safe =
        !has_recent_yellow_lane_info() ||
        latest_grid_center_ratio_ <= yellow_entry_grid_max_ratio_;
    const bool boundary_visible =
        use_yellow_color_guidance_ && has_recent_yellow() &&
        (latest_yellow_area_ >=
             std::max(2200.0, yellow_min_area_for_guidance_ * 0.55) ||
         yellow_area_ratio() >= 0.025) &&
        std::abs(latest_yellow_x_ - image_center_x_) <=
            yellow_entry_gate_lock_tolerance_ * 1.4 &&
        grid_center_safe;
    const bool lane_visible =
        has_recent_yellow_lane_info() &&
        latest_yellow_area_ratio_ >=
            std::max(0.025, yellow_gate_threshold_ * 0.45) &&
        latest_lane_yellow_visible_ &&
        latest_lane_width_ >= yellow_entry_lane_width_min_ * 0.8 &&
        std::abs(latest_lane_center_x_ - image_center_x_) <=
            yellow_entry_gate_lock_tolerance_ * 1.3 &&
        grid_center_safe;
    return boundary_visible || lane_visible;
  }

  // 黄区入口格子是否清除：无黄白格禁区且中心格子比例低于阈值。
  bool yellow_entry_grid_clear() const {
    if (!has_recent_yellow_lane_info()) {
      return false;
    }
    return !forbidden_grid_detected_now() &&
           latest_grid_center_ratio_ <= yellow_entry_grid_max_ratio_;
  }

  // 黄区入口是否被黄白格子拒绝：任一格子比例超阈值则拒绝进入。
  bool yellow_entry_grid_rejected() const {
    if (!has_recent_yellow_lane_info()) {
      return false;
    }
    return forbidden_grid_detected_now() ||
           latest_grid_center_ratio_ > yellow_entry_grid_max_ratio_ ||
           latest_grid_max_ratio_ > yellow_entry_grid_max_ratio_;
  }

  // 当前近处白色比例，0~1，用于白色禁入判断。
  double white_seen_ratio_now() const {
    if (!has_recent_yellow_lane_info()) {
      return 0.0;
    }
    return latest_white_near_ratio_;
  }

  // 白色危险比例：结合黄白格重叠和纯白区域，返回 0~1。
  double white_danger_ratio_now() const {
    if (!has_recent_yellow_lane_info()) {
      return 0.0;
    }
    const double white_seen_ratio = white_seen_ratio_now();
    const double checker_overlap =
        std::max(latest_grid_center_ratio_, latest_grid_max_ratio_);
    const double white_danger_ratio =
        std::max(checker_overlap, white_seen_ratio - latest_yellow_near_ratio_);
    return std::clamp(white_danger_ratio, 0.0, 1.0);
  }

  // 是否看到白色禁入区域。
  bool white_seen_detected_now() const {
    return white_seen_ratio_now() > white_seen_threshold_;
  }

  // 是否达到近距离白色危险阈值。
  bool white_danger_detected_now() const {
    return white_danger_ratio_now() > white_danger_threshold_;
  }

  // 白色禁入统一入口，当前等价于 white_seen_detected_now。
  bool white_forbidden_detected_now() const {
    return white_seen_detected_now();
  }

  // 黄区入口视觉禁入判断：黄白格、白色区域等会给出 reason。
  bool entry_forbidden_visual_detected(std::string &reason) const {
    if (!has_recent_yellow_lane_info()) {
      return false;
    }
    if (forbidden_grid_detected_) {
      reason = "forbidden_grid";
      return true;
    }
    if (forbidden_grid_detected_now()) {
      reason = "checker";
      return true;
    }
    if (white_seen_detected_now()) {
      reason = "white_seen";
      return true;
    }
    return false;
  }

  // 黄区入口近距离危险判断：白色紧贴或格子占比过高时触发。
  bool entry_forbidden_close_danger_detected(std::string &reason) const {
    if (!has_recent_yellow_lane_info()) {
      return false;
    }
    if (white_seen_ratio_now() > white_emergency_threshold_) {
      reason = "white_emergency";
      return true;
    }
    if (latest_grid_center_ratio_ >= entry_close_grid_threshold_ ||
        latest_grid_max_ratio_ >= entry_close_grid_threshold_) {
      reason = "close_danger";
      return true;
    }
    if ((latest_grid_center_ratio_ >= entry_close_grid_soft_threshold_ ||
         latest_grid_max_ratio_ >= entry_close_grid_soft_threshold_) &&
        white_danger_detected_now()) {
      reason = "close_danger";
      return true;
    }
    return false;
  }

  // 黄白格实时判断：显式格子标志或近处黄/白比例同时超阈值。
  bool forbidden_grid_detected_now() const {
    if (!has_recent_yellow_lane_info()) {
      return false;
    }
    return forbidden_grid_detected_ ||
           (latest_yellow_near_ratio_ > grid_yellow_ratio_threshold_ &&
            latest_white_near_ratio_ > grid_white_ratio_threshold_);
  }

  // 强制接近黄区门口后，格子是否仍处于硬危险状态。
  bool post_approach_hard_grid_detected() const {
    return latest_grid_center_ratio_ > post_approach_hard_grid_threshold_ ||
           latest_grid_max_ratio_ > post_approach_hard_grid_threshold_;
  }

  // 强制前探后的黄区候选信号：黄色面积或中心有效即认为有候选。
  bool post_approach_candidate_signal_detected(double &center_x,
                                               bool &center_valid) const {
    center_x = 0.0;
    center_valid = yellow_entry_center_valid(center_x);
    return latest_yellow_area_ratio_ > post_approach_candidate_area_threshold_ ||
           center_valid;
  }

  // 强制接近后的宽限时间窗口，elapsed 单位秒。
  bool post_approach_candidate_grace_active(double &elapsed) const {
    elapsed = 0.0;
    if (post_approach_candidate_grace_duration_ <= 0.0 ||
        post_approach_candidate_grace_start_time_.nanoseconds() == 0) {
      return false;
    }
    elapsed = (now() - post_approach_candidate_grace_start_time_).seconds();
    return elapsed >= 0.0 &&
           elapsed < post_approach_candidate_grace_duration_;
  }

  // 宽限窗口内是否允许把黄区候选视为有效，同时排除紧急白色和硬格子。
  bool post_approach_candidate_grace_allows(double &elapsed, double &center_x,
                                            bool &center_valid) const {
    if (!post_approach_candidate_grace_active(elapsed)) {
      center_x = 0.0;
      center_valid = false;
      return false;
    }
    if (white_seen_ratio_now() > white_emergency_threshold_ ||
        post_approach_hard_grid_detected()) {
      center_x = 0.0;
      center_valid = false;
      return false;
    }
    return post_approach_candidate_signal_detected(center_x, center_valid);
  }

  // 强黄区候选判断：黄色面积/中心/车道可见，并且未被白色或格子禁入。
  bool strong_yellow_candidate_detected(double &center_x,
                                        bool &center_valid) const {
    center_x = 0.0;
    center_valid = yellow_entry_center_valid(center_x);
    const bool emergency_white =
        white_seen_ratio_now() > white_emergency_threshold_;
    return (latest_yellow_area_ratio_ > yellow_candidate_area_threshold_ ||
            center_valid || latest_lane_yellow_visible_) &&
           !white_danger_detected_now() &&
           !forbidden_grid_detected_ &&
           !forbidden_grid_detected_now() &&
           !emergency_white;
  }

  // 黄区入口拒绝逻辑：优先处理近距离危险，再处理视觉禁入和无候选白色。
  bool entry_yellow_rejected(std::string &reason) const {
    double center_x = 0.0;
    bool center_valid = false;
    const bool strong_candidate =
        strong_yellow_candidate_detected(center_x, center_valid);
    if (entry_forbidden_close_danger_detected(reason)) {
      return true;
    }
    std::string visual_reason;
    if (entry_forbidden_visual_detected(visual_reason)) {
      if (visual_reason == "white_seen") {
        if (!strong_candidate) {
          reason = "white_seen_no_candidate";
          return true;
        }
      } else {
        reason = visual_reason;
        return true;
      }
    }
    if (white_seen_detected_now() && !strong_candidate) {
      reason = "white_seen_no_candidate";
      return true;
    }
    return false;
  }

  // 入口危险判断别名，保留给恢复逻辑调用。
  bool entry_forbidden_danger_detected(std::string &reason) const {
    return entry_forbidden_close_danger_detected(reason);
  }

  // 入口禁入判断别名，返回黄区入口是否应该拒绝。
  bool entry_forbidden_detected(std::string &reason) const {
    return entry_yellow_rejected(reason);
  }

  // 黄区候选判断别名，返回可跟随的中心点。
  bool yellow_candidate_detected(double &center_x, bool &center_valid) const {
    return strong_yellow_candidate_detected(center_x, center_valid);
  }

  // 纯黄色通道判断：黄色面积足够、白色格子低且无禁入格。
  bool pure_yellow_lane_detected() const {
    if (!has_recent_yellow_lane_info()) {
      return false;
    }
    return latest_yellow_area_ratio_ > yellow_lane_area_threshold_ &&
           latest_white_near_ratio_ < white_grid_reject_threshold_ &&
           (!checker_forbidden_enabled_ || !forbidden_grid_detected_now());
  }

  // 黄白格恢复动作：检测到格子后直线后退，直到格子消失或进入下一轮后退。
  bool apply_grid_recovery(geometry_msgs::msg::Twist &cmd) {
    const bool checker_detected = forbidden_grid_detected_now();
    // 没检测到格子且不在恢复流程时，不接管控制。
    if (!checker_detected && !grid_recovering_) {
      return false;
    }

    // 首次检测到黄白格：启动后退恢复流程并记录次数。
    if (checker_detected && !grid_recovering_) {
      ++grid_recover_count_;
      ++checker_recover_count_;
      grid_recovering_ = true;
      grid_recover_phase_ = 1;
      grid_recover_start_time_ = now();
      RCLCPP_WARN(get_logger(),
                  "[CHECKER] detected -> straight_backup, count=%d",
                  checker_recover_count_);
    }

    if (!grid_recovering_) {
      return false;
    }

    double elapsed = (now() - grid_recover_start_time_).seconds();
    // 后退时间结束后复查格子；如果仍存在，则重新开始一次后退。
    if (elapsed >= checker_backup_duration_) {
      if (!checker_detected) {
        grid_recovering_ = false;
        grid_recover_phase_ = 0;
        RCLCPP_INFO(get_logger(),
                    "[CHECKER_RECOVER] done -> resume yellow search");
        return false;
      }
      ++grid_recover_count_;
      ++checker_recover_count_;
      grid_recover_start_time_ = now();
      elapsed = 0.0;
      RCLCPP_WARN(get_logger(),
                  "[CHECKER] detected -> straight_backup, count=%d",
                  checker_recover_count_);
    }

    cmd = geometry_msgs::msg::Twist();
    // 黄白格禁入恢复只后退，不叠加转向，避免误转入白格。
    cmd.linear.x = -std::abs(checker_backup_speed_);
    cmd.angular.z = 0.0;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[CHECKER_RECOVER] backup elapsed=%.2f cmd=(%.2f,%.2f)",
        elapsed, cmd.linear.x, cmd.angular.z);
    return true;
  }

  // 根据格子出现在左/右/中位置，选择远离格子的入口避让角速度。
  double yellow_entry_grid_avoid_angular() const {
    if (latest_grid_left_ratio_ > latest_grid_right_ratio_ &&
        latest_grid_left_ratio_ >= latest_grid_center_ratio_) {
      return -std::abs(yellow_entry_grid_avoid_angular_);
    }
    if (latest_grid_right_ratio_ > latest_grid_left_ratio_ &&
        latest_grid_right_ratio_ >= latest_grid_center_ratio_) {
      return std::abs(yellow_entry_grid_avoid_angular_);
    }
    return yellow_entry_turn_left_
               ? std::abs(yellow_entry_grid_avoid_angular_)
               : -std::abs(yellow_entry_grid_avoid_angular_);
  }

  // 黄区入口车道角速度：lane_center_x 为像素坐标，输出限制在最大入口角速度内。
  double yellow_entry_lane_angular(double lane_center_x) const {
    return std::clamp((image_center_x_ - lane_center_x) * yellow_boundary_kp_,
                      -yellow_entry_approach_max_angular_,
                      yellow_entry_approach_max_angular_);
  }

  // 黄区车道是否已满足入口条件：面积、宽度、中心偏差和格子清除都要满足。
  bool yellow_lane_entry_ready() const {
    if (!has_recent_yellow_lane_info()) {
      return false;
    }
    return latest_yellow_area_ratio_ >= yellow_gate_threshold_ &&
           latest_lane_yellow_visible_ &&
           latest_lane_width_ >= yellow_entry_lane_width_min_ &&
           latest_lane_width_ <= lane_width_max_ &&
           std::abs(latest_lane_center_x_ - image_center_x_) <=
               yellow_entry_gate_lock_tolerance_ &&
           yellow_entry_grid_clear();
  }

  // 面积主导入口判断预留，目前固定关闭，不改变原有入口策略。
  bool yellow_entry_area_dominant_ready() const {
    return false;
  }

  // 黄区入口面积是否就绪：黄色边界面积足够或车道入口 ready。
  bool yellow_entry_area_ready() const {
    const bool boundary_area_ready =
        has_recent_yellow() &&
        has_recent_yellow_lane_info() &&
        latest_lane_yellow_visible_ &&
        latest_lane_width_ >= yellow_entry_lane_width_min_ &&
        latest_yellow_area_ >=
            std::max(yellow_entry_min_area_, yellow_min_area_for_guidance_ * 4.0) &&
        std::abs(latest_yellow_x_ - image_center_x_) <=
            yellow_entry_gate_lock_tolerance_ &&
        yellow_entry_grid_clear();
    return boundary_area_ready || yellow_lane_entry_ready();
  }

  // 黄色质心面积归一化为 0~1，yellow_roi_pixel_area_ 单位为像素。
  double yellow_area_ratio() const {
    if (!has_recent_yellow() || yellow_roi_pixel_area_ <= 1.0) {
      return 0.0;
    }
    return std::clamp(latest_yellow_area_ / yellow_roi_pixel_area_, 0.0, 1.0);
  }

  // 只使用黄色质心控制入口方向；无黄色时返回盲走角速度。
  double yellow_entry_color_only_angular() const {
    if (use_yellow_color_guidance_ && has_recent_yellow() &&
        latest_yellow_area_ >= yellow_min_area_for_guidance_) {
      return yellow_color_angular();
    }
    return yellow_entry_blind_angular_;
  }

  // 黄区绕行角速度：按顺/逆时针目标横向比例保持黄色边界在目标位置。
  double yellow_lap_angular(TurnDirection direction) const {
    const double target_ratio =
        (direction == TurnDirection::COUNTERCLOCKWISE)
            ? counterclockwise_target_ratio_
            : clockwise_target_ratio_;
    const double target_x = std::clamp(target_ratio, 0.05, 0.95) * image_center_x_ * 2.0;
    const double error = latest_yellow_x_ - target_x;
    return std::clamp(-yellow_kp_ * error, -yellow_max_angular_z_, yellow_max_angular_z_);
  }

  double yellow_direction_search_sign(TurnDirection direction) const {
    if (direction == TurnDirection::COUNTERCLOCKWISE) {
      return 1.0;
    }
    if (direction == TurnDirection::CLOCKWISE) {
      return -1.0;
    }
    return 0.0;
  }

  double clamp_yellow_direction_angular(double angular,
                                        TurnDirection direction) const {
    const double sign = yellow_direction_search_sign(direction);
    if (sign > 0.0 && angular < 0.0) {
      return 0.0;
    }
    if (sign < 0.0 && angular > 0.0) {
      return 0.0;
    }
    return angular;
  }

  double yellow_direction_search_angular(TurnDirection direction,
                                         double magnitude) const {
    const double sign = yellow_direction_search_sign(direction);
    if (sign == 0.0) {
      return magnitude;
    }
    return sign * std::abs(magnitude);
  }

  // yellow_lane_info 是否在有效历史窗口内，窗口单位秒。
  bool has_recent_yellow_lane_info() const {
    if (!has_yellow_lane_info_) {
      return false;
    }
    return (now() - last_yellow_lane_info_time_).seconds() < yellow_lane_history_keep_time_;
  }

  // 计算黄区车道中心：双边可见用真实中心，单边可见用估计宽度补全。
  bool yellow_lane_center(double &lane_center_x, double &lane_width) const {
    if (!has_recent_yellow_lane_info() ||
        latest_yellow_area_ratio_ < yellow_min_area_ratio_) {
      return false;
    }

    const bool left_ok = latest_yellow_left_x_ >= 0.0;
    const bool right_ok = latest_yellow_right_x_ >= 0.0;
    const bool width_ok =
        latest_lane_width_ >= lane_width_min_ && latest_lane_width_ <= lane_width_max_;
    // 左右边界都有效：直接使用检测出的中心和宽度。
    if (left_ok && right_ok && width_ok && latest_lane_yellow_visible_) {
      lane_center_x = latest_lane_center_x_;
      lane_width = latest_lane_width_;
      return true;
    }
    // 只有左边界：用历史宽度估计右边界，补出车道中心。
    if (left_ok && !right_ok) {
      lane_center_x = latest_yellow_left_x_ + 0.5 * lane_width_estimate_;
      lane_width = lane_width_estimate_;
      return true;
    }
    // 只有右边界：用历史宽度估计左边界，补出车道中心。
    if (!left_ok && right_ok) {
      lane_center_x = latest_yellow_right_x_ - 0.5 * lane_width_estimate_;
      lane_width = lane_width_estimate_;
      return true;
    }
    return false;
  }

  bool yellow_lane_center_for_direction(TurnDirection direction,
                                        double &lane_center_x,
                                        double &lane_width) const {
    if (!has_recent_yellow_lane_info() ||
        latest_yellow_area_ratio_ < yellow_min_area_ratio_) {
      return false;
    }

    const bool left_ok = latest_yellow_left_x_ >= 0.0;
    const bool right_ok = latest_yellow_right_x_ >= 0.0;
    const bool left_clipped = latest_yellow_left_x_ <= 2.0;
    const bool right_clipped =
        latest_yellow_right_x_ >= image_center_x_ * 2.0 - 2.0;
    const bool width_ok =
        latest_lane_width_ >= lane_width_min_ && latest_lane_width_ <= lane_width_max_;

    if (left_ok && right_ok && width_ok && latest_lane_yellow_visible_ &&
        !left_clipped && !right_clipped) {
      lane_center_x = latest_lane_center_x_;
      lane_width = latest_lane_width_;
      return true;
    }

    lane_width = lane_width_estimate_;
    if (direction == TurnDirection::COUNTERCLOCKWISE) {
      if (right_ok) {
        lane_center_x = latest_yellow_right_x_ - 0.5 * lane_width_estimate_;
        return true;
      }
      if (left_ok) {
        lane_center_x = latest_yellow_left_x_ + 0.5 * lane_width_estimate_;
        return true;
      }
    } else {
      if (left_ok) {
        lane_center_x = latest_yellow_left_x_ + 0.5 * lane_width_estimate_;
        return true;
      }
      if (right_ok) {
        lane_center_x = latest_yellow_right_x_ - 0.5 * lane_width_estimate_;
        return true;
      }
    }
    return false;
  }

  // 黄车道中心 Alpha-Beta 滤波器（稳态卡尔曼）：对 lane_center_x 做位置+速度估计，
  // 消除 HSV 检测逐帧噪声，抑制纯 P 控制引起的转向抖动。
  void update_yellow_lane_filter(double measurement) {
    const rclcpp::Time now_time = now();
    if (!yellow_lane_filter_initialized_) {
      yellow_lane_filter_x_ = measurement;
      yellow_lane_filter_v_ = 0.0;
      yellow_lane_filter_last_time_ = now_time;
      yellow_lane_filter_initialized_ = true;
      return;
    }
    const double dt = (now_time - yellow_lane_filter_last_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      // 时间跳变或第一帧：重新初始化。
      yellow_lane_filter_x_ = measurement;
      yellow_lane_filter_v_ = 0.0;
      yellow_lane_filter_last_time_ = now_time;
      return;
    }
    // 预测：匀速模型。
    const double x_pred = yellow_lane_filter_x_ + yellow_lane_filter_v_ * dt;
    // 更新：测量残差驱动。
    const double residual = measurement - x_pred;
    yellow_lane_filter_x_ = x_pred + yellow_lane_alpha_ * residual;
    yellow_lane_filter_v_ = yellow_lane_filter_v_ + yellow_lane_beta_ * residual / dt;
    yellow_lane_filter_last_time_ = now_time;
  }

  void reset_yellow_lane_filter() {
    yellow_lane_filter_initialized_ = false;
    yellow_lane_filter_x_ = 320.0;
    yellow_lane_filter_v_ = 0.0;
  }

  // 黄区丢线/禁区附近的逃逸方向：根据格子、绿色禁区和入口方向选择左右。
  double yellow_lane_escape_direction() const {
    if (latest_grid_left_ratio_ > latest_grid_right_ratio_ &&
        latest_grid_left_ratio_ > latest_grid_center_ratio_) {
      return -1.0;
    }
    if (latest_grid_right_ratio_ > latest_grid_left_ratio_ &&
        latest_grid_right_ratio_ > latest_grid_center_ratio_) {
      return 1.0;
    }
    if (latest_green_left_ratio_ > latest_green_right_ratio_) {
      return -1.0;
    }
    if (latest_green_right_ratio_ > latest_green_left_ratio_) {
      return 1.0;
    }
    return yellow_entry_turn_left_ ? 1.0 : -1.0;
  }

  // 判断某个转向方向是否会转向绿色禁区或黄白格禁区。
  bool yellow_lane_turn_blocked(double angular) const {
    // angular > 0 为左转，左侧禁区存在时禁止继续左转。
    if (angular > 0.0 && latest_green_left_ratio_ >= green_forbidden_threshold_) {
      return true;
    }
    // angular < 0 为右转，右侧禁区存在时禁止继续右转。
    if (angular < 0.0 && latest_green_right_ratio_ >= green_forbidden_threshold_) {
      return true;
    }
    if (angular > 0.0 && latest_grid_left_ratio_ >= grid_forbidden_threshold_) {
      return true;
    }
    if (angular < 0.0 && latest_grid_right_ratio_ >= grid_forbidden_threshold_) {
      return true;
    }
    return forbidden_grid_detected_ &&
           latest_grid_max_ratio_ >= grid_forbidden_threshold_;
  }

  // 黄区出口门判断：只按预期绕行时间触发出口动作。
  // 是否真正出通道只由 FORWARD_EXIT_GATE 中重新看到返程巡线决定。
  bool yellow_gate_detected(double stage_seconds) const {
    return expected_lap_time_ > 0.0 && stage_seconds >= expected_lap_time_;
  }

  // 黄通道出口特征：左侧仍是绿墙，右侧绿色消失，说明右侧接到主通道出口。
  bool yellow_exit_green_gap_detected(double stage_seconds) {
    if (stage_seconds < yellow_exit_green_gap_min_time_ ||
        !has_recent_yellow_lane_info()) {
      yellow_exit_green_gap_ticks_ = 0;
      return false;
    }

    const bool green_gap =
        latest_green_left_ratio_ >= yellow_exit_green_left_threshold_ &&
        latest_green_right_ratio_ <= yellow_exit_green_right_clear_threshold_ &&
        latest_yellow_right_ratio_ >= yellow_exit_right_yellow_threshold_;
    if (green_gap) {
      ++yellow_exit_green_gap_ticks_;
    } else {
      yellow_exit_green_gap_ticks_ = 0;
    }
    return yellow_exit_green_gap_ticks_ >= yellow_exit_green_gap_ticks_required_;
  }

  // 出黄区岔口选择：优先朝黄色面积大、绿色更少的一侧走，避免继续钻进窄黄通道绕圈。
  double wide_yellow_exit_angular(double fallback_angular) {
    const double turn = std::abs(exit_gate_turn_angular_z_);
    if (!has_recent_yellow_lane_info()) {
      return exit_wide_direction_ == 0
                 ? fallback_angular
                 : static_cast<double>(exit_wide_direction_) * turn;
    }

    // 绿色在出口阶段只作为避让依据，不再作为黄绿边界循迹目标。
    double left_score =
        latest_yellow_left_ratio_ - 0.75 * latest_green_left_ratio_;
    double right_score =
        latest_yellow_right_ratio_ - 0.75 * latest_green_right_ratio_;

    if (latest_lane_width_ >= lane_width_min_) {
      const double center_offset =
          std::clamp((latest_lane_center_x_ - image_center_x_) / image_center_x_,
                     -1.0, 1.0);
      if (center_offset > 0.12) {
        right_score += 0.04;
      } else if (center_offset < -0.12) {
        left_score += 0.04;
      }
    }

    const double diff = right_score - left_score;
    if (std::abs(diff) >= 0.025) {
      exit_wide_direction_ = diff > 0.0 ? -1 : 1;
    }

    if (exit_wide_direction_ != 0) {
      return static_cast<double>(exit_wide_direction_) * turn;
    }
    return fallback_angular;
  }

  // 黄区闭环绕行角速度：优先用绿色边界，其次黄色边界，最后开环转弯兜底。
  double yellow_loop_closed_loop_angular(TurnDirection direction) const {
    const bool green_ok =
        has_recent_green() && latest_green_area_ >= yellow_loop_green_min_area_;
    const bool yellow_ok =
        use_yellow_color_guidance_ && has_recent_yellow() &&
        latest_yellow_area_ >= yellow_loop_yellow_min_area_;

    // 绿色边界可靠时，把绿色保持在目标侧向偏移处，避免压到绿色禁区。
    if (green_ok) {
      const double side =
          (direction == TurnDirection::CLOCKWISE) ? 1.0 : -1.0;
      const double target_x = image_center_x_ + side * yellow_loop_green_target_offset_;
      const double green_cmd =
          std::clamp((target_x - latest_green_x_) * yellow_loop_green_kp_, -0.75, 0.75);
      if (yellow_ok) {
        // 同时看到黄/绿时按权重融合，提高环道中心保持稳定性。
        const double yellow_cmd =
            std::clamp((image_center_x_ - latest_yellow_x_) * yellow_loop_yellow_kp_,
                       -0.65, 0.65);
        const double green_w = std::clamp(yellow_loop_green_weight_, 0.0, 1.0);
        return std::clamp(green_w * green_cmd + (1.0 - green_w) * yellow_cmd,
                          -0.75, 0.75);
      }
      return green_cmd;
    }

    // 只看到黄色时，按黄色质心居中控制。
    if (yellow_ok) {
      return std::clamp((image_center_x_ - latest_yellow_x_) * yellow_loop_yellow_kp_,
                        -0.65, 0.65);
    }

    // 两种边界都不可用时，用配置的开环角速度继续找边界。
    return yellow_loop_corner_turn(direction, yellow_loop_fallback_angular_);
  }

  // 绿色禁区是否位于画面中心且面积足够大，表示前方危险。
  bool green_danger_centered() const {
    return has_recent_green() &&
           latest_green_area_ >= yellow_loop_green_danger_area_ &&
           std::abs(latest_green_x_ - image_center_x_) <=
               yellow_loop_green_danger_tolerance_;
  }

  // 巡线点是否新鲜，超时时间 0.35 秒。
  bool has_recent_track() const {
    if (!has_track_) {
      return false;
    }
    return (now() - last_track_time_).seconds() < 0.35;
  }

  // 雷达左右居中角速度：右距减左距，经 Kp 转为 angular.z。
  double lidar_centering() const {
    // 任一侧距离接近默认远距时认为墙面不可用，不做居中。
    if (left_range_ > 9.0 || right_range_ > 9.0) {
      return 0.0;
    }

    return std::clamp((right_range_ - left_range_) * lidar_center_kp_, -0.5, 0.5);
  }

  // 障碍物避让角速度：障碍在左则右绕，障碍在右则左绕。
  double obstacle_avoidance() const {
    if (!obstacle_active()) {
      return 0.0;
    }
    return obstacle_avoidance_sign() * obstacle_avoid_angular_;
  }

  double obstacle_avoidance_sign() const {
    if (use_lidar_safety_ && left_range_ < 9.0 && right_range_ < 9.0 &&
        std::abs(left_range_ - right_range_) >= obstacle_lidar_side_margin_) {
      return left_range_ > right_range_ ? 1.0 : -1.0;
    }
    return (obstacle_center_x_ < image_center_x_) ? -1.0 : 1.0;
  }

  bool obstacle_candidate_near(double front_distance) const {
    const bool lidar_near = use_lidar_safety_ && front_range_ <= front_distance;
    const bool visual_near =
        obstacle_bottom_y_ >= obstacle_image_height_ * obstacle_near_bottom_ratio_;
    return lidar_near || visual_near;
  }

  double obstacle_path_target_x(bool yellow_stage) const {
    if (yellow_stage && latest_lane_yellow_visible_ && latest_lane_center_x_ >= 0.0) {
      return latest_lane_center_x_;
    }
    if (!yellow_stage && has_recent_track()) {
      return latest_track_x_;
    }
    return image_center_x_;
  }

  bool obstacle_blocks_path(bool yellow_stage) const {
    const double tolerance = yellow_stage ? yellow_obstacle_path_tolerance_
                                          : obstacle_path_tolerance_;
    const double target_x = obstacle_path_target_x(yellow_stage);
    const double inflated_tolerance = tolerance + obstacle_width_ * 0.35;
    return std::abs(obstacle_center_x_ - target_x) <= inflated_tolerance;
  }

  bool obstacle_candidate_confirmed(double trigger_area,
                                    bool yellow_stage) const {
    if (!obstacle_seen_ || obstacle_area_ < trigger_area) {
      return false;
    }
    const double front_distance =
        yellow_stage ? yellow_obstacle_confirm_front_distance_
                     : obstacle_confirm_front_distance_;
    return obstacle_blocks_path(yellow_stage) &&
           obstacle_candidate_near(front_distance);
  }

  // 障碍物是否处于活动状态：当前检测到或保持时间内仍认为存在。
  bool obstacle_active() const {
    if (obstacle_candidate_confirmed(obstacle_trigger_area_, false)) {
      return true;
    }
    if (obstacle_hold_seconds_ <= 0.0 ||
        last_obstacle_time_.nanoseconds() == 0) {
      return false;
    }
    return (now() - last_obstacle_time_).seconds() <= obstacle_hold_seconds_;
  }

  bool yellow_obstacle_active() const {
    if (obstacle_candidate_confirmed(yellow_obstacle_trigger_area_, true)) {
      return true;
    }
    if (obstacle_hold_seconds_ <= 0.0 ||
        last_obstacle_time_.nanoseconds() == 0) {
      return false;
    }
    return (now() - last_obstacle_time_).seconds() <= obstacle_hold_seconds_;
  }

  // 前方是否被雷达或深度图判定为阻塞，距离单位米。
  bool front_blocked() const {
    const bool lidar_blocked = use_lidar_safety_ && front_range_ < front_stop_distance_;
    const bool depth_blocked =
        use_depth_safety_ && has_recent_depth() && latest_depth_range_ < depth_stop_distance_;
    return lidar_blocked || depth_blocked;
  }

  // 计算安全速度缩放：雷达和深度两路取更保守结果，范围 0~1。
  double safety_scale() const {
    const double lidar_safety =
        use_lidar_safety_
            ? compute_safety_scale(front_range_, front_stop_distance_, front_slow_distance_)
            : 1.0;
    const double depth_safety =
        (use_depth_safety_ && has_recent_depth())
            ? compute_safety_scale(latest_depth_range_, depth_stop_distance_, depth_slow_distance_)
            : 1.0;
    return combine_safety_scales(lidar_safety, depth_safety);
  }

  // 深度图距离是否新鲜，超时时间 0.35 秒。
  bool has_recent_depth() const {
    return (now() - last_depth_time_).seconds() < 0.35;
  }

  // 前方是否小于指定距离，distance 单位米。
  bool front_within_distance(double distance) const {
    const bool lidar_close = use_lidar_safety_ && front_range_ <= distance;
    const bool depth_close =
        use_depth_safety_ && has_recent_depth() && latest_depth_range_ <= distance;
    return lidar_close || depth_close;
  }

  static double normalize_angle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double rad_to_deg(double radians) {
    return radians * 180.0 / M_PI;
  }

  static double deg_to_rad(double degrees) {
    return degrees * M_PI / 180.0;
  }

  void update_yaw_from_quaternion(double x, double y, double z, double w,
                                  const char *source) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    yaw_source_ = source;
    last_yaw_time_ = now();
  }

  bool has_recent_yaw() const {
    return last_yaw_time_.nanoseconds() != 0 &&
           (now() - last_yaw_time_).seconds() < 0.5;
  }

  void update_distance_integrator() {
    const auto current_time = now();
    if (last_distance_update_time_.nanoseconds() == 0) {
      last_distance_update_time_ = current_time;
      return;
    }
    const double dt = (current_time - last_distance_update_time_).seconds();
    last_distance_update_time_ = current_time;
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }
    distance_traveled_ += last_cmd_linear_x_ * dt;
  }

  void start_qr_entry_turn_if_ready(const char *tag,
                                    double signed_angle_deg) {
    if (qr_turn_started_) {
      return;
    }
    if (!has_recent_yaw()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[%s] waiting yaw from /odom or /imu before turn", tag);
      return;
    }
    qr_turn_started_ = true;
    qr_turn_start_yaw_ = current_yaw_;
    qr_turn_target_yaw_ = normalize_angle(
        qr_turn_start_yaw_ + deg_to_rad(signed_angle_deg));
    RCLCPP_INFO(
        get_logger(),
        "[%s] start_yaw=%.2f target_yaw=%.2f delta=%.2f source=%s",
        tag, rad_to_deg(qr_turn_start_yaw_),
        rad_to_deg(qr_turn_target_yaw_), signed_angle_deg,
        yaw_source_.c_str());
  }

  void publish_qr_entry_done_once() {
    if (qr_turn_done_sent_) {
      return;
    }
    publish_flag(qr_hardcode_done_pub_);
    qr_turn_done_sent_ = true;
  }

  void command_qr_entry_turn(geometry_msgs::msg::Twist &cmd,
                             const char *tag,
                             double signed_angle_deg,
                             double stage_seconds) {
    start_qr_entry_turn_if_ready(tag, signed_angle_deg);
    if (!qr_turn_started_ || stage_seconds < qr_turn_settle_seconds_) {
      cmd = geometry_msgs::msg::Twist();
      return;
    }

    const double error = normalize_angle(qr_turn_target_yaw_ - current_yaw_);
    const double tolerance = deg_to_rad(qr_turn_yaw_tolerance_deg_);
    if (qr_entry_turn_complete(error, tolerance)) {
      cmd = geometry_msgs::msg::Twist();
      publish_qr_entry_done_once();
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[%s] done yaw=%.2f target=%.2f error=%.2f deg",
          tag, rad_to_deg(current_yaw_), rad_to_deg(qr_turn_target_yaw_),
          rad_to_deg(error));
      return;
    }

    const double turn_sign = error >= 0.0 ? 1.0 : -1.0;
    cmd.linear.x = std::clamp(qr_turn_calib_linear_x_, 0.0, 0.15);
    cmd.angular.z =
        turn_sign * std::clamp(std::abs(qr_turn_calib_angular_z_), 0.0, 0.8);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[%s] yaw=%.2f target=%.2f error=%.2f deg cmd=(%.2f,%.2f)",
        tag, rad_to_deg(current_yaw_), rad_to_deg(qr_turn_target_yaw_),
        rad_to_deg(error), cmd.linear.x, cmd.angular.z);
  }

  // 发布一次 Bool=true 事件，供 mission_manager 推进状态机。
  void publish_flag(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    pub->publish(msg);
  }

  // 切换任务阶段时复位一次性发布标志、锁定计数和恢复状态。
  void reset_stage_latches() {
    qr_ready_sent_ = false;
    qr_capture_active_ = false;
    qr_visual_lock_count_ = 0;
    qr_turn_started_ = false;
    qr_turn_done_sent_ = false;
    qr_entry_advance_started_ = false;
    yellow_entry_sent_ = false;
    yellow_entry_lock_ticks_ = 0;
    yellow_entry_detect_ticks_ = 0;
    yellow_gate_guidance_lock_ticks_ = 0;
    yellow_gate_guidance_locked_ = false;
    gate_aligned_sent_ = false;
    loop_complete_sent_ = false;
    yellow_lap_line_return_ticks_ = 0;
    yellow_exit_green_gap_ticks_ = 0;
    yellow_exit_side_yellow_ticks_ = 0;
    yellow_exit_arm_ticks_ = 0;
    yellow_exit_armed_ = false;
    yellow_exit_candidate_seen_ = false;
    yellow_exit_candidate_cleared_ = false;
    parking_lock_ticks_ = 0;
    parking_approach_active_ = false;
    parking_approach_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    park_zone_sent_ = false;
    parked_sent_ = false;
    human_result_sent_ = false;
    direct_yellow_entry_lock_count_ = 0;
    exit_gate_ready_sent_ = false;
    line_reacquired_sent_ = false;
    return_complete_sent_ = false;
    qr_lap_green_recovery_used_ = false;
    line_lost_finish_active_ = false;
    yellow_lane_lost_active_ = false;
    last_yellow_lane_angular_ = 0.0;
    exit_wide_direction_ = 0;
    recovery_yellow_lock_ticks_ = 0;
    obstacle_recovery_candidate_ticks_ = 0;
    grid_recovering_ = false;
    grid_recover_phase_ = 0;
    grid_recover_count_ = 0;
    yellow_enter_confirm_count_ = 0;
    checker_recover_count_ = 0;
    entry_forbidden_recovering_ = false;
    entry_forbidden_reason_.clear();
    forced_approach_gate_done_logged_ = false;
    reset_obstacle_bypass_state();
  }

  // 将恢复阶段枚举转换为日志字符串。
  const char *recovery_phase_name() const {
    switch (recovery_phase_) {
      case RecoveryPhase::IDLE:
        return "IDLE";
      case RecoveryPhase::STOP:
        return "STOP";
      case RecoveryPhase::BACKUP:
        return "BACKUP";
      case RecoveryPhase::LEFT_TURN:
        return "LEFT_TURN";
      case RecoveryPhase::SEARCH_YELLOW:
        return "SEARCH_YELLOW";
    }
    return "IDLE";
  }

  // 哪些任务阶段允许通用恢复逻辑接管控制。
  bool stage_allows_recovery() const {
    return stage_ == "LINE_TO_QR" ||
           stage_ == "FOLLOW_TO_QR" ||
           stage_ == "FORWARD_TO_GATE" ||
           stage_ == "YELLOW_SEARCH" ||
           stage_ == "FORWARD_EXIT_GATE" ||
           stage_ == "LINE_BACK_HOME";
  }

  bool stage_allows_front_obstacle_recovery() const {
    return stage_ == "LINE_TO_QR" ||
           stage_ == "FOLLOW_TO_QR" ||
           stage_ == "FORWARD_TO_GATE" ||
           stage_ == "YELLOW_SEARCH";
  }

  // 当前阶段是否依赖黄区视觉信息。
  bool stage_uses_yellow() const {
    return stage_ == "GO_TO_YELLOW_ENTRY" ||
           stage_ == "FORWARD_TO_GATE" ||
           stage_ == "YELLOW_SEARCH" ||
           stage_ == "YELLOW_LAP";
  }

  // 当前阶段是否处于黄区入口搜索流程。
  bool stage_uses_yellow_entry_search() const {
    return stage_ == "GO_TO_YELLOW_ENTRY" ||
           stage_ == "FORWARD_TO_GATE" ||
           stage_ == "YELLOW_SEARCH";
  }

  // QR 后静态右转保护：白色/格子风险存在时禁止或限制向右打角。
  void block_static_right_steer_after_qr(geometry_msgs::msg::Twist &cmd) {
    // 保护未启用、当前不是黄区入口搜索阶段，或当前没有右转指令时不处理。
    if (!block_static_right_steer_after_qr_ ||
        !stage_uses_yellow_entry_search() ||
        cmd.angular.z >= 0.0) {
      return;
    }

    const double original_angular = cmd.angular.z;
    const bool white_seen = white_seen_detected_now();
    const bool white_danger = white_danger_detected_now();
    const bool checker = forbidden_grid_detected_now();
    const bool forbidden_grid = forbidden_grid_detected_;
    const bool emergency_white =
        white_seen_ratio_now() > white_emergency_threshold_;

    double candidate_center_x = 0.0;
    bool candidate_center_valid = false;
    double grace_elapsed = 0.0;
    const bool grace_candidate =
        yellow_candidate_allow_right_steer_ &&
        post_approach_candidate_grace_allows(grace_elapsed,
                                             candidate_center_x,
                                             candidate_center_valid);
    const bool strong_yellow_candidate =
        yellow_candidate_allow_right_steer_ &&
        strong_yellow_candidate_detected(candidate_center_x,
                                         candidate_center_valid);
    const bool allow_candidate_right_steer =
        strong_yellow_candidate || grace_candidate;
    const bool hard_grid = post_approach_hard_grid_detected();

    // 近处白色/格子或硬格子危险时，直接清零右转角速度。
    if ((!grace_candidate &&
         (white_danger || checker || forbidden_grid)) ||
        emergency_white || hard_grid) {
      cmd.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[SAFETY] right steer blocked reason=%s stage=%s old=%.2f new=0.00",
          emergency_white ? "white_emergency" :
          (hard_grid ? "hard_grid" :
           (white_danger ? "white_danger" :
            (checker ? "checker" : "grid"))),
          stage_.c_str(), original_angular);
      return;
    }

    // 有黄区候选时允许小幅右转，但限制最大右转角速度，单位 rad/s。
    if (allow_candidate_right_steer) {
      const double limited_angular = std::max(
          cmd.angular.z, -std::abs(yellow_candidate_max_right_angular_z_));
      if (limited_angular != cmd.angular.z) {
        cmd.angular.z = limited_angular;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[SAFETY] candidate right steer limited old=%.2f new=%.2f",
            original_angular, cmd.angular.z);
      }
      return;
    }

    // 没有有效黄区候选时禁止静态右打，避免从蓝区误转向白格。
    cmd.angular.z = 0.0;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[SAFETY] right steer blocked reason=%s stage=%s old=%.2f new=0.00",
        white_seen ? "white_seen_no_candidate" : "no_candidate",
        stage_.c_str(), original_angular);
  }

  // 黄区禁入总判断：黄白格、绿色禁区或黄区丢失不安全时返回 true。
  bool forbidden_active(std::string &reason) const {
    // 非黄区相关阶段或没有新鲜黄区信息时，不触发禁入恢复。
    if (!stage_uses_yellow() || !has_recent_yellow_lane_info()) {
      return false;
    }
    // 黄白格禁区：格子标志或格子面积比例超过阈值。
    if (checker_forbidden_enabled_ &&
        (forbidden_grid_detected_now() ||
         latest_grid_center_ratio_ >= grid_forbidden_threshold_ ||
         latest_grid_max_ratio_ >= grid_forbidden_threshold_ ||
         latest_grid_center_ratio_ >= checker_forbidden_area_ratio_ ||
         latest_grid_max_ratio_ >= checker_forbidden_area_ratio_ ||
         (forbidden_grid_detected_ &&
          latest_grid_max_ratio_ >= grid_forbidden_threshold_))) {
      reason = "checker";
      return true;
    }
    // 绿色禁区：中心、左右或总面积比例超过阈值。
    if (green_danger_confirmed_ && green_forbidden_threshold_crossed()) {
      if (stage_ == "YELLOW_LAP" && yellow_obstacle_green_override_active()) {
        return false;
      }
      if (stage_ == "YELLOW_LAP" && !yellow_lap_green_recovery_danger()) {
        return false;
      }
      reason = "green";
      return true;
    }
    // 黄区绕行中黄线丢失且同时看到危险色块，进入不安全恢复。
    if (stage_ == "YELLOW_LAP" &&
        !latest_lane_yellow_visible_ &&
        (yellow_lap_green_recovery_danger() ||
         latest_grid_max_ratio_ > grid_forbidden_threshold_)) {
      if (yellow_obstacle_green_override_active()) {
        return false;
      }
      reason = "yellow_lost_unsafe";
      return true;
    }
    return false;
  }

  // 恢复是否完成：黄区阶段要求黄线稳定，普通阶段要求巡线恢复。
  bool recovery_done() const {
    double lane_center_x = 0.0;
    double lane_width = 0.0;
    if (qr_hardcode_yellow_lap_grace_active_now()) {
      return yellow_lane_center(lane_center_x, lane_width) &&
             latest_yellow_area_ratio_ >= yellow_min_area_ratio_;
    }
    if (stage_uses_yellow()) {
      std::string reason;
      return !forbidden_active(reason) &&
             yellow_lane_center(lane_center_x, lane_width) &&
             latest_yellow_area_ratio_ >= yellow_min_area_ratio_;
    }
    return has_recent_track();
  }

  bool yellow_lap_green_recovery_danger() const {
    if (stage_ != "YELLOW_LAP") {
      return false;
    }
    if (!green_danger_confirmed_) {
      return false;
    }
    return latest_green_center_ratio_ >= yellow_lap_green_recovery_center_ratio_ ||
           latest_green_area_ratio_ >= yellow_lap_green_recovery_area_ratio_ ||
           latest_green_left_ratio_ >= yellow_lap_green_recovery_area_ratio_ ||
           latest_green_right_ratio_ >= yellow_lap_green_recovery_area_ratio_ ||
           (!latest_lane_yellow_visible_ &&
            latest_green_area_ratio_ >= green_forbidden_area_ratio_);
  }

  bool yellow_lap_green_steer_danger() const {
    if (stage_ != "YELLOW_LAP") {
      return false;
    }
    return latest_green_center_ratio_ >=
               yellow_lap_green_recovery_center_ratio_ * 0.75 ||
           latest_green_left_ratio_ >= yellow_lap_green_recovery_area_ratio_ * 0.85 ||
           latest_green_right_ratio_ >= yellow_lap_green_recovery_area_ratio_ * 0.85;
  }

  double yellow_lap_green_escape_angular() const {
    const auto direction =
        (direction_ == "CLOCKWISE")
            ? TurnDirection::CLOCKWISE
            : (direction_ == "COUNTERCLOCKWISE")
                  ? TurnDirection::COUNTERCLOCKWISE
                  : TurnDirection::UNKNOWN;
    if (direction != TurnDirection::UNKNOWN) {
      return yellow_direction_search_angular(
          direction, yellow_loop_green_danger_angular_);
    }
    return latest_green_x_ >= image_center_x_
               ? std::abs(yellow_loop_green_danger_angular_)
               : -std::abs(yellow_loop_green_danger_angular_);
  }

  bool green_forbidden_threshold_crossed() const {
    return latest_green_center_ratio_ >= green_forbidden_threshold_ * 0.5 ||
           latest_green_left_ratio_ >= green_forbidden_threshold_ ||
           latest_green_right_ratio_ >= green_forbidden_threshold_ ||
           latest_green_area_ratio_ >= green_forbidden_threshold_ ||
           latest_green_center_ratio_ >= green_forbidden_area_ratio_ ||
           latest_green_area_ratio_ >= green_forbidden_area_ratio_;
  }

  bool yellow_lap_green_recovery_cooling_down() const {
    if (yellow_lap_green_recovery_cooldown_ <= 0.0 ||
        last_green_recovery_time_.nanoseconds() == 0) {
      return false;
    }
    return (now() - last_green_recovery_time_).seconds() <
           yellow_lap_green_recovery_cooldown_;
  }

  bool yellow_exit_all_yellow_corner_view() const {
    if (!has_recent_yellow_lane_info()) {
      return false;
    }
    return latest_yellow_left_ratio_ >= yellow_exit_side_yellow_threshold_ &&
           latest_yellow_right_ratio_ >= yellow_exit_side_yellow_threshold_ &&
           latest_green_left_ratio_ <= yellow_exit_side_yellow_green_clear_ &&
           latest_green_right_ratio_ <= yellow_exit_side_yellow_green_clear_;
  }

  bool yellow_exit_corner_inhibit_active() const {
    if (yellow_corner_boost_active_ || yellow_exit_all_yellow_corner_view()) {
      return true;
    }
    if (yellow_exit_corner_inhibit_seconds_ <= 0.0 ||
        yellow_corner_last_end_time_.nanoseconds() == 0) {
      return false;
    }
    return (now() - yellow_corner_last_end_time_).seconds() <
           yellow_exit_corner_inhibit_seconds_;
  }

  bool yellow_lap_line_return_ready(double stage_seconds) {
    if (stage_ != "YELLOW_LAP" ||
        stage_seconds < yellow_lap_line_return_min_time_ ||
        !yellow_exit_armed_ ||
        !yellow_exit_candidate_cleared_ ||
        yellow_exit_corner_inhibit_active() ||
        !has_recent_track()) {
      yellow_lap_line_return_ticks_ = 0;
      return false;
    }
    ++yellow_lap_line_return_ticks_;
    return yellow_lap_line_return_ticks_ >= yellow_lap_line_return_ticks_required_;
  }

  void update_yellow_exit_arm(TurnDirection direction, bool lane_ok) {
    if (yellow_exit_armed_) {
      return;
    }
    const bool lane_stable =
        stage_ == "YELLOW_LAP" &&
        has_recent_yellow_lane_info() &&
        lane_ok &&
        latest_lane_yellow_visible_ &&
        latest_yellow_area_ratio_ >= yellow_min_area_ratio_ &&
        latest_lane_width_ >= yellow_entry_lane_width_min_;
    if (!lane_stable) {
      yellow_exit_arm_ticks_ = 0;
      return;
    }

    const double inner_green =
        direction == TurnDirection::CLOCKWISE
            ? latest_green_right_ratio_
            : latest_green_left_ratio_;
    if (inner_green >= yellow_exit_arm_inner_green_threshold_) {
      ++yellow_exit_arm_ticks_;
    } else {
      yellow_exit_arm_ticks_ = 0;
    }

    if (yellow_exit_arm_ticks_ >= yellow_exit_arm_ticks_required_) {
      yellow_exit_armed_ = true;
      RCLCPP_INFO(
          get_logger(),
          "[YELLOW_EXIT] armed direction=%s inner_green=%.3f lane_width=%.1f yellow=%.3f",
          direction == TurnDirection::CLOCKWISE ? "CLOCKWISE" : "COUNTERCLOCKWISE",
          inner_green, latest_lane_width_, latest_yellow_area_ratio_);
    }
  }

  bool yellow_exit_candidate_present(TurnDirection direction) const {
    if (!has_recent_yellow_lane_info()) {
      return false;
    }
    if (yellow_exit_all_yellow_corner_view()) {
      return true;
    }

    return yellow_side_exit_candidate(
        direction, latest_yellow_left_ratio_, latest_yellow_right_ratio_,
        latest_green_left_ratio_, latest_green_right_ratio_,
        yellow_exit_side_yellow_threshold_,
        yellow_exit_side_yellow_green_clear_);
  }

  bool yellow_lap_visual_exit_ready(double stage_seconds,
                                    TurnDirection direction) {
    if (stage_ != "YELLOW_LAP" ||
        stage_seconds < yellow_exit_green_gap_min_time_ ||
        !yellow_exit_armed_ ||
        yellow_exit_corner_inhibit_active() ||
        !has_recent_yellow_lane_info()) {
      yellow_exit_green_gap_ticks_ = 0;
      return false;
    }

    const bool left_green_only =
        latest_green_left_ratio_ >= yellow_exit_green_left_threshold_ &&
        latest_green_right_ratio_ <= yellow_exit_green_right_clear_threshold_;
    const bool right_green_only =
        latest_green_right_ratio_ >= yellow_exit_green_left_threshold_ &&
        latest_green_left_ratio_ <= yellow_exit_green_right_clear_threshold_;
    const bool right_yellow_open =
        latest_yellow_right_ratio_ >= yellow_exit_right_yellow_threshold_;
    const bool left_yellow_open =
        latest_yellow_left_ratio_ >= yellow_exit_right_yellow_threshold_;
    const bool exit_right = left_green_only && right_yellow_open;
    const bool exit_left = right_green_only && left_yellow_open;
    if (!yellow_visual_exit_matches_direction(direction, exit_left,
                                              exit_right)) {
      yellow_exit_green_gap_ticks_ = 0;
      return false;
    }

    // 逆时针从右侧回通道，顺时针从左侧回通道。
    yellow_exit_turn_sign_ =
        direction == TurnDirection::COUNTERCLOCKWISE ? -1.0 : 1.0;
    ++yellow_exit_green_gap_ticks_;
    return yellow_exit_green_gap_ticks_ >= yellow_exit_green_gap_ticks_required_;
  }

  bool yellow_lap_side_yellow_exit_ready(double stage_seconds,
                                         TurnDirection direction) {
    if (stage_ != "YELLOW_LAP" ||
        stage_seconds < yellow_exit_side_yellow_min_time_ ||
        !yellow_exit_armed_ ||
        yellow_exit_corner_inhibit_active() ||
        !has_recent_yellow_lane_info()) {
      yellow_exit_side_yellow_ticks_ = 0;
      return false;
    }

    const bool use_right_side = direction == TurnDirection::COUNTERCLOCKWISE;
    const bool side_exit = yellow_side_exit_candidate(
        direction, latest_yellow_left_ratio_, latest_yellow_right_ratio_,
        latest_green_left_ratio_, latest_green_right_ratio_,
        yellow_exit_side_yellow_threshold_,
        yellow_exit_side_yellow_green_clear_);
    if (!side_exit) {
      yellow_exit_side_yellow_ticks_ = 0;
      return false;
    }
    yellow_exit_turn_sign_ =
        use_right_side ? yellow_exit_yellow_side_turn_sign_
                       : -yellow_exit_yellow_side_turn_sign_;
    ++yellow_exit_side_yellow_ticks_;
    return yellow_exit_side_yellow_ticks_ >=
           yellow_exit_green_gap_ticks_required_;
  }

  // 获取黄区入口可用中心：优先黄车道中心，其次黄色质心或 lane_info 中心。
  bool yellow_entry_center_valid(double &center_x) const {
    double lane_center_x = 0.0;
    double lane_width = 0.0;
    if (yellow_lane_center(lane_center_x, lane_width)) {
      center_x = lane_center_x;
      return true;
    }
    if (has_recent_yellow() && latest_yellow_area_ratio_ >= yellow_soft_seen_threshold_) {
      center_x = latest_yellow_x_;
      return true;
    }
    if (has_recent_yellow_lane_info() &&
        latest_yellow_area_ratio_ >= yellow_soft_seen_threshold_) {
      center_x = latest_lane_center_x_;
      return true;
    }
    return false;
  }

  // 黄区入口禁入恢复：白色/格子危险时后退、停车、强制左转、再接近黄色。
  bool apply_entry_forbidden_recovery(geometry_msgs::msg::Twist &cmd) {
    std::string visual_reason;
    std::string reason;
    const bool visual = entry_forbidden_visual_detected(visual_reason);
    const double white_seen_ratio = white_seen_ratio_now();
    const double white_danger_ratio = white_danger_ratio_now();
    const bool danger = entry_forbidden_danger_detected(reason);
    const bool emergency_white =
        white_seen_ratio > white_emergency_threshold_;
    const bool hard_grid = post_approach_hard_grid_detected();
    const bool in_forced_approach_window =
        (stage_ == "FORWARD_TO_GATE" || stage_ == "YELLOW_SEARCH") &&
        forced_approach_gate_duration_ > 0.0 &&
        (now() - stage_enter_time_).seconds() < forced_approach_gate_duration_;
    double candidate_center_x = 0.0;
    bool candidate_center_valid = false;
    (void)strong_yellow_candidate_detected(candidate_center_x,
                                           candidate_center_valid);
    double grace_elapsed = 0.0;
    double grace_center_x = 0.0;
    bool grace_center_valid = false;
    const bool post_approach_grace_active =
        post_approach_candidate_grace_active(grace_elapsed);
    const bool grace_candidate_allowed =
        post_approach_candidate_grace_allows(grace_elapsed, grace_center_x,
                                             grace_center_valid);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[WHITE_STATE] seen_ratio=%.3f seen=%d danger_ratio=%.3f danger=%d seen_roi=%.2f~%.2f danger_roi=%.2f~%.2f",
        white_seen_ratio, white_seen_detected_now(),
        white_danger_ratio, white_danger_detected_now(),
        white_seen_roi_start_ratio_, white_seen_roi_end_ratio_,
        white_danger_roi_start_ratio_, white_danger_roi_end_ratio_);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[FORBIDDEN_VISUAL] visual=%d reason=%s grid=%d checker=%d white_seen=%d",
        visual, visual ? visual_reason.c_str() : "none",
        forbidden_grid_detected_, forbidden_grid_detected_now(),
        white_seen_detected_now());
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[FORBIDDEN_DANGER] close=%d reason=%s white=%.3f grid_center=%.3f grid_max=%.3f threshold=%.3f soft_threshold=%.3f",
        danger, danger ? reason.c_str() : "none", white_seen_ratio,
        latest_grid_center_ratio_, latest_grid_max_ratio_,
        entry_close_grid_threshold_, entry_close_grid_soft_threshold_);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[GRID] forbidden_grid=%d checker=%d yellow_area=%.3f",
        forbidden_grid_detected_, forbidden_grid_detected_now(),
        latest_yellow_area_ratio_);
    if (post_approach_grace_active) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[POST_APPROACH_GRACE] active=1 elapsed=%.2f area=%.3f center_valid=%d grid_center=%.3f grid_max=%.3f",
          grace_elapsed, latest_yellow_area_ratio_, grace_center_valid,
          latest_grid_center_ratio_, latest_grid_max_ratio_);
    }

    // 强制接近黄门口的早期窗口内，除紧急白色外先忽略近距离危险。
    if (!entry_forbidden_recovering_ && in_forced_approach_window && danger &&
        reason != "white_emergency") {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[FORCED_APPROACH_GATE] close danger ignored during approach white=%.3f grid_center=%.3f grid_max=%.3f",
          white_seen_ratio, latest_grid_center_ratio_, latest_grid_max_ratio_);
      return false;
    }

    // 前探宽限期内若仍有黄区候选，允许继续尝试，不马上恢复。
    if (!entry_forbidden_recovering_ && danger && post_approach_grace_active) {
      if (emergency_white) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[POST_APPROACH_GRACE] blocked reason=emergency_white");
      } else if (hard_grid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[POST_APPROACH_GRACE] blocked reason=hard_grid");
      } else if (grace_candidate_allowed) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[POST_APPROACH_GRACE] allow yellow_candidate despite close_danger");
        return false;
      }
    }

    // 没有危险且不在恢复中时，不接管控制。
    if (!danger && !entry_forbidden_recovering_) {
      return false;
    }

    const bool yellow_signal =
        latest_yellow_area_ratio_ > yellow_candidate_area_threshold_ ||
        candidate_center_valid || latest_lane_yellow_visible_;
    if (!entry_forbidden_recovering_ && danger && yellow_signal) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[YELLOW_CANDIDATE] blocked reason=%s area=%.3f center_valid=%d",
          reason.c_str(), latest_yellow_area_ratio_, candidate_center_valid);
    }

    // 首次确认危险：启动入口禁入恢复并清空黄区入口确认帧。
    if (danger && !entry_forbidden_recovering_) {
      yellow_candidate_active_ = false;
      entry_forbidden_recovering_ = true;
      entry_forbidden_reason_ = reason;
      entry_forbidden_recover_start_time_ = now();
      yellow_entry_detect_ticks_ = 0;
      RCLCPP_WARN(get_logger(),
                  "[FORBIDDEN_RECOVER] start reason=%s",
                  reason.c_str());
    }

    if (!entry_forbidden_recovering_) {
      return false;
    }

    const double elapsed = (now() - entry_forbidden_recover_start_time_).seconds();
    const double backup_end = forbidden_backup_duration_;
    const double stop_end = backup_end + forbidden_stop_duration_;
    const double turn_end = stop_end + forced_left_turn_duration_;
    const double approach_end = turn_end + approach_yellow_duration_;
    cmd = geometry_msgs::msg::Twist();
    yellow_entry_detect_ticks_ = 0;

    // 阶段 1：后退远离白色/黄白格，速度单位 m/s，时间单位秒。
    if (elapsed < backup_end) {
      cmd.linear.x = -std::abs(forbidden_backup_speed_);
      cmd.angular.z = 0.0;
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[FORBIDDEN_RECOVER] phase=BACKUP elapsed=%.2f cmd=(%.2f,%.2f)",
          elapsed, cmd.linear.x, cmd.angular.z);
      return true;
    }

    // 阶段 2：短暂停车，等待车身稳定。
    if (elapsed < stop_end) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[FORBIDDEN_RECOVER] phase=STOP elapsed=%.2f cmd=(%.2f,%.2f)",
          elapsed - backup_end, cmd.linear.x, cmd.angular.z);
      return true;
    }

    // 阶段 3：强制左转，避免继续朝右侧白格/禁区进入。
    if (elapsed < turn_end) {
      cmd.linear.x = std::abs(forced_left_linear_speed_);
      cmd.angular.z = std::abs(forced_left_angular_z_);
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[FORBIDDEN_RECOVER] phase=FORCED_LEFT_TURN elapsed=%.2f cmd=(%.2f,%.2f)",
          elapsed - stop_end, cmd.linear.x, cmd.angular.z);
      return true;
    }

    // 阶段 4：低速重新接近黄色通道；若白色紧急超阈值则重新后退。
    if (elapsed < approach_end) {
      const double approach_elapsed = elapsed - turn_end;
      if (white_seen_ratio > white_emergency_threshold_) {
        entry_forbidden_reason_ = "white_emergency";
        entry_forbidden_recover_start_time_ = now();
        cmd.linear.x = -std::abs(forbidden_backup_speed_);
        cmd.angular.z = 0.0;
        RCLCPP_WARN(
            get_logger(),
            "[FORBIDDEN_RECOVER] approach interrupted reason=white_emergency white=%.3f threshold=%.3f",
            white_seen_ratio, white_emergency_threshold_);
        RCLCPP_INFO(
            get_logger(),
            "[FORBIDDEN_RECOVER] phase=BACKUP elapsed=0.00 cmd=(%.2f,%.2f)",
            cmd.linear.x, cmd.angular.z);
        return true;
      }
      cmd.linear.x = std::abs(approach_yellow_speed_);
      cmd.angular.z = std::abs(approach_yellow_angular_z_);
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[FORBIDDEN_RECOVER] phase=APPROACH_YELLOW elapsed=%.2f cmd=(%.2f,%.2f)",
          approach_elapsed, cmd.linear.x, cmd.angular.z);
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[YELLOW] during_approach yellow_area=%.3f white=%.3f",
          latest_yellow_area_ratio_, white_seen_ratio);
      return true;
    }

    // 恢复动作全部结束，交还黄区搜索逻辑。
    entry_forbidden_recovering_ = false;
    entry_forbidden_reason_.clear();
    RCLCPP_INFO(get_logger(),
                "[FORBIDDEN_RECOVER] approach done -> resume yellow search");
    return false;
  }

  bool qr_hardcode_yellow_grace_active(double stage_seconds) const {
    return stage_ == "YELLOW_SEARCH" &&
           previous_stage_ == "QR_HARDCODE_ENTRY" &&
           qr_hardcode_yellow_grace_duration_ > 0.0 &&
           stage_seconds < qr_hardcode_yellow_grace_duration_;
  }

  bool qr_hardcode_forbidden_recovery_disabled() const {
    return stage_ == "YELLOW_SEARCH" &&
           previous_stage_ == "QR_HARDCODE_ENTRY" &&
           qr_hardcode_disable_forbidden_recovery_;
  }

  bool qr_hardcode_yellow_entry_force_ready() const {
    return stage_ == "YELLOW_SEARCH" &&
           previous_stage_ == "QR_HARDCODE_ENTRY" &&
           qr_hardcode_yellow_entry_force_ratio_ > 0.0 &&
           latest_yellow_area_ratio_ >= qr_hardcode_yellow_entry_force_ratio_;
  }

  bool qr_hardcode_yellow_lap_grace_active(double stage_seconds) const {
    return stage_ == "YELLOW_LAP" &&
           qr_hardcode_yellow_path_active_ &&
           qr_hardcode_yellow_lap_grace_duration_ > 0.0 &&
           stage_seconds < qr_hardcode_yellow_lap_grace_duration_;
  }

  bool qr_hardcode_yellow_lap_grace_active_now() const {
    return qr_hardcode_yellow_lap_grace_active(
        (now() - stage_enter_time_).seconds());
  }

  // 强制前进接近黄区门口：用于 QR 后无稳定入口信息时的短时间前探。
  bool command_forced_approach_gate(geometry_msgs::msg::Twist &cmd,
                                    double stage_seconds) {
    // duration <= 0 表示关闭强制前探。
    if (forced_approach_gate_duration_ <= 0.0) {
      return false;
    }

    // 前探窗口内输出固定低速和小角速度；遇到紧急白色立即停车。
    if (stage_seconds < forced_approach_gate_duration_) {
      cmd = geometry_msgs::msg::Twist();
      const double white_seen_ratio = white_seen_ratio_now();
      std::string visual_reason;
      const bool visual = entry_forbidden_visual_detected(visual_reason);
      if (white_seen_ratio > white_emergency_threshold_) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[FORCED_APPROACH_GATE] emergency interrupt white=%.3f",
            white_seen_ratio);
        return true;
      }

      if (visual) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[FORCED_APPROACH_GATE] visual forbidden ignored during approach reason=%s",
            visual_reason.c_str());
      }
      cmd.linear.x = std::abs(forced_approach_gate_speed_);
      cmd.angular.z = forced_approach_gate_angular_z_;
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[FORCED_APPROACH_GATE] elapsed=%.2f cmd=(%.2f,%.2f)",
          stage_seconds, cmd.linear.x, cmd.angular.z);
      return true;
    }

    // 前探结束后打开候选宽限窗口，允许短时间内识别黄区候选。
    if (!forced_approach_gate_done_logged_) {
      forced_approach_gate_done_logged_ = true;
      post_approach_candidate_grace_start_time_ = now();
      RCLCPP_INFO(get_logger(),
                  "[FORCED_APPROACH_GATE] done -> normal yellow search");
    }
    return false;
  }

  // 黄区入口搜索控制：在候选黄通道、白色/格子拒绝和弧形搜索之间切换。
  void command_yellow_entry_search(geometry_msgs::msg::Twist &cmd,
                                   const char *tag) {
    cmd = geometry_msgs::msg::Twist();
    const double yellow_area = latest_yellow_area_ratio_;
    std::string reject_reason;
    const bool rejected = entry_yellow_rejected(reject_reason);
    const bool white_seen_only =
        white_seen_detected_now() && !white_danger_detected_now();
    double center_x = 0.0;
    bool center_valid = false;
    const bool strong_yellow_candidate =
        strong_yellow_candidate_detected(center_x, center_valid);
    double grace_elapsed = 0.0;
    double grace_center_x = 0.0;
    bool grace_center_valid = false;
    const bool grace_candidate_allowed =
        post_approach_candidate_grace_allows(grace_elapsed, grace_center_x,
                                             grace_center_valid);
    const bool candidate_ready =
        strong_yellow_candidate || grace_candidate_allowed;
    if (grace_candidate_allowed) {
      center_x = grace_center_x;
      center_valid = grace_center_valid;
    }
    const bool allowed = !rejected && center_valid &&
                         yellow_area > yellow_soft_seen_threshold_;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[YELLOW_TARGET] allowed=%d strong=%d yellow_area=%.3f center_valid=%d white_seen=%d white_danger=%d grid=%d reject=%s",
        allowed, strong_yellow_candidate, yellow_area, center_valid,
        white_seen_detected_now(), white_danger_detected_now(),
        forbidden_grid_detected_now(), rejected ? reject_reason.c_str() : "none");

    // 已找到可用黄区中心：低速前进并按中心误差闭环。
    if (allowed) {
      yellow_candidate_active_ = false;
      const double yellow_error = center_x - image_center_x_;
      cmd.linear.x = forward_to_gate_speed_;
      cmd.angular.z = std::clamp(
          yellow_entry_turn_sign_ * yellow_entry_kp_ * yellow_error,
          -yellow_entry_max_angular_z_, yellow_entry_max_angular_z_);
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[%s] mode=yellow_center_follow center=%.1f error=%.1f cmd=(%.2f,%.2f)",
          tag, center_x, yellow_error, cmd.linear.x, cmd.angular.z);
      return;
    }

    // 黄区候选首次出现：进入短时前探状态，防止一帧信号被错过。
    if (candidate_ready && !yellow_candidate_active_) {
      yellow_candidate_active_ = true;
      yellow_candidate_start_time_ = now();
    }

    // 候选前探：在限定时间内继续小速度前进，并根据中心微调方向。
    if (yellow_candidate_active_) {
      const double elapsed = (now() - yellow_candidate_start_time_).seconds();
      if (!candidate_ready ||
          white_seen_ratio_now() > white_emergency_threshold_) {
        yellow_candidate_active_ = false;
      } else if (elapsed < yellow_candidate_duration_) {
        cmd.linear.x = yellow_candidate_forward_speed_;
        const double yellow_error = center_x - image_center_x_;
        if (center_valid) {
          cmd.angular.z = std::clamp(
              yellow_candidate_kp_ * yellow_error,
              -std::abs(yellow_candidate_max_right_angular_z_),
              std::abs(yellow_candidate_left_angular_z_));
        } else {
          cmd.angular.z = yellow_candidate_left_angular_z_;
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[YELLOW_CANDIDATE] strong=%d area=%.3f center_valid=%d white_seen=%d white_danger=%d grid=%d error=%.1f cmd=(%.2f,%.2f)",
            candidate_ready, yellow_area, center_valid,
            white_seen_detected_now(), white_danger_detected_now(),
            forbidden_grid_detected_now(), yellow_error, cmd.linear.x,
            cmd.angular.z);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[YELLOW_CANDIDATE] go_forward elapsed=%.2f cmd=(%.2f,%.2f)",
            elapsed, cmd.linear.x, cmd.angular.z);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[%s] mode=yellow_candidate_go_forward cmd=(%.2f,%.2f)",
            tag, cmd.linear.x, cmd.angular.z);
        return;
      } else {
        yellow_candidate_active_ = false;
      }
    }

    // 只看到白色但没有危险白色时，低速左搜索，避免直接把白色当入口。
    if (!candidate_ready &&
        (white_seen_only || (rejected && !white_danger_detected_now()))) {
      cmd.linear.x = yellow_search_linear_speed_;
      cmd.angular.z = yellow_search_angular_z_;
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[%s] mode=white_seen_left_search cmd=(%.2f,%.2f) reject=%s",
          tag, cmd.linear.x, cmd.angular.z,
          rejected ? reject_reason.c_str() : "white_seen");
      return;
    }

    // 默认黄区搜索：低速弧形搜索黄色通道入口。
    cmd.linear.x = yellow_search_linear_speed_;
    cmd.angular.z = yellow_search_angular_z_;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[%s] mode=arc_search cmd=(%.2f,%.2f) yellow_area=%.3f grid=%d",
        tag, cmd.linear.x, cmd.angular.z, yellow_area,
        forbidden_grid_detected_now());
  }

  // 更新黄区入口确认帧数：连续满足面积、中心和无禁入后才发布入口到达。
  bool update_yellow_entry_confirm() {
    std::string forbidden_reason;
    const bool entry_rejected = entry_yellow_rejected(forbidden_reason);
    double center_x = 0.0;
    bool center_valid = false;
    const bool strong_yellow_candidate =
        strong_yellow_candidate_detected(center_x, center_valid);
    std::string reject_reason;
    if (entry_rejected) {
      reject_reason = forbidden_reason;
    } else if (latest_yellow_area_ratio_ <= yellow_entry_area_threshold_) {
      reject_reason = "area";
    } else if (!center_valid) {
      reject_reason = "no_center";
    }

    const bool reached = !entry_rejected && center_valid &&
                         latest_yellow_area_ratio_ > yellow_entry_area_threshold_;
    // 满足入口条件累加帧数，否则清零，避免单帧误触发。
    if (reached) {
      ++yellow_entry_detect_ticks_;
    } else {
      yellow_entry_detect_ticks_ = 0;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[YELLOW_ENTRY_REJECT] reason=%s",
          reject_reason.empty() ? "unknown" : reject_reason.c_str());
    }
    const bool confirmed = yellow_entry_detect_ticks_ >= yellow_entry_confirm_frames_;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[YELLOW_ENTRY_CHECK] confirm=%d/%d strong_candidate=%d white=%.3f seen=%d danger=%d yellow_area=%.3f reached=%d",
        yellow_entry_detect_ticks_, yellow_entry_confirm_frames_,
        strong_yellow_candidate, latest_white_near_ratio_, white_seen_detected_now(),
        white_danger_detected_now(), latest_yellow_area_ratio_, confirmed);
    return confirmed;
  }

  // 启动通用恢复流程；黄白格由专用 grid_recovery 处理。
  void start_recovery(const std::string &reason) {
    if (reason == "checker") {
      return;
    }
    recovery_active_ = true;
    recovery_phase_ = RecoveryPhase::STOP;
    recovery_start_time_ = now();
    recovery_reason_ = reason;
    recovery_yellow_lock_ticks_ = 0;
    if (reason == "obstacle") {
      ++obstacle_recovery_count_;
    } else if (reason == "green") {
      last_green_recovery_time_ = now();
    }
    RCLCPP_WARN(get_logger(), "[RACE] recovery_start reason=%s count=%d",
                recovery_reason_.c_str(), obstacle_recovery_count_);
  }

  // 根据当前阶段、禁区和障碍物状态，决定是否启动恢复流程。
  void maybe_start_recovery() {
    // 已在恢复或当前阶段不允许恢复时不重复启动。
    if (recovery_active_ || !stage_allows_recovery()) {
      return;
    }
    if (qr_hardcode_yellow_lap_grace_active_now()) {
      return;
    }
    if (grid_recovering_) {
      return;
    }

    // 黄白格不走通用恢复，避免转向进入格子。
    std::string reason;
    if (forbidden_active(reason)) {
      if (reason == "checker") {
        return;
      }
      if (reason == "green" && yellow_lap_green_recovery_cooling_down()) {
        return;
      }
      start_recovery(reason);
      return;
    }

    // 视觉桶锥只作为顺滑绕行偏置处理，不触发通用后退恢复。
    // 雷达恢复只留给大厅/入口阶段；黄区和返程里后退左转会破坏路线。
    const bool front_obstacle =
        front_blocked() && !qr_seen_ && stage_allows_front_obstacle_recovery();
    if (front_obstacle) {
      ++obstacle_recovery_candidate_ticks_;
    } else {
      obstacle_recovery_candidate_ticks_ = 0;
    }

    if (obstacle_recovery_candidate_ticks_ >= obstacle_recovery_detect_frames_) {
      start_recovery("front_obstacle");
      obstacle_recovery_candidate_ticks_ = 0;
    }
  }

  // 执行通用恢复状态机：停车、后退、左转、搜索黄线/巡线。
  void apply_recovery(geometry_msgs::msg::Twist &cmd) {
    // 未处于恢复状态时不接管控制。
    if (!recovery_active_) {
      return;
    }

    const bool obstacle_reason =
        recovery_reason_ == "obstacle" || recovery_reason_ == "front_obstacle";
    const double stop_duration =
        obstacle_reason ? obstacle_stop_duration_ : forbidden_stop_duration_;
    const double backup_duration =
        obstacle_reason ? obstacle_backup_duration_ : forbidden_backup_duration_;
    const double backup_speed =
        obstacle_reason ? obstacle_backup_speed_ : forbidden_backup_speed_;
    const double turn_duration =
        obstacle_reason ? obstacle_turn_duration_ : forbidden_turn_duration_;
    const double turn_speed =
        obstacle_reason ? obstacle_turn_speed_ : forbidden_turn_speed_;
    const auto yellow_direction =
        (direction_ == "CLOCKWISE")
            ? TurnDirection::CLOCKWISE
            : (direction_ == "COUNTERCLOCKWISE")
                  ? TurnDirection::COUNTERCLOCKWISE
                  : TurnDirection::UNKNOWN;
    const bool yellow_direction_locked_recovery =
        stage_ == "YELLOW_LAP" && !obstacle_reason &&
        (recovery_reason_ == "green" ||
         recovery_reason_ == "yellow_lost_unsafe");
    const double recovery_turn =
        yellow_direction_locked_recovery
            ? yellow_direction_search_angular(yellow_direction, turn_speed)
            : std::abs(turn_speed);

    const double elapsed = (now() - recovery_start_time_).seconds();
    const double backup_end = stop_duration + backup_duration;
    const double turn_end = backup_end + turn_duration;

    cmd = geometry_msgs::msg::Twist();
    // 阶段 STOP：输出零速度，等待车体停止。
    if (elapsed < stop_duration) {
      recovery_phase_ = RecoveryPhase::STOP;
    // 阶段 BACKUP：按配置速度倒车远离障碍或禁区。
    } else if (elapsed < backup_end) {
      recovery_phase_ = RecoveryPhase::BACKUP;
      cmd.linear.x = -std::abs(backup_speed);
    // 阶段 LEFT_TURN：小速度左转，尝试脱离障碍/禁区。
    } else if (elapsed < turn_end) {
      recovery_phase_ = RecoveryPhase::LEFT_TURN;
      cmd.linear.x = std::min(yellow_lane_safe_speed_, 0.06);
      cmd.angular.z = recovery_turn;
    // 阶段 SEARCH_YELLOW：恢复后重新寻找黄线或普通巡线。
    } else {
      recovery_phase_ = RecoveryPhase::SEARCH_YELLOW;
      cmd.linear.x = std::min(yellow_entry_search_speed_, 0.08);
      cmd.angular.z =
          yellow_direction_locked_recovery
              ? yellow_direction_search_angular(yellow_direction,
                                                yellow_entry_search_angular_)
              : std::abs(yellow_entry_search_angular_);

      std::string forbidden_reason;
      if (!qr_hardcode_yellow_lap_grace_active_now() &&
          forbidden_active(forbidden_reason) &&
          !(forbidden_reason == "green" &&
            yellow_lap_green_recovery_cooling_down())) {
        recovery_start_time_ = now();
        recovery_phase_ = RecoveryPhase::STOP;
        recovery_reason_ = forbidden_reason;
        recovery_yellow_lock_ticks_ = 0;
        cmd = geometry_msgs::msg::Twist();
      } else if (stage_uses_yellow()) {
        if (elapsed >= turn_end + yellow_recovery_search_min_seconds_ &&
            recovery_done()) {
          ++recovery_yellow_lock_ticks_;
        } else {
          recovery_yellow_lock_ticks_ = 0;
        }
      } else {
        recovery_yellow_lock_ticks_ = recovery_done() ? yellow_valid_frames_required_ : 0;
      }

      if (recovery_yellow_lock_ticks_ >= yellow_valid_frames_required_) {
        // 连续满足恢复完成条件后退出恢复，避免单帧抖动。
        if (recovery_reason_ == "green") {
          last_green_recovery_time_ = now();
        }
        recovery_active_ = false;
        recovery_phase_ = RecoveryPhase::IDLE;
        RCLCPP_INFO(get_logger(),
                    "[RACE] recovery_done reason=%s elapsed=%.2f yellow_lock=%d/%d track=%d",
                    recovery_reason_.c_str(), elapsed, recovery_yellow_lock_ticks_,
                    yellow_valid_frames_required_, has_recent_track());
      }
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[RACE] state=%s reason=%s cmd=(%.2f,%.2f) elapsed=%.2f",
        recovery_phase_name(), recovery_reason_.c_str(), cmd.linear.x,
        cmd.angular.z, elapsed);
  }

  // 调试模式下发布黄区到达事件，直接推进到停车完成相关流程。
  void publish_debug_park_finish(const char *reason) {
    if (yellow_entry_sent_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "[MISSION] FORWARD_TO_YELLOW -> PARK_FINISH reason=%s",
                reason);
    publish_flag(yellow_entry_pub_);
    yellow_entry_sent_ = true;
  }

  // 调试用黄区前进：只在纯黄通道确认后发布入口到达，否则搜索/后退格子。
  void handle_debug_forward_to_yellow(geometry_msgs::msg::Twist &cmd,
                                      double stage_seconds,
                                      double safety) {
    double lane_center_x = 0.0;
    double lane_width = 0.0;
    const bool lane_guidance_ok = yellow_lane_center(lane_center_x, lane_width);

    if (apply_grid_recovery(cmd)) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[CHECKER_RECOVER] phase=%s elapsed=%.2f count=%d",
          grid_recover_phase_ == 1 ? "backup" : "resume_search",
          (now() - grid_recover_start_time_).seconds(),
          checker_recover_count_);
      return;
    }

    // 超过最大前进时间后停车并发布调试完成，防止无限前进。
    if (stage_seconds > forward_to_yellow_max_time_) {
      cmd = geometry_msgs::msg::Twist();
      RCLCPP_WARN(get_logger(), "[SAFETY] forward_to_yellow timeout, stop");
      publish_debug_park_finish("timeout");
      return;
    }

    const bool pure_yellow = pure_yellow_lane_detected();
    // 检测到纯黄色通道时按通道中心闭环前进。
    if (pure_yellow) {
      const double yellow_center_x = lane_guidance_ok ? lane_center_x : latest_yellow_x_;
      const double yellow_error = yellow_center_x - image_center_x_;
      cmd.linear.x = std::clamp(forward_to_yellow_speed_ * safety, -0.15, 0.15);
      cmd.angular.z =
          std::clamp(-yellow_center_kp_ * yellow_error, -0.25, 0.25);

      // 纯黄面积超过阈值并持续多帧，认为已经进入黄区。
      if (stage_seconds > forward_to_yellow_min_time_ &&
          latest_yellow_area_ratio_ > yellow_enter_area_threshold_) {
        ++yellow_enter_confirm_count_;
      } else {
        yellow_enter_confirm_count_ = 0;
      }

      if (yellow_enter_confirm_count_ >= yellow_enter_confirm_frames_) {
        cmd = geometry_msgs::msg::Twist();
        publish_debug_park_finish(debug_stop_after_enter_yellow_
                                      ? "entered_pure_yellow"
                                      : "entered_pure_yellow_continue");
      }
    } else {
      // 未检测到纯黄通道时停前进，只按搜索角速度寻找黄区。
      yellow_enter_confirm_count_ = 0;
      cmd.linear.x = 0.0;
      cmd.angular.z = yellow_search_angular_z_;
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[FORWARD_TO_YELLOW] elapsed=%.2f yellow_area=%.3f confirm=%d/%d cmd=(%.2f,%.2f)",
        stage_seconds, latest_yellow_area_ratio_, yellow_enter_confirm_count_,
        yellow_enter_confirm_frames_, cmd.linear.x, cmd.angular.z);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[GRID] yellow_near=%.3f white_near=%.3f detected=%d",
        latest_yellow_near_ratio_, latest_white_near_ratio_,
        forbidden_grid_detected_now());
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[YELLOW] pure=%d yellow_area=%.3f white_near=%.3f",
        pure_yellow, latest_yellow_area_ratio_, latest_white_near_ratio_);
  }

  // 主控制周期：每 50 ms 根据 mission stage、感知状态和安全状态输出 /cmd_vel。
  void step() {
    update_distance_integrator();
    geometry_msgs::msg::Twist cmd;
    const double safety = safety_scale();
    const double stage_seconds = (now() - stage_enter_time_).seconds();

    // 阶段：沿线行驶到二维码，负责巡线、二维码减速对准和停车触发。
    if (stage_ == "LINE_TO_QR" || stage_ == "FOLLOW_TO_QR") {
      const double obstacle = obstacle_avoidance();
      // QR 面积连续达到 fallback 阈值后累加视觉锁定帧数。
      if (qr_seen_ && qr_area_ >= qr_fallback_area_) {
        ++qr_visual_lock_count_;
      } else {
        qr_visual_lock_count_ = 0;
      }

      // QR 停车：面积/连帧/解码+可见/激光(仅与 qr 同时见，避免无 qr 时误停。
      // 薄立牌在激光上往往 invisible，主要靠降速+对准+面积阈值，勿依赖 front 单独判停。
      const bool visual_lock_candidate =
          qr_visual_stop_candidate(qr_text_locked_,
                                   qr_seen_,
                                   qr_area_,
                                   qr_visual_lock_count_,
                                   qr_fallback_area_,
                                   qr_visual_lock_ticks_required_);
      const bool qr_stop_condition = visual_lock_candidate ||
                                     (qr_seen_ && qr_area_ >= qr_stop_area_) ||
                                     (qr_text_locked_ && qr_seen_ && qr_area_ >= qr_fallback_area_) ||
                                     (front_blocked() && qr_seen_);

      // 已经发布 QR 到达后保持停车，等待 mission_manager 切阶段。
      if (qr_ready_sent_) {
        cmd = geometry_msgs::msg::Twist();
      // QR 捕获中或满足停车条件：停车等待解码/保持时间，然后发布 qr_reached。
      } else if (qr_capture_active_ || qr_stop_condition) {
        if (!qr_capture_active_) {
          qr_capture_active_ = true;
          qr_capture_start_time_ = now();
        }
        cmd = geometry_msgs::msg::Twist();
        const double stop_elapsed = (now() - qr_capture_start_time_).seconds();
        const bool ready_to_leave =
            qr_text_locked_ && stop_elapsed >= qr_capture_hold_seconds_;
        if (should_trigger_once(ready_to_leave, qr_ready_sent_)) {
          publish_flag(qr_ready_pub_);
          qr_ready_sent_ = true;
        } else if (!qr_text_locked_) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "[QR] stop_and_wait_decode elapsed=%.2f qr_seen=%d area=%.0f",
              stop_elapsed, qr_seen_, qr_area_);
        }
      } else {
        qr_capture_active_ = false;
        // 未停车：远跟线；近对二维码降速与对准
        const bool near_qr_for_brake =
            qr_seen_ && qr_area_ >= qr_approach_brake_start_area_;
        const bool should_align_qr = false;
        const bool has_track = has_recent_track();
        // 接近二维码时降速爬行；无巡线且没看到 QR 时按丢线速度处理。
        if (near_qr_for_brake) {
          cmd.linear.x = qr_approach_creep_speed_ * safety;
        } else if (!has_track && !qr_seen_) {
          cmd.linear.x = line_lost_speed_ * safety;
        } else {
          cmd.linear.x = follow_speed_ * safety;
        }
        // 障碍物优先避让，其次 QR 对准，再普通巡线或丢线搜索。
        if (obstacle_active() ||
            line_obstacle_phase_ != ObstacleBypassPhase::IDLE) {
          cmd.linear.x = std::min(cmd.linear.x, obstacle_avoid_speed_ * safety);
          if (line_obstacle_phase_ == ObstacleBypassPhase::RETURNING) {
            cmd.linear.x = std::min(cmd.linear.x, back_line_speed_ * safety);
          }
          cmd.angular.z = line_obstacle_bypass_angular(line_angular());
        } else if (should_align_qr) {
          const double to_qr = std::clamp(
              (image_center_x_ - qr_center_x_) * qr_align_kp_, -0.75, 0.75);
          const double line_w = qr_line_blend_;
          cmd.angular.z = line_w * line_angular() + (1.0 - line_w) * to_qr + obstacle;
        } else if (!has_track && !qr_seen_) {
          cmd.angular.z = line_lost_search_angular_;
        } else {
          cmd.angular.z = line_angular() + obstacle;
        }
      }
    // 阶段：二维码解码/任务解析期间保持静止。
    } else if (stage_ == "QR_DETECT" || stage_ == "PARSE_QR_TASK") {
      cmd = geometry_msgs::msg::Twist();
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[MOTION] stage=QR_DETECT stop_for_qr_parse");
    // 无后退入口第一段：固定左转，里程计/IMU 到达目标角度后推进状态机。
    } else if (stage_ == "QR_HARDCODE_TURN_1") {
      command_qr_entry_turn(cmd, "QR_TURN_1",
                            std::abs(turn1_angle_deg_), stage_seconds);
    // 无后退入口第二段：按累计前进距离闭环，不依赖固定执行时间。
    } else if (stage_ == "QR_ENTRY_ADVANCE") {
      if (!qr_entry_advance_started_) {
        qr_entry_advance_started_ = true;
        qr_entry_advance_start_distance_ = distance_traveled_;
        RCLCPP_INFO(
            get_logger(),
            "[QR_ENTRY_ADVANCE] start distance=%.3f target_delta=%.3f",
            qr_entry_advance_start_distance_, entry_advance_distance_);
      }
      const double advanced =
          distance_traveled_ - qr_entry_advance_start_distance_;
      if (qr_entry_advance_complete(advanced, entry_advance_distance_)) {
        cmd = geometry_msgs::msg::Twist();
        publish_qr_entry_done_once();
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[QR_ENTRY_ADVANCE] done advanced=%.3f target=%.3f",
            advanced, entry_advance_distance_);
      } else {
        cmd.linear.x = std::clamp(entry_advance_speed_, 0.0, 0.15);
        cmd.angular.z = 0.0;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[QR_ENTRY_ADVANCE] advanced=%.3f/%.3f cmd=(%.2f,%.2f)",
            advanced, entry_advance_distance_, cmd.linear.x, cmd.angular.z);
      }
    // 无后退入口第三段：固定右转，完成后交给黄色视觉搜索。
    } else if (stage_ == "QR_HARDCODE_TURN_2") {
      command_qr_entry_turn(cmd, "QR_TURN_2",
                            -std::abs(turn2_angle_deg_), stage_seconds);
    // 阶段：二维码后 C++ 开环硬编码进黄区，避免 Python 节点抢 /cmd_vel 的时序不确定。
    } else if (stage_ == "QR_HARDCODE_ENTRY") {
      const double backup_end = qr_hardcode_backup_duration_;
      const double turn_end = backup_end + qr_hardcode_turn_duration_;
      const double forward_end = turn_end + qr_hardcode_forward_duration_;
      const double entry_right1_end =
          forward_end + (qr_hardcode_full_entry_
                             ? qr_hardcode_entry_right1_duration_
                             : 0.0);
      const double entry_straight_end =
          entry_right1_end + (qr_hardcode_full_entry_
                                  ? qr_hardcode_entry_straight_duration_
                                  : 0.0);
      const double entry_right2_end =
          entry_straight_end + (qr_hardcode_full_entry_
                                    ? qr_hardcode_entry_right2_duration_
                                    : 0.0);
      const double entry_final_straight_end =
          entry_right2_end + (qr_hardcode_full_entry_
                                  ? qr_hardcode_entry_final_straight_duration_
                                  : 0.0);
      if (stage_seconds < backup_end) {
        cmd.linear.x = qr_hardcode_backup_speed_;
        cmd.angular.z = 0.0;
      } else if (stage_seconds < turn_end) {
        cmd.linear.x = qr_hardcode_turn_speed_;
        cmd.angular.z = qr_hardcode_turn_angular_z_;
      } else if (stage_seconds < forward_end) {
        cmd.linear.x = qr_hardcode_forward_speed_;
        cmd.angular.z = 0.0;
      } else if (stage_seconds < entry_right1_end) {
        cmd.linear.x = qr_hardcode_entry_speed_;
        cmd.angular.z = -std::abs(qr_hardcode_entry_right_angular_z_);
      } else if (stage_seconds < entry_straight_end) {
        cmd.linear.x = qr_hardcode_entry_speed_;
        cmd.angular.z = 0.0;
      } else if (stage_seconds < entry_right2_end) {
        const auto entry_direction =
            (direction_ == "CLOCKWISE")
                ? TurnDirection::CLOCKWISE
                : (direction_ == "COUNTERCLOCKWISE")
                      ? TurnDirection::COUNTERCLOCKWISE
                      : TurnDirection::UNKNOWN;
        cmd.linear.x = qr_hardcode_entry_speed_;
        cmd.angular.z = qr_hardcode_final_entry_turn(
            entry_direction, qr_hardcode_entry_right_angular_z_);
      } else if (stage_seconds < entry_final_straight_end) {
        cmd.linear.x = qr_hardcode_entry_speed_;
        cmd.angular.z = 0.0;
      } else {
        cmd = geometry_msgs::msg::Twist();
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[QR_HARDCODE_CPP] elapsed=%.2f backup_end=%.2f turn_end=%.2f forward_end=%.2f right1_end=%.2f straight_end=%.2f right2_end=%.2f final_straight_end=%.2f direction=%s cmd=(%.2f,%.2f)",
          stage_seconds, backup_end, turn_end, forward_end, entry_right1_end,
          entry_straight_end, entry_right2_end, entry_final_straight_end,
          direction_.c_str(), cmd.linear.x, cmd.angular.z);
    // 阶段：二维码识别后退动作，duration 单位秒，speed 单位 m/s。
    } else if (stage_ == "BACK_FROM_QR") {
      if (stage_seconds < qr_backup_duration_) {
        cmd.linear.x = -std::abs(qr_backup_speed_);
      } else {
        cmd.linear.x = 0.0;
      }
      cmd.angular.z = 0.0;
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[BACK_FROM_QR] elapsed=%.2f duration=%.2f speed=%.2f cmd=(%.2f,%.2f)",
          stage_seconds, qr_backup_duration_, qr_backup_speed_, cmd.linear.x,
          cmd.angular.z);
    // 阶段：从二维码点开环转向黄区门口，角速度单位 rad/s。
    } else if (stage_ == "TURN_TO_GATE") {
      if (stage_seconds < turn_to_gate_duration_) {
        cmd.linear.x = std::abs(ackermann_turn_linear_speed_);
        cmd.angular.z = turn_to_gate_angular_z_;
      } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
      }
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[TURN_TO_GATE] elapsed=%.2f duration=%.2f linear_x=%.2f angular_z=%.2f cmd=(%.2f,%.2f)",
          stage_seconds, turn_to_gate_duration_, ackermann_turn_linear_speed_,
          turn_to_gate_angular_z_, cmd.linear.x, cmd.angular.z);
    // 阶段：向黄区入口前进，包含调试直进、禁入恢复和入口确认。
    } else if (stage_ == "FORWARD_TO_GATE") {
      // 调试模式：进入纯黄区后直接发布 yellow_entry_reached。
      if (debug_stop_after_enter_yellow_) {
        handle_debug_forward_to_yellow(cmd, stage_seconds, safety);
      } else {
        // 入口禁入恢复优先级最高，避免继续压入白色/黄白格。
        if (entry_forbidden_recovering_ && apply_entry_forbidden_recovery(cmd)) {
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "[FORWARD_TO_GATE] mode=forbidden_recovery cmd=(%.2f,%.2f)",
              cmd.linear.x, cmd.angular.z);
        } else if (apply_entry_forbidden_recovery(cmd)) {
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "[FORWARD_TO_GATE] mode=forbidden_recovery cmd=(%.2f,%.2f)",
              cmd.linear.x, cmd.angular.z);
        } else if (command_forced_approach_gate(cmd, stage_seconds)) {
          // 强制前探阶段已填充 cmd。
        } else {
          command_yellow_entry_search(cmd, "FORWARD_TO_GATE");
          // 黄区入口连续确认成功后停车并发布入口到达事件。
          if (should_trigger_once(update_yellow_entry_confirm(), yellow_entry_sent_)) {
            cmd = geometry_msgs::msg::Twist();
            publish_flag(yellow_entry_pub_);
            yellow_entry_sent_ = true;
          }
        }
      }
    // 阶段：二维码后直接去黄区入口，先后退/转向，再按黄线或黄色搜索进入。
    } else if (stage_ == "GO_TO_YELLOW_ENTRY") {
      const double backup_end = qr_backup_duration_;
      const double turn_end = backup_end + turn_duration_;
      const bool yellow_seen_for_gate =
          use_yellow_color_guidance_ && has_recent_yellow() &&
          latest_yellow_area_ >= yellow_min_area_for_guidance_;
      double lane_center_x = 0.0;
      double lane_width = 0.0;
      const bool lane_guidance_ok = yellow_lane_center(lane_center_x, lane_width);
      if (apply_entry_forbidden_recovery(cmd)) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[GO_TO_YELLOW_ENTRY] mode=forbidden_recovery cmd=(%.2f,%.2f)",
            cmd.linear.x, cmd.angular.z);
      } else if (stage_seconds < backup_end) {
        // 子阶段：二维码识别后先后退，避免贴近二维码立牌。
        cmd.linear.x = -std::abs(qr_backup_speed_);
        cmd.angular.z = 0.0;
      } else if (stage_seconds < turn_end && !yellow_seen_for_gate &&
                 !lane_guidance_ok && !yellow_entry_grid_rejected()) {
        // 子阶段：尚未看到黄门口和黄车道时执行开环转向。
        cmd.linear.x = std::clamp(qr_turn_linear_speed_, -0.15, 0.15);
        cmd.angular.z = std::clamp(std::abs(qr_turn_angular_speed_), 0.0, 0.8);
      } else {
        // 子阶段：看到纯黄车道后闭环进入，否则弧形搜索黄区。
        if (pure_yellow_lane_detected() && lane_guidance_ok) {
          cmd.linear.x = std::clamp(forward_to_yellow_speed_ * safety, -0.15, 0.15);
          cmd.angular.z = yellow_entry_lane_angular(lane_center_x);
        } else if (pure_yellow_lane_detected()) {
          cmd.linear.x = std::clamp(forward_to_yellow_speed_ * safety, -0.15, 0.15);
          cmd.angular.z = yellow_entry_color_only_angular();
        } else {
          cmd.linear.x = yellow_search_linear_speed_;
          cmd.angular.z = yellow_search_angular_z_;
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "[FORWARD_TO_GATE] arc_search linear=%.2f angular=%.2f yellow_area=%.3f grid=%d pure=%d",
              cmd.linear.x, cmd.angular.z, latest_yellow_area_ratio_,
              forbidden_grid_detected_now(), pure_yellow_lane_detected());
        }

        std::string forbidden_reason;
        const bool entry_ready =
            !entry_forbidden_detected(forbidden_reason) &&
            stage_seconds >= turn_end + yellow_search_min_seconds_ &&
            latest_yellow_area_ratio_ > yellow_entry_area_threshold_ &&
            lane_guidance_ok;
        if (entry_ready) {
          // 黄区入口 ready 需要连续帧确认。
          ++yellow_entry_detect_ticks_;
        } else {
          yellow_entry_detect_ticks_ = 0;
        }
        if (should_trigger_once(
                yellow_entry_detect_ticks_ >= yellow_entry_confirm_frames_,
                yellow_entry_sent_)) {
          cmd = geometry_msgs::msg::Twist();
          publish_flag(yellow_entry_pub_);
          yellow_entry_sent_ = true;
        }
      }
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[RACE] state=GO_TO_YELLOW_ENTRY elapsed=%.2f backup_end=%.2f turn_end=%.2f yellow_seen=%d lane_guidance=%d yellow_ratio=%.3f grid_reject=%d yellow_ticks=%d/%d cmd=(%.2f,%.2f)",
          stage_seconds, backup_end, turn_end, yellow_seen_for_gate,
          lane_guidance_ok, latest_yellow_area_ratio_, yellow_entry_grid_rejected(),
          yellow_entry_detect_ticks_, yellow_entry_confirm_frames_, cmd.linear.x,
          cmd.angular.z);
    // 阶段：黄区入口搜索，带总超时、禁入恢复、强制前探和入口确认。
    } else if (stage_ == "YELLOW_SEARCH") {
      const bool qr_hardcode_grace =
          qr_hardcode_yellow_grace_active(stage_seconds);
      // 总超时保护：超过配置秒数后停车，不再继续搜索。
      if (yellow_entry_total_timeout_ > 0.0 &&
          stage_seconds > yellow_entry_total_timeout_) {
        cmd = geometry_msgs::msg::Twist();
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[SAFETY] yellow_entry_total_timeout -> STOP elapsed=%.2f cmd=(%.2f,%.2f)",
            stage_seconds, cmd.linear.x, cmd.angular.z);
      } else if (should_trigger_once(qr_hardcode_yellow_entry_force_ready(),
                                     yellow_entry_sent_)) {
        cmd = geometry_msgs::msg::Twist();
        publish_flag(yellow_entry_pub_);
        yellow_entry_sent_ = true;
        RCLCPP_INFO(
            get_logger(),
            "[YELLOW_SEARCH] mode=qr_hardcode_force_entry yellow_area=%.3f threshold=%.3f",
            latest_yellow_area_ratio_, qr_hardcode_yellow_entry_force_ratio_);
      } else if (qr_hardcode_grace) {
        if (command_forced_approach_gate(cmd, stage_seconds)) {
          // 强制前探阶段已填充 cmd。
        } else {
          command_yellow_entry_search(cmd, "YELLOW_SEARCH_GRACE");
          const bool entry_ready =
              qr_hardcode_yellow_entry_force_ready() ||
              update_yellow_entry_confirm();
          if (should_trigger_once(entry_ready, yellow_entry_sent_)) {
            cmd = geometry_msgs::msg::Twist();
            publish_flag(yellow_entry_pub_);
            yellow_entry_sent_ = true;
          }
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[YELLOW_SEARCH] mode=qr_hardcode_grace elapsed=%.2f/%.2f cmd=(%.2f,%.2f)",
            stage_seconds, qr_hardcode_yellow_grace_duration_, cmd.linear.x,
            cmd.angular.z);
      } else if (qr_hardcode_forbidden_recovery_disabled()) {
        if (command_forced_approach_gate(cmd, stage_seconds)) {
          // 强制前探阶段已填充 cmd。
        } else {
          command_yellow_entry_search(cmd, "YELLOW_SEARCH_QR_HARDCODE");
          const bool entry_ready =
              qr_hardcode_yellow_entry_force_ready() ||
              update_yellow_entry_confirm();
          if (should_trigger_once(entry_ready, yellow_entry_sent_)) {
            cmd = geometry_msgs::msg::Twist();
            publish_flag(yellow_entry_pub_);
            yellow_entry_sent_ = true;
          }
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[YELLOW_SEARCH] mode=qr_hardcode_no_forbidden elapsed=%.2f cmd=(%.2f,%.2f)",
            stage_seconds, cmd.linear.x, cmd.angular.z);
      } else if (entry_forbidden_recovering_ && apply_entry_forbidden_recovery(cmd)) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[YELLOW_SEARCH] mode=forbidden_recovery cmd=(%.2f,%.2f)",
            cmd.linear.x, cmd.angular.z);
      } else if (apply_entry_forbidden_recovery(cmd)) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[YELLOW_SEARCH] mode=forbidden_recovery cmd=(%.2f,%.2f)",
            cmd.linear.x, cmd.angular.z);
      } else if (command_forced_approach_gate(cmd, stage_seconds)) {
        // 强制前探阶段已填充 cmd。
      } else {
        command_yellow_entry_search(cmd, "FORWARD_TO_GATE");
        // 黄区入口确认成功后发布 yellow_entry_reached。
        if (should_trigger_once(update_yellow_entry_confirm(), yellow_entry_sent_)) {
          cmd = geometry_msgs::msg::Twist();
          publish_flag(yellow_entry_pub_);
          yellow_entry_sent_ = true;
        }
      }
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[RACE] state=YELLOW_SEARCH elapsed=%.2f yellow_area=%.3f checker=%.3f grid_reject=%d yellow_ticks=%d/%d cmd=(%.2f,%.2f)",
          stage_seconds, latest_yellow_area_ratio_, latest_grid_max_ratio_,
          yellow_entry_grid_rejected(), yellow_entry_detect_ticks_,
          yellow_entry_confirm_frames_, cmd.linear.x, cmd.angular.z);
    // 阶段：黄区绕行，按黄线车道闭环，同时处理绿色禁区、黄白格、障碍物和出口门。
    } else if (stage_ == "YELLOW_LAP") {
      // 每次进入黄区环道时重置车道滤波器和转角硬编码状态。
      if (previous_stage_ != "YELLOW_LAP") {
        reset_yellow_lane_filter();
        yellow_corner_hc_active_ = false;
        yellow_corner_hc_cooldown_end_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      const auto direction =
          (direction_ == "COUNTERCLOCKWISE")
              ? TurnDirection::COUNTERCLOCKWISE
              : TurnDirection::CLOCKWISE;
      const double entry_right1_end = yellow_loop_entry_right1_duration_;
      const double entry_straight_end =
          entry_right1_end + yellow_loop_entry_straight_duration_;
      const double entry_right2_end =
          entry_straight_end + yellow_loop_entry_right2_duration_;
      const bool run_entry_hardcode =
          !qr_hardcode_full_entry_ || !qr_hardcode_yellow_path_active_;
      if (run_entry_hardcode && stage_seconds < entry_right2_end) {
        cmd.linear.x = yellow_loop_entry_speed_ * safety;
        if (stage_seconds < entry_right1_end) {
          cmd.angular.z = yellow_loop_entry_right_angular_z_;
        } else if (stage_seconds < entry_straight_end) {
          cmd.angular.z = 0.0;
        } else {
          cmd.angular.z = yellow_loop_entry_right_angular_z_;
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[YELLOW_LAP] mode=entry_hardcode elapsed=%.2f right1_end=%.2f straight_end=%.2f right2_end=%.2f cmd=(%.2f,%.2f)",
            stage_seconds, entry_right1_end, entry_straight_end,
            entry_right2_end, cmd.linear.x, cmd.angular.z);
      } else {
        const bool qr_lap_grace =
            qr_hardcode_yellow_lap_grace_active(stage_seconds);
        double lane_center_x = 0.0;
        double lane_width = 0.0;
        const bool lane_ok =
            yellow_lane_center_for_direction(direction, lane_center_x, lane_width);
        const bool green_recovery_danger =
            yellow_lap_green_recovery_danger();
        const bool grid_danger_raw =
            forbidden_grid_detected_now() ||
            (forbidden_grid_detected_ &&
             latest_grid_max_ratio_ >= grid_forbidden_threshold_);
        const bool grid_danger =
            checker_forbidden_enabled_ && !qr_lap_grace && grid_danger_raw;
        const bool green_recovery_allowed =
            green_recovery_danger &&
            !yellow_lap_green_recovery_cooling_down();
        const bool green_steer_danger = yellow_lap_green_steer_danger();
        const bool yellow_obstacle_override =
            yellow_obstacle_allow_green_ &&
            (yellow_obstacle_active() || yellow_obstacle_active_or_returning());
        const bool allow_yellow_lap_green_recovery = false;

        // ===== 黄圈转角硬编码：一边绿+对侧黄 → 往黄色方向开环转弯 =====
        std::string avoid_direction = "none";
        const bool corner_hc_armed =
            yellow_corner_hardcode_enabled_ && !qr_lap_grace &&
            !yellow_corner_hc_active_ &&
            (now() - yellow_corner_hc_cooldown_end_).seconds() > 0.0;
        if (corner_hc_armed && lane_ok) {
          const bool green_left =
              latest_green_left_ratio_ >= yellow_corner_green_threshold_;
          const bool yellow_right =
              latest_yellow_right_ratio_ >= yellow_corner_yellow_threshold_;
          const bool green_right =
              latest_green_right_ratio_ >= yellow_corner_green_threshold_;
          const bool yellow_left =
              latest_yellow_left_ratio_ >= yellow_corner_yellow_threshold_;
          if (green_left && yellow_right) {
            yellow_corner_hc_active_ = true;
            yellow_corner_hc_sign_ = -1.0;  // 右转（黄在右）
            yellow_corner_hc_start_time_ = now();
            RCLCPP_INFO(get_logger(),
                        "[YELLOW_CORNER_HC] start turn=RIGHT green_left=%.3f yellow_right=%.3f",
                        latest_green_left_ratio_, latest_yellow_right_ratio_);
          } else if (green_right && yellow_left) {
            yellow_corner_hc_active_ = true;
            yellow_corner_hc_sign_ = 1.0;   // 左转（黄在左）
            yellow_corner_hc_start_time_ = now();
            RCLCPP_INFO(get_logger(),
                        "[YELLOW_CORNER_HC] start turn=LEFT green_right=%.3f yellow_left=%.3f",
                        latest_green_right_ratio_, latest_yellow_left_ratio_);
          }
        }

        if (yellow_corner_hc_active_) {
          const double hc_elapsed =
              (now() - yellow_corner_hc_start_time_).seconds();
          if (hc_elapsed < yellow_corner_hc_turn_duration_) {
            cmd.linear.x = yellow_corner_hc_turn_speed_ * safety;
            cmd.angular.z = clamp_yellow_direction_angular(
                yellow_corner_hc_sign_ * yellow_corner_hc_turn_angular_,
                direction);
            avoid_direction = "corner_hardcode";
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 500,
                "[YELLOW_CORNER_HC] turning elapsed=%.2f/%.2f sign=%.1f cmd=(%.2f,%.2f)",
                hc_elapsed, yellow_corner_hc_turn_duration_,
                yellow_corner_hc_sign_, cmd.linear.x, cmd.angular.z);
          } else {
            yellow_corner_hc_active_ = false;
            yellow_corner_hc_cooldown_end_ = now();
            RCLCPP_INFO(get_logger(),
                        "[YELLOW_CORNER_HC] done, cooldown %.1fs",
                        yellow_corner_cooldown_seconds_);
          }
        }

        if (!yellow_corner_hc_active_) {
        // ===== 常规车道跟随（无硬编码转角时）=====
        // 黄白格危险优先直线后退恢复，禁止继续转向。
        if (grid_danger) {
          if (apply_grid_recovery(cmd)) {
            avoid_direction = "grid_straight_backup";
          } else {
            cmd = geometry_msgs::msg::Twist();
            avoid_direction = "grid_hold";
          }
        } else if (allow_yellow_lap_green_recovery &&
                   green_recovery_allowed && !yellow_obstacle_override) {
          // YELLOW_LAP 内禁用绿色触发的通用后退/转向恢复；继续交给黄线闭环和 green guard 处理。
          avoid_direction = "green_recovery_disabled";
        } else if (lane_ok) {
          // 黄车道有效：按滤波后的车道中心误差闭环，偏差大时自动降速。
          yellow_lane_lost_active_ = false;
          update_yellow_lane_filter(lane_center_x);
          const double filtered_center = yellow_lane_filter_x_;
          const double lane_error = filtered_center - image_center_x_;
          const double bias =
              direction == TurnDirection::COUNTERCLOCKWISE
                  ? std::clamp(ccw_bias_, 0.0, ccw_bias_max_)
                  : -std::clamp(ccw_bias_, 0.0, ccw_bias_max_);
          const double raw_lane_angular =
              std::clamp(-yellow_lane_kp_ * lane_error + bias,
                         -yellow_lane_max_angular_z_,
                         yellow_lane_max_angular_z_);
          // Normal lane centering must allow both steering directions. The
          // QR direction lock is only for search/recovery/corner motions.
          const double lane_angular = raw_lane_angular;
          bool corner_boost_override = false;
          if (yellow_corner_boost_enabled_ && !yellow_obstacle_override &&
              !green_steer_danger) {
            const double image_width = image_center_x_ * 2.0;
            const bool single_side_boundary =
                latest_yellow_left_x_ <= 2.0 ||
                latest_yellow_right_x_ >= image_width - 2.0;
            const bool corner_cooling_down =
                yellow_corner_last_end_time_.nanoseconds() != 0 &&
                (now() - yellow_corner_last_end_time_).seconds() <
                    yellow_corner_cooldown_;
            if (!yellow_corner_boost_active_ && !corner_cooling_down &&
                single_side_boundary &&
                std::abs(lane_error) >= yellow_corner_trigger_error_) {
              yellow_corner_boost_active_ = true;
              yellow_corner_boost_phase_ = 0;
              yellow_corner_boost_start_time_ = now();
              yellow_corner_boost_sign_ =
                  std::abs(lane_angular) > 0.05
                      ? (lane_angular > 0.0 ? 1.0 : -1.0)
                      : (lane_error < 0.0 ? 1.0 : -1.0);
              yellow_corner_boost_sign_ =
                  yellow_direction_search_sign(direction) == 0.0
                      ? yellow_corner_boost_sign_
                      : yellow_direction_search_sign(direction);
            }

            if (yellow_corner_boost_active_) {
              const double phase_elapsed =
                  (now() - yellow_corner_boost_start_time_).seconds();
              if (yellow_corner_boost_phase_ == 0 &&
                  phase_elapsed < yellow_corner_turn_duration_) {
                cmd.linear.x = yellow_corner_turn_speed_ * safety;
                cmd.angular.z = clamp_yellow_direction_angular(
                    yellow_corner_boost_sign_ *
                        std::abs(yellow_corner_turn_angular_),
                    direction);
                corner_boost_override = true;
                avoid_direction = "corner_boost_turn";
              } else {
                if (yellow_corner_boost_phase_ == 0) {
                  yellow_corner_boost_phase_ = 1;
                  yellow_corner_boost_start_time_ = now();
                }
                const double exit_elapsed =
                    (now() - yellow_corner_boost_start_time_).seconds();
                if (exit_elapsed < yellow_corner_exit_straight_duration_) {
                  cmd.linear.x = yellow_corner_turn_speed_ * safety;
                  cmd.angular.z = 0.0;
                  corner_boost_override = true;
                  avoid_direction = "corner_boost_straight";
                } else {
                  yellow_corner_boost_active_ = false;
                  yellow_corner_last_end_time_ = now();
                }
              }
            }
          }

          if (!corner_boost_override) {
            cmd.angular.z = lane_angular;
            cmd.linear.x = (std::abs(lane_error) > 120.0)
                               ? yellow_lane_safe_speed_ * safety
                               : yellow_lane_speed_ * safety;
          }

          const double yellow_guard_sign = yellow_direction_search_sign(direction);
          const double yellow_guard_side_green =
              direction == TurnDirection::COUNTERCLOCKWISE
                  ? latest_green_right_ratio_
                  : (direction == TurnDirection::CLOCKWISE
                         ? latest_green_left_ratio_
                         : std::max(latest_green_left_ratio_,
                                    latest_green_right_ratio_));
          const bool yellow_green_guard_warning =
              yellow_guard_sign != 0.0 &&
              (yellow_guard_side_green >= yellow_lap_green_guard_side_ratio_ ||
               latest_green_center_ratio_ >= yellow_lap_green_guard_center_ratio_);
          const bool yellow_green_guard_strong =
              yellow_guard_sign != 0.0 &&
              (yellow_guard_side_green >=
                   yellow_lap_green_guard_strong_side_ratio_ ||
               latest_green_center_ratio_ >=
                   yellow_lap_green_guard_strong_center_ratio_);
          if (yellow_green_guard_warning && !yellow_obstacle_override) {
            yellow_corner_boost_active_ = false;
            cmd.linear.x = std::min(cmd.linear.x, yellow_lane_safe_speed_ * safety);
            if (yellow_green_guard_strong) {
              cmd.angular.z = yellow_direction_search_angular(
                  direction, yellow_lap_green_guard_force_angular_);
              avoid_direction = "green_guard_strong";
            } else {
              if (yellow_guard_sign * cmd.angular.z < 0.0) {
                cmd.angular.z = 0.0;
              }
              cmd.angular.z = std::clamp(
                  cmd.angular.z + yellow_guard_sign *
                                      std::abs(yellow_lap_green_guard_bias_angular_),
                  -yellow_lane_max_angular_z_, yellow_lane_max_angular_z_);
              avoid_direction = "green_guard";
            }
          }

          if (green_steer_danger && !yellow_obstacle_override) {
            yellow_corner_boost_active_ = false;
            cmd.linear.x = yellow_lane_safe_speed_ * safety;
            cmd.angular.z = clamp_yellow_direction_angular(
                std::clamp(yellow_lap_green_escape_angular(),
                           -yellow_lane_max_angular_z_,
                           yellow_lane_max_angular_z_),
                direction);
            avoid_direction = "green_escape";
          }

          if (yellow_obstacle_active() || yellow_obstacle_active_or_returning()) {
            if (yellow_obstacle_allow_green_) {
              apply_yellow_obstacle_bypass(cmd, lane_angular, lane_error, safety,
                                           avoid_direction);
            } else {
              // 黄区内桶锥只顺滑绕行；若绕行方向被禁区阻挡，保留低速黄线跟随。
              avoid_direction =
                  obstacle_center_x_ < image_center_x_ ? "right" : "left";
              const double avoid = obstacle_avoidance();
              if (yellow_lane_turn_blocked(avoid)) {
                cmd.linear.x =
                    std::min(cmd.linear.x, yellow_lane_safe_speed_ * safety);
                cmd.angular.z = std::clamp(cmd.angular.z,
                                           -yellow_lane_max_angular_z_ * 0.5,
                                           yellow_lane_max_angular_z_ * 0.5);
              } else {
                cmd.linear.x =
                    std::min(cmd.linear.x, obstacle_avoid_speed_ * safety);
                cmd.angular.z =
                    std::clamp(cmd.angular.z + avoid,
                               -yellow_lane_max_angular_z_,
                               yellow_lane_max_angular_z_);
              }
            }
          }
          last_yellow_lane_angular_ = cmd.angular.z;
        } else {
          if (yellow_obstacle_active() || yellow_obstacle_active_or_returning()) {
            if (yellow_obstacle_active() &&
                yellow_obstacle_phase_ == ObstacleBypassPhase::IDLE) {
              start_yellow_obstacle_avoidance();
            }
            if (yellow_obstacle_phase_ == ObstacleBypassPhase::AVOIDING) {
              cmd.linear.x = yellow_obstacle_avoid_speed_ * safety;
              cmd.angular.z =
                  std::clamp(yellow_obstacle_sign_ * obstacle_avoid_angular_,
                             -0.60, 0.60);
              avoid_direction = "yellow_obstacle_lost_lane";
            } else {
              yellow_obstacle_phase_ = ObstacleBypassPhase::RETURNING;
              cmd.linear.x = yellow_lane_safe_speed_ * safety;
              cmd.angular.z =
                  std::clamp(-yellow_obstacle_sign_ *
                                 yellow_obstacle_return_angular_,
                             -0.60, 0.60);
              avoid_direction = "yellow_obstacle_return_lost_lane";
            }
          } else {
          // 黄线丢失：短时间沿上一次角速度低速前进，超时后停车。
          if (!yellow_lane_lost_active_) {
            yellow_lane_lost_active_ = true;
            yellow_lane_lost_start_time_ = now();
          }
          const double lost_time = (now() - yellow_lane_lost_start_time_).seconds();
          if (lost_time <= yellow_lane_lost_timeout_) {
            cmd.linear.x = yellow_lane_safe_speed_ * safety;
            cmd.angular.z = clamp_yellow_direction_angular(
                std::clamp(last_yellow_lane_angular_,
                           -yellow_lane_max_angular_z_,
                           yellow_lane_max_angular_z_),
                direction);
          } else {
            cmd.linear.x = yellow_lane_safe_speed_ * safety;
            const double search_angular =
                std::abs(last_yellow_lane_angular_) > 0.05
                    ? last_yellow_lane_angular_
                    : yellow_direction_search_angular(
                          direction, yellow_lane_escape_angular_);
            cmd.angular.z = clamp_yellow_direction_angular(
                std::clamp(search_angular,
                           -yellow_lane_max_angular_z_,
                           yellow_lane_max_angular_z_),
                direction);
          }
          }
        }

        // 黄区内识别到行人并达到面积阈值后，上报一次行人结果。
        if (should_trigger_once(human_seen_ && human_area_ >= human_trigger_area_,
                                human_result_sent_)) {
          std_msgs::msg::String msg;
          msg.data = human_label_;
          human_result_pub_->publish(msg);
          human_result_sent_ = true;
        }

        }  // end if (!yellow_corner_hc_active_)

        update_yellow_exit_arm(direction, lane_ok);
        const bool side_exit_candidate =
            yellow_exit_candidate_present(direction);
        if (yellow_exit_armed_) {
          if (side_exit_candidate) {
            yellow_exit_candidate_seen_ = true;
          } else if (yellow_exit_candidate_seen_) {
            // Require one outlet-like view to disappear before the same
            // direction-side structure can be accepted on the next pass.
            yellow_exit_candidate_cleared_ = true;
          }
        }

        const bool visual_exit_ready =
            yellow_lap_visual_exit_ready(stage_seconds, direction);
        const bool side_yellow_exit_ready =
            yellow_lap_side_yellow_exit_ready(stage_seconds, direction);
        const bool yellow_exit_ready =
            visual_exit_ready || side_yellow_exit_ready;
        yellow_lap_line_return_ticks_ = 0;

        // 正确方向的出口候选一出现就接管转向，避免连续确认期间被
        // 绿区保护或普通黄线居中命令拉向相反方向。
        const bool exit_approach_active =
            yellow_exit_armed_ &&
            (yellow_exit_green_gap_ticks_ > 0 ||
             yellow_exit_side_yellow_ticks_ > 0);
        if (exit_approach_active) {
          yellow_corner_boost_active_ = false;
          cmd.linear.x = exit_gate_align_speed_ * safety;
          cmd.angular.z =
              yellow_exit_turn_sign_ * std::abs(exit_gate_turn_angular_z_);
          avoid_direction = "exit_approach";
        }

        // 黄通道必须看到真实出口：优先使用绿/黄开口结构，方向侧大黄作为
        // armed 后的备用证据；普通巡线不能单独触发。
        if (should_trigger_once(yellow_exit_ready, line_reacquired_sent_)) {
          cmd = geometry_msgs::msg::Twist();
          RCLCPP_INFO(
              get_logger(),
              "[YELLOW_EXIT] trigger=%s elapsed=%.2f direction=%s green_ticks=%d/%d side_ticks=%d/%d armed=%d cleared=%d yellow_left=%.3f yellow_right=%.3f green_left=%.3f green_right=%.3f turn_sign=%.1f",
              visual_exit_ready ? "visual_opening" : "side_yellow",
              stage_seconds, direction_.c_str(),
              yellow_exit_green_gap_ticks_,
              yellow_exit_green_gap_ticks_required_,
              yellow_exit_side_yellow_ticks_,
              yellow_exit_green_gap_ticks_required_,
              yellow_exit_armed_, yellow_exit_candidate_cleared_,
              latest_yellow_left_ratio_, latest_yellow_right_ratio_,
              latest_green_left_ratio_, latest_green_right_ratio_,
              yellow_exit_turn_sign_);
          publish_flag(loop_complete_pub_);
          line_reacquired_sent_ = true;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[YELLOW_LANE] left=%.1f right=%.1f center=%.1f width=%.1f green_left=%.3f green_right=%.3f green_center=%.3f grid_left=%.3f grid_right=%.3f grid_center=%.3f grid_detected=%d qr_lap_grace=%d corner_hc=%d cooldown=%.1f lane_error=%.1f lost_time=%.2f obstacle=%d avoid=%s min_lap=%.1f elapsed=%.2f dir=%s exit_arm=%d exit_seen=%d exit_clear=%d exit_green_ticks=%d exit_side_ticks=%d cmd=(%.2f, %.2f)",
          latest_yellow_left_x_,
          latest_yellow_right_x_,
          lane_ok ? lane_center_x : latest_lane_center_x_,
          lane_ok ? lane_width : latest_lane_width_,
          latest_green_left_ratio_,
          latest_green_right_ratio_,
          latest_green_center_ratio_,
          latest_grid_left_ratio_,
          latest_grid_right_ratio_,
          latest_grid_center_ratio_,
          grid_danger,
          qr_lap_grace,
          yellow_corner_hc_active_,
          yellow_corner_hc_active_ ? 0.0 :
              std::max(0.0, (yellow_corner_cooldown_seconds_ -
                             (now() - yellow_corner_hc_cooldown_end_).seconds())),
          lane_ok ? (lane_center_x - image_center_x_) : 0.0,
          yellow_lane_lost_active_ ? (now() - yellow_lane_lost_start_time_).seconds() : 0.0,
          yellow_obstacle_active(),
          avoid_direction.c_str(),
          min_lap_time_,
          stage_seconds,
          direction_.c_str(),
          yellow_exit_armed_,
          yellow_exit_candidate_seen_,
          yellow_exit_candidate_cleared_,
          yellow_exit_green_gap_ticks_,
          yellow_exit_side_yellow_ticks_,
          cmd.linear.x,
          cmd.angular.z);
      }
    // 阶段：出口转弯。朝视觉确认的大黄开口转出，不能因为误见普通线直接跳过。
    } else if (stage_ == "EXIT_GATE") {
      cmd.linear.x = exit_gate_align_speed_ * safety;
      cmd.angular.z =
          yellow_exit_turn_sign_ * std::abs(exit_gate_turn_angular_z_);

      const bool yellow_lane_gone =
          stage_seconds >= min_forward_exit_gate_time_ &&
          (!has_recent_yellow_lane_info() || !latest_lane_yellow_visible_);
      const bool timed_out =
          stage_seconds >= exit_gate_visual_timeout_seconds_;

      if (should_trigger_once(yellow_lane_gone || timed_out,
                              exit_gate_ready_sent_)) {
        cmd = geometry_msgs::msg::Twist();
        publish_flag(exit_gate_ready_pub_);
        exit_gate_ready_sent_ = true;
      }
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[EXIT_GATE] visual_turn elapsed=%.2f track=%d lane_gone=%d timeout=%d yellow_left=%.3f yellow_right=%.3f green_left=%.3f green_right=%.3f lane_yellow_vis=%d turn_sign=%.1f cmd=(%.2f,%.2f)",
          stage_seconds, has_recent_track(), yellow_lane_gone, timed_out,
          latest_yellow_left_ratio_, latest_yellow_right_ratio_,
          latest_green_left_ratio_, latest_green_right_ratio_,
          latest_lane_yellow_visible_, yellow_exit_turn_sign_,
          cmd.linear.x, cmd.angular.z);
    // 阶段：出黄区后前进找回普通巡线，找回后发布 line_reacquired。
    } else if (stage_ == "FORWARD_EXIT_GATE") {
      cmd.linear.x = forward_exit_gate_speed_ * safety;
      cmd.angular.z =
          std::clamp(yellow_exit_turn_sign_ *
                         std::abs(exit_gate_turn_angular_z_) * 0.35 +
                         obstacle_avoidance() * 0.15,
                     -yellow_max_angular_z_, yellow_max_angular_z_);
      if (should_trigger_once(stage_seconds >= min_forward_exit_gate_time_ && has_recent_track(),
                              line_reacquired_sent_)) {
        cmd = geometry_msgs::msg::Twist();
        publish_flag(line_reacquired_pub_);
        line_reacquired_sent_ = true;
      }
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[FORWARD_EXIT_GATE] visual_turn yellow_left=%.3f yellow_right=%.3f green_left=%.3f green_right=%.3f turn_sign=%.1f track=%d cmd=(%.2f,%.2f)",
          latest_yellow_left_ratio_, latest_yellow_right_ratio_,
          latest_green_left_ratio_, latest_green_right_ratio_,
          yellow_exit_turn_sign_, has_recent_track(), cmd.linear.x, cmd.angular.z);
    // 阶段：沿线返回停车点，处理障碍物、丢线完成判定和 P 点停车。
    } else if (stage_ == "LINE_BACK_HOME") {
      cmd.linear.x = back_line_speed_ * safety;
      cmd.angular.z = line_angular() + obstacle_avoidance();
      if (obstacle_active() ||
          line_obstacle_phase_ != ObstacleBypassPhase::IDLE) {
        // 返程遇障碍：降低线速度并混合巡线/避障角速度。
        cmd.linear.x = std::min(cmd.linear.x, obstacle_avoid_speed_ * safety);
        if (line_obstacle_phase_ == ObstacleBypassPhase::RETURNING) {
          cmd.linear.x = std::min(cmd.linear.x, back_line_speed_ * safety);
        }
        cmd.angular.z = line_obstacle_bypass_angular(line_angular());
      } else if (!has_recent_track()) {
        // 返程停车只允许由 P 点检测触发；丢线时继续低速找线，不再判定返程完成。
        cmd.linear.x = line_lost_speed_ * safety;
        cmd.angular.z = line_lost_search_angular_;
      } else {
        line_lost_finish_active_ = false;
      }

      const bool parking_centered =
          std::abs(parking_center_x_ - image_center_x_) <= 140.0 &&
          parking_center_y_ >= 180.0;
      const bool parking_candidate =
          has_recent_track() && parking_seen_ &&
          parking_area_ >= parking_trigger_area_ && parking_centered;
      if (parking_candidate) {
        // 返程停车必须同时沿黑线且连续看到 P 点，避免错误阶段或远处误报停车。
        ++parking_lock_ticks_;
      } else {
        parking_lock_ticks_ = 0;
      }

      if (!parking_approach_active_ &&
          (now() - stage_enter_time_).seconds() >= min_back_time_ &&
          parking_lock_ticks_ >= parking_required_ticks_) {
        parking_approach_active_ = true;
        parking_approach_start_time_ = now();
        RCLCPP_INFO(
            get_logger(),
            "[PARK_APPROACH] start area=%.0f center=(%.1f,%.1f) ticks=%d/%d",
            parking_area_, parking_center_x_, parking_center_y_,
            parking_lock_ticks_, parking_required_ticks_);
      }

      if (parking_approach_active_) {
        cmd.linear.x = parking_approach_speed_ * safety;
        cmd.angular.z = line_angular();
        const double approach_elapsed =
            (now() - parking_approach_start_time_).seconds();
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[PARK_APPROACH] elapsed=%.2f/%.2f cmd=(%.2f,%.2f)",
            approach_elapsed, parking_approach_after_seen_seconds_,
            cmd.linear.x, cmd.angular.z);
      }

      if (should_trigger_once(
              parking_approach_active_ &&
              (now() - parking_approach_start_time_).seconds() >=
                  parking_approach_after_seen_seconds_,
              park_zone_sent_)) {
        cmd = geometry_msgs::msg::Twist();
        publish_flag(park_zone_pub_);
        park_zone_sent_ = true;
      }
    // 阶段：停止/等待启动，始终输出零速度。
    } else if (stage_ == "STOP" || stage_ == "WAIT_START") {
      cmd = geometry_msgs::msg::Twist();
    }

    // 调试硬锁阶段不运行通用恢复，避免恢复逻辑覆盖指定开环动作。
    const bool hard_locked_debug_stage =
        stage_ == "BACK_FROM_QR" ||
        stage_ == "QR_HARDCODE_TURN_1" ||
        stage_ == "QR_ENTRY_ADVANCE" ||
        stage_ == "QR_HARDCODE_TURN_2" ||
        stage_ == "QR_HARDCODE_ENTRY" ||
        stage_ == "TURN_TO_GATE" ||
        stage_ == "EXIT_GATE" ||
        (debug_stop_after_enter_yellow_ && stage_ == "FORWARD_TO_GATE");
    if (!hard_locked_debug_stage) {
      maybe_start_recovery();
      apply_recovery(cmd);
    }

    // 最终安全保护：必要时限制 QR 后黄区入口搜索中的静态右转。
    block_static_right_steer_after_qr(cmd);

    if (hardcode_override_active_) {
      last_cmd_linear_x_ = 0.0;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[QR_HARDCODE] override active, skip motion_controller /cmd_vel");
      return;
    }

    last_cmd_linear_x_ = cmd.linear.x;

    // 发布最终速度指令，linear.x 单位 m/s，angular.z 单位 rad/s。
    cmd_pub_->publish(cmd);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[CMD] linear=%.2f angular=%.2f",
        cmd.linear.x, cmd.angular.z);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "control tick: stage=%s has_track=%d qr_seen=%d parking_seen=%d obstacle_active=%d obstacle_area=%.0f line_obstacle=%s yellow_obstacle=%s yellow_seen=%d yellow_lock=%d front=%.2f depth=%.2f cmd=(%.2f, %.2f)",
        stage_.c_str(),
        has_track_,
        qr_seen_,
        parking_seen_,
        obstacle_active(),
        obstacle_area_,
        obstacle_bypass_phase_name(line_obstacle_phase_),
        obstacle_bypass_phase_name(yellow_obstacle_phase_),
        yellow_entry_seen_,
        yellow_entry_lock_ticks_,
        front_range_,
        latest_depth_range_,
        cmd.linear.x,
        cmd.angular.z);
  }

  // 当前任务阶段，由 /race/stage 更新。
  std::string stage_{"WAIT_START"};
  std::string previous_stage_{""};
  std::string direction_{"UNKNOWN"};
  // QR 解码是否已经拿到非空文本。
  bool qr_text_locked_{false};
  std::string qr_text_;
  double distance_traveled_{0.0};
  double last_cmd_linear_x_{0.0};
  rclcpp::Time last_distance_update_time_{0, 0, RCL_ROS_TIME};
  double current_yaw_{0.0};
  std::string yaw_source_{"none"};
  rclcpp::Time last_yaw_time_{0, 0, RCL_ROS_TIME};
  bool qr_turn_started_{false};
  bool qr_turn_done_sent_{false};
  bool qr_entry_advance_started_{false};
  double qr_turn_start_yaw_{0.0};
  double qr_turn_target_yaw_{0.0};
  double qr_entry_advance_start_distance_{0.0};
  // 普通巡线状态：latest_track_x_ 为图像横向像素坐标。
  bool has_track_{false};
  double latest_track_x_{320.0};
  rclcpp::Time last_track_time_{0, 0, RCL_ROS_TIME};
  // 黄色边界质心状态：x 为像素坐标，area 为像素面积。
  bool has_yellow_{false};
  double latest_yellow_x_{320.0};
  /// /yellow_boundary_center point.y，像素面积，用于信度门限
  double latest_yellow_area_{0.0};
  rclcpp::Time last_yellow_time_{0, 0, RCL_ROS_TIME};
  bool has_green_{false};
  double latest_green_x_{320.0};
  double latest_green_area_{0.0};
  rclcpp::Time last_green_time_{0, 0, RCL_ROS_TIME};
  // 黄区车道数组状态：比例字段范围 0~1，x/width 字段单位像素。
  bool has_yellow_lane_info_{false};
  double latest_lane_center_x_{320.0};
  double latest_yellow_left_x_{-1.0};
  double latest_yellow_right_x_{-1.0};
  double latest_lane_width_{0.0};
  double latest_yellow_area_ratio_{0.0};
  double latest_green_left_ratio_{0.0};
  double latest_green_right_ratio_{0.0};
  double latest_green_center_ratio_{0.0};
  double latest_green_area_ratio_{0.0};
  bool green_filter_initialized_{false};
  bool green_danger_confirmed_{false};
  int green_danger_ticks_{0};
  int green_clear_ticks_{0};
  double latest_grid_area_ratio_{0.0};
  double latest_white_near_ratio_{0.0};
  double latest_yellow_near_ratio_{0.0};
  double latest_yellow_left_ratio_{0.0};
  double latest_yellow_right_ratio_{0.0};
  double latest_grid_left_ratio_{0.0};
  double latest_grid_right_ratio_{0.0};
  double latest_grid_center_ratio_{0.0};
  double latest_grid_max_ratio_{0.0};
  bool forbidden_grid_detected_{false};
  bool latest_lane_yellow_visible_{false};
  bool latest_lane_green_visible_{false};
  double lane_width_estimate_{220.0};
  double last_yellow_lane_angular_{0.0};
  bool yellow_lane_lost_active_{false};
  rclcpp::Time last_yellow_lane_info_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time yellow_lane_lost_start_time_{0, 0, RCL_ROS_TIME};
  // 雷达/深度距离状态：单位米，默认 10 m 表示远距离安全。
  double front_range_{10.0};
  double left_range_{10.0};
  double right_range_{10.0};
  double latest_depth_range_{10.0};
  rclcpp::Time last_depth_time_{0, 0, RCL_ROS_TIME};
  // 目标检测状态：面积单位像素，中心 x 单位像素。
  bool qr_seen_{false};
  double qr_center_x_{320.0};
  double qr_area_{0.0};
  bool human_seen_{false};
  std::string human_label_;
  double human_area_{0.0};
  bool parking_seen_{false};
  double parking_center_x_{320.0};
  double parking_center_y_{240.0};
  double parking_area_{0.0};
  bool obstacle_seen_{false};
  double obstacle_center_x_{320.0};
  double obstacle_width_{0.0};
  double obstacle_height_{0.0};
  double obstacle_bottom_y_{0.0};
  double obstacle_area_{0.0};
  rclcpp::Time last_obstacle_time_{0, 0, RCL_ROS_TIME};
  bool yellow_entry_seen_{false};
  // 一次性事件发布锁存，防止同一阶段重复发布 true。
  bool qr_ready_sent_{false};
  bool qr_capture_active_{false};
  int qr_visual_lock_count_{0};
  double qr_approach_brake_start_area_{10000.0};
  double qr_approach_creep_speed_{0.16};
  double qr_align_start_area_{5000.0};
  double qr_line_blend_{0.35};
  bool yellow_entry_turn_left_{true};
  double yellow_entry_turn_line_weight_{0.70};
  bool yellow_entry_sent_{false};
  bool gate_aligned_sent_{false};
  int yellow_entry_lock_ticks_{0};
  int direct_yellow_entry_lock_count_{0};
  bool loop_complete_sent_{false};
  int yellow_lap_line_return_ticks_{0};
  int yellow_exit_green_gap_ticks_{0};
  int yellow_exit_side_yellow_ticks_{0};
  int parking_lock_ticks_{0};
  bool parking_approach_active_{false};
  bool park_zone_sent_{false};
  bool parked_sent_{false};
  bool human_result_sent_{false};
  bool exit_gate_ready_sent_{false};
  bool line_reacquired_sent_{false};
  bool return_complete_sent_{false};
  int exit_wide_direction_{0};
  bool line_lost_finish_active_{false};
  bool hardcode_override_active_{false};
  bool recovery_active_{false};
  RecoveryPhase recovery_phase_{RecoveryPhase::IDLE};
  std::string recovery_reason_;
  int obstacle_recovery_count_{0};
  int obstacle_recovery_candidate_ticks_{0};
  int recovery_yellow_lock_ticks_{0};
  rclcpp::Time stage_enter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time qr_capture_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time line_lost_finish_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time parking_approach_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_green_recovery_time_{0, 0, RCL_ROS_TIME};

  // 线速度参数，单位 m/s；时间参数以 seconds 结尾，单位秒。
  double follow_speed_{0.55};
  double approach_speed_{0.20};
  double qr_capture_hold_seconds_{0.35};
  double qr_stop_distance_{0.60};
  double qr_decode_wait_seconds_{1.20};
  int qr_visual_lock_ticks_required_{3};
  double yellow_entry_speed_{0.20};
  double yellow_entry_pause_seconds_{0.30};
  double yellow_entry_turn_seconds_{2.40};
  double yellow_entry_turn_angular_{0.95};
  double qr_backup_duration_{1.2};
  double qr_backup_speed_{0.10};
  double ackermann_turn_linear_speed_{0.10};
  double turn_angular_z_{0.6};
  double turn_duration_{1.5};
  double qr_turn_linear_speed_{0.06};
  double qr_turn_angular_speed_{0.60};
  double qr_forward_speed_{0.08};
  double qr_forward_small_left_angular_{0.10};
  double turn1_angle_deg_{170.42};
  double entry_advance_distance_{0.834};
  double entry_advance_speed_{0.12};
  double turn2_angle_deg_{82.80};
  double qr_turn_yaw_tolerance_deg_{4.0};
  double qr_turn_settle_seconds_{0.20};
  double qr_turn_calib_linear_x_{0.14};
  double qr_turn_calib_angular_z_{0.72};
  double qr_hardcode_backup_speed_{-0.12};
  double qr_hardcode_backup_duration_{6.0};
  double qr_hardcode_turn_speed_{0.08};
  double qr_hardcode_turn_angular_z_{0.55};
  double qr_hardcode_turn_duration_{12.5};
  double qr_hardcode_forward_speed_{0.08};
  double qr_hardcode_forward_duration_{9.0};
  bool qr_hardcode_full_entry_{true};
  double qr_hardcode_entry_speed_{0.15};
  double qr_hardcode_entry_right_angular_z_{-0.56};
  double qr_hardcode_entry_right1_duration_{2.0};
  double qr_hardcode_entry_straight_duration_{4.0};
  double qr_hardcode_entry_right2_duration_{2.4};
  double qr_hardcode_entry_final_straight_duration_{2.0};
  double qr_hardcode_yellow_grace_duration_{3.0};
  bool qr_hardcode_disable_forbidden_recovery_{true};
  double qr_hardcode_yellow_entry_force_ratio_{0.50};
  double qr_hardcode_yellow_lap_grace_duration_{25.0};
  bool qr_hardcode_yellow_path_active_{false};
  bool qr_lap_green_recovery_used_{false};
  bool debug_stop_after_enter_yellow_{true};
  double qr_backup_stop_time_{0.0};
  double turn_to_gate_angular_z_{0.60};
  double turn_to_gate_duration_{1.5};
  double turn_to_gate_stop_time_{0.0};
  double turn_min_visual_stop_seconds_{1.6};
  double forward_to_gate_speed_{0.14};
  double yellow_gate_threshold_{0.15};
  double yellow_roi_pixel_area_{138240.0};
  double min_forward_to_gate_time_{0.5};
  double forward_to_gate_timeout_seconds_{10.0};
  int yellow_entry_detect_ticks_required_{4};
  int yellow_valid_frames_required_{6};
  double yellow_recovery_search_min_seconds_{0.8};
  double yellow_search_min_seconds_{2.0};
  double yellow_track_min_area_ratio_{0.25};
  double yellow_entry_min_area_{2500.0};
  double yellow_entry_forward_seconds_{4.20};
  double yellow_entry_forward_speed_{0.28};
  double yellow_entry_search_speed_{0.18};
  double yellow_entry_search_angular_{0.55};
  double yellow_entry_track_speed_{0.20};
  double yellow_entry_center_tolerance_{55.0};
  int yellow_entry_line_lock_ticks_{6};
  int yellow_entry_detect_ticks_{0};
  double yellow_entry_search_timeout_seconds_{3.50};
  double yellow_entry_gate_lock_tolerance_{85.0};
  int yellow_entry_gate_lock_ticks_{4};
  double yellow_entry_approach_max_angular_{0.10};
  double yellow_entry_grid_reject_tolerance_{115.0};
  double yellow_entry_grid_avoid_angular_{0.18};
  double yellow_entry_lane_width_min_{180.0};
  double yellow_entry_grid_max_ratio_{0.03};
  double forward_to_yellow_max_time_{4.0};
  double forward_to_yellow_min_time_{0.5};
  double yellow_enter_area_threshold_{0.18};
  int yellow_enter_confirm_frames_{5};
  int yellow_enter_confirm_count_{0};
  double yellow_center_kp_{0.003};
  int yellow_gate_guidance_lock_ticks_{0};
  bool yellow_gate_guidance_locked_{false};
  double yellow_entry_blind_angular_{0.0};
  double yellow_loop_speed_{0.24};
  double yellow_loop_turn_speed_{0.18};
  double yellow_loop_turn_angular_{0.72};
  double yellow_loop_entry_seconds_{0.70};
  double yellow_loop_branch_turn_seconds_{1.10};
  double yellow_loop_first_straight_seconds_{1.70};
  double yellow_loop_corner_turn_seconds_{0.95};
  double yellow_loop_side_seconds_{1.80};
  double yellow_loop_top_seconds_{2.10};
  double yellow_loop_second_side_seconds_{1.80};
  double yellow_loop_bottom_return_seconds_{1.70};
  double yellow_loop_exit_turn_seconds_{1.05};
  double yellow_loop_exit_seconds_{1.60};
  double return_speed_{0.45};
  double park_speed_{0.14};
  double line_kp_{0.0035};
  double lidar_center_kp_{0.8};
  double qr_align_kp_{0.003};
  double park_align_kp_{0.003};
  double front_stop_distance_{0.35};
  double front_slow_distance_{0.60};
  double qr_stop_area_{68000.0};
  double qr_fallback_area_{22000.0};
  double human_trigger_area_{26000.0};
  double parking_trigger_area_{28000.0};
  double obstacle_trigger_area_{18000.0};
  double obstacle_confirm_front_distance_{0.75};
  double obstacle_image_height_{480.0};
  double obstacle_near_bottom_ratio_{0.72};
  double obstacle_path_tolerance_{95.0};
  double obstacle_lidar_side_margin_{0.15};
  double obstacle_avoid_angular_{0.35};
  double obstacle_avoid_speed_{0.10};
  double obstacle_line_weight_{0.15};
  double obstacle_hold_seconds_{0.0};
  bool obstacle_return_enabled_{true};
  double obstacle_avoid_min_seconds_{0.45};
  double obstacle_return_seconds_{0.75};
  double obstacle_return_angular_{0.22};
  double obstacle_return_line_weight_{0.85};
  double obstacle_clear_tail_seconds_{0.0};
  double obstacle_clear_tail_angular_scale_{0.35};
  double obstacle_return_force_seconds_{0.90};
  double obstacle_reacquire_error_{80.0};
  int obstacle_reacquire_ticks_{4};
  ObstacleBypassPhase line_obstacle_phase_{ObstacleBypassPhase::IDLE};
  double line_obstacle_sign_{0.0};
  int line_obstacle_reacquire_ticks_{0};
  rclcpp::Time line_obstacle_phase_start_time_{0, 0, RCL_ROS_TIME};
  double obstacle_stop_duration_{0.2};
  double obstacle_backup_duration_{0.7};
  double obstacle_backup_speed_{0.10};
  double obstacle_turn_duration_{1.0};
  double obstacle_turn_speed_{0.35};
  int obstacle_recovery_max_count_{3};
  int obstacle_recovery_detect_frames_{4};
  double forbidden_stop_duration_{0.2};
  double forbidden_backup_duration_{1.2};
  double forbidden_backup_speed_{0.10};
  double forbidden_turn_duration_{1.0};
  double forbidden_turn_speed_{0.35};
  double green_forbidden_area_ratio_{0.12};
  double checker_forbidden_area_ratio_{0.10};
  double checker_backup_speed_{0.08};
  double checker_backup_duration_{1.0};
  int checker_recover_count_{0};
  double white_forbidden_roi_start_ratio_{0.60};
  double white_forbidden_threshold_{0.42};
  double white_seen_roi_start_ratio_{0.35};
  double white_seen_roi_end_ratio_{0.95};
  double white_seen_threshold_{0.42};
  double white_danger_roi_start_ratio_{0.88};
  double white_danger_roi_end_ratio_{1.00};
  double white_danger_threshold_{0.70};
  double forced_left_linear_speed_{0.04};
  double forced_left_angular_z_{0.55};
  double forced_left_turn_duration_{6.0};
  double approach_yellow_speed_{0.08};
  double approach_yellow_angular_z_{0.15};
  double approach_yellow_duration_{3.0};
  double white_emergency_threshold_{0.97};
  double entry_close_grid_threshold_{0.30};
  double entry_close_grid_soft_threshold_{0.18};
  double post_approach_candidate_grace_duration_{1.5};
  double post_approach_candidate_area_threshold_{0.10};
  double post_approach_hard_grid_threshold_{0.45};
  double forbidden_adjust_linear_speed_{0.03};
  double forbidden_adjust_left_angular_z_{0.06};
  double forbidden_adjust_duration_{0.5};
  bool entry_forbidden_recovering_{false};
  std::string entry_forbidden_reason_{};
  rclcpp::Time entry_forbidden_recover_start_time_{0, 0, RCL_ROS_TIME};
  double yellow_soft_seen_threshold_{0.03};
  double yellow_entry_turn_sign_{-1.0};
  double yellow_entry_kp_{0.003};
  double yellow_entry_max_angular_z_{0.18};
  double yellow_entry_area_threshold_{0.18};
  double yellow_entry_total_timeout_{35.0};
  bool block_static_right_steer_after_qr_{true};
  double static_right_linear_threshold_{0.02};
  int yellow_entry_confirm_frames_{5};
  double line_lost_speed_{0.0};
  double line_lost_search_angular_{0.0};
  int parking_required_ticks_{4};
  double parking_approach_after_seen_seconds_{2.5};
  double parking_approach_speed_{0.10};
  double loop_hold_seconds_{3.5};
  double qr_approach_timeout_seconds_{2.5};
  double return_to_p_seconds_{8.0};
  double image_center_x_{320.0};
  bool use_lidar_safety_{true};
  bool use_depth_safety_{true};
  double depth_stop_distance_{0.35};
  double depth_slow_distance_{0.60};
  double depth_unit_scale_{0.001};
  int depth_roi_width_{96};
  int depth_roi_height_{72};
  bool use_yellow_color_guidance_{true};
  double yellow_boundary_kp_{0.0028};
  /// 有巡线+有黄时：黄占该比例、巡线占 1-该值（0.7=黄优先）
  double yellow_entry_yellow_line_blend_{0.7};
  double yellow_min_area_for_guidance_{3500.0};
  double yellow_entry_line_kp_scale_{0.32};
  bool direct_yellow_entry_mode_{false};
  // 直接黄区入口锁定：面积像素、中心容差像素、连续帧数。
  double direct_yellow_entry_min_area_{9000.0};
  double direct_yellow_entry_center_tolerance_{65.0};
  int direct_yellow_entry_lock_ticks_{8};
  bool yellow_loop_closed_loop_{false};
  double yellow_loop_closed_loop_seconds_{18.0};
  double yellow_loop_closed_loop_speed_{0.20};
  double yellow_loop_entry_right1_duration_{2.5};
  double yellow_loop_entry_straight_duration_{5.0};
  double yellow_loop_entry_right2_duration_{3.5};
  double yellow_loop_entry_speed_{0.12};
  double yellow_loop_entry_right_angular_z_{-0.45};
  // 黄圈转角硬编码参数与状态。
  bool yellow_corner_hardcode_enabled_{true};
  double yellow_corner_green_threshold_{0.06};
  double yellow_corner_yellow_threshold_{0.45};
  double yellow_corner_hc_turn_duration_{1.80};
  double yellow_corner_hc_turn_angular_{0.70};
  double yellow_corner_hc_turn_speed_{0.10};
  double yellow_corner_cooldown_seconds_{4.50};
  bool yellow_corner_hc_active_{false};
  double yellow_corner_hc_sign_{0.0};
  rclcpp::Time yellow_corner_hc_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time yellow_corner_hc_cooldown_end_{0, 0, RCL_ROS_TIME};
  double yellow_loop_green_kp_{0.0022};
  double yellow_loop_yellow_kp_{0.0024};
  double yellow_loop_green_target_offset_{135.0};
  double yellow_loop_green_min_area_{1800.0};
  double yellow_loop_yellow_min_area_{2500.0};
  double yellow_loop_fallback_angular_{0.28};
  double yellow_loop_green_weight_{0.75};
  double yellow_loop_green_danger_area_{35000.0};
  double yellow_loop_green_danger_tolerance_{95.0};
  double yellow_loop_green_danger_angular_{0.42};
  double yellow_loop_green_danger_speed_scale_{0.55};
  // 黄区车道闭环控制参数：速度 m/s，角速度 rad/s，面积/颜色比例 0~1。
  double yellow_lane_speed_{0.10};
  double yellow_lane_kp_{0.003};
  double yellow_lane_max_angular_z_{0.45};
  double yellow_lane_alpha_{0.40};
  double yellow_lane_beta_{0.10};
  // 黄车道 Alpha-Beta 滤波器状态：平滑位置、速度估计、上一帧时间戳。
  double yellow_lane_filter_x_{320.0};
  double yellow_lane_filter_v_{0.0};
  bool yellow_lane_filter_initialized_{false};
  rclcpp::Time yellow_lane_filter_last_time_{0, 0, RCL_ROS_TIME};
  bool yellow_corner_boost_enabled_{false};
  double yellow_corner_trigger_error_{155.0};
  double yellow_corner_turn_speed_{0.18};
  double yellow_corner_turn_angular_{0.68};
  double yellow_corner_turn_duration_{0.85};
  double yellow_corner_exit_straight_duration_{0.35};
  double yellow_corner_cooldown_{1.6};
  double yellow_exit_corner_inhibit_seconds_{1.2};
  bool yellow_corner_boost_active_{false};
  int yellow_corner_boost_phase_{0};
  double yellow_corner_boost_sign_{0.0};
  rclcpp::Time yellow_corner_boost_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time yellow_corner_last_end_time_{0, 0, RCL_ROS_TIME};
  bool yellow_obstacle_allow_green_{true};
  double yellow_obstacle_trigger_area_{3500.0};
  double yellow_obstacle_confirm_front_distance_{0.70};
  double yellow_obstacle_path_tolerance_{85.0};
  double yellow_obstacle_avoid_speed_{0.15};
  double yellow_obstacle_avoid_seconds_{0.70};
  double yellow_obstacle_return_seconds_{0.90};
  double yellow_obstacle_return_angular_{0.30};
  double yellow_obstacle_return_line_weight_{0.85};
  double yellow_obstacle_reacquire_error_{100.0};
  int yellow_obstacle_reacquire_ticks_{4};
  ObstacleBypassPhase yellow_obstacle_phase_{ObstacleBypassPhase::IDLE};
  double yellow_obstacle_sign_{0.0};
  int yellow_obstacle_reacquire_count_{0};
  rclcpp::Time yellow_obstacle_phase_start_time_{0, 0, RCL_ROS_TIME};
  double lane_width_min_{60.0};
  double lane_width_max_{500.0};
  double lane_width_estimate_init_{220.0};
  double yellow_min_area_ratio_{0.03};
  double green_forbidden_threshold_{0.08};
  double green_filter_alpha_{0.30};
  int green_danger_confirm_ticks_{4};
  int green_clear_confirm_ticks_{3};
  double yellow_lap_green_recovery_center_ratio_{0.20};
  double yellow_lap_green_recovery_area_ratio_{0.45};
  double yellow_lap_green_recovery_cooldown_{6.0};
  double yellow_lap_green_guard_side_ratio_{0.16};
  double yellow_lap_green_guard_center_ratio_{0.04};
  double yellow_lap_green_guard_strong_side_ratio_{0.30};
  double yellow_lap_green_guard_strong_center_ratio_{0.06};
  double yellow_lap_green_guard_bias_angular_{0.18};
  double yellow_lap_green_guard_force_angular_{0.55};
  bool checker_forbidden_enabled_{true};
  double grid_forbidden_threshold_{0.42};
  double yellow_lane_lost_timeout_{0.8};
  double yellow_lane_history_keep_time_{0.5};
  double yellow_lane_safe_speed_{0.04};
  double yellow_lane_escape_angular_{0.35};
  double grid_yellow_ratio_threshold_{0.05};
  double grid_white_ratio_threshold_{0.42};
  double grid_near_roi_start_ratio_{0.60};
  double white_grid_reject_threshold_{0.42};
  double yellow_lane_area_threshold_{0.12};
  double grid_recover_backup_speed_{0.08};
  double grid_recover_backup_duration_{0.6};
  double grid_recover_search_angular_z_{0.25};
  double grid_recover_search_duration_{1.2};
  int grid_recover_max_count_{2};
  double forward_to_yellow_speed_{0.08};
  double yellow_search_linear_speed_{0.04};
  double yellow_search_angular_z_{0.08};
  double yellow_candidate_area_threshold_{0.04};
  double yellow_candidate_forward_speed_{0.08};
  double yellow_candidate_left_angular_z_{0.03};
  double yellow_candidate_duration_{1.5};
  bool yellow_candidate_allow_right_steer_{true};
  double yellow_candidate_max_right_angular_z_{0.12};
  double yellow_candidate_kp_{0.003};
  // 强制前探黄区门口动作：速度 m/s、角速度 rad/s、持续时间秒。
  double forced_approach_gate_speed_{0.10};
  double forced_approach_gate_angular_z_{0.05};
  double forced_approach_gate_duration_{4.0};
  bool forced_approach_gate_done_logged_{false};
  rclcpp::Time post_approach_candidate_grace_start_time_{0, 0, RCL_ROS_TIME};
  bool grid_recovering_{false};
  int grid_recover_phase_{0};
  rclcpp::Time grid_recover_start_time_{0, 0, RCL_ROS_TIME};
  int grid_recover_count_{0};
  bool yellow_candidate_active_{false};
  rclcpp::Time yellow_candidate_start_time_{0, 0, RCL_ROS_TIME};
  // 黄区绕行出口/返程参数：速度 m/s、时间秒、比例阈值 0~1。
  double ccw_bias_{0.0};
  double ccw_bias_max_{0.08};
  double yellow_speed_{0.12};
  double yellow_kp_{0.003};
  double yellow_max_angular_z_{0.5};
  double yellow_lost_threshold_{0.03};
  double yellow_search_speed_{0.04};
  double clockwise_target_ratio_{0.60};
  double counterclockwise_target_ratio_{0.40};
  double min_lap_time_{12.0};
  double expected_lap_time_{90.0};
  double yellow_lap_line_return_min_time_{110.0};
  int yellow_lap_line_return_ticks_required_{10};
  double yellow_exit_green_gap_min_time_{60.0};
  double yellow_exit_green_left_threshold_{0.12};
  double yellow_exit_green_right_clear_threshold_{0.03};
  double yellow_exit_right_yellow_threshold_{0.08};
  int yellow_exit_green_gap_ticks_required_{6};
  int yellow_exit_arm_ticks_required_{8};
  double yellow_exit_arm_inner_green_threshold_{0.08};
  double yellow_exit_side_yellow_min_time_{60.0};
  double yellow_exit_side_yellow_threshold_{0.60};
  double yellow_exit_side_yellow_green_clear_{0.08};
  double yellow_exit_yellow_side_turn_sign_{-1.0};
  double exit_gate_yellow_ratio_threshold_{0.06};
  double exit_gate_align_speed_{0.08};
  double exit_gate_align_seconds_{3.0};
  double exit_gate_turn_angular_z_{-0.55};
  double exit_gate_visual_timeout_seconds_{12.0};
  double yellow_exit_turn_sign_{-1.0};
  int yellow_exit_arm_ticks_{0};
  bool yellow_exit_armed_{false};
  bool yellow_exit_candidate_seen_{false};
  bool yellow_exit_candidate_cleared_{false};
  double forward_exit_gate_speed_{0.12};
  double min_forward_exit_gate_time_{1.2};
  double back_line_speed_{0.14};
  double min_back_time_{3.0};
  double line_lost_finish_forward_time_{0.4};
  double line_lost_finish_speed_{0.08};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stage_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr direction_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_text_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr track_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr yellow_boundary_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr green_boundary_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr yellow_lane_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hardcode_override_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detect_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr qr_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr qr_hardcode_done_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gate_aligned_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr yellow_entry_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loop_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr park_zone_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parked_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exit_gate_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr line_reacquired_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr return_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr human_result_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace

// 工厂函数：race_main 通过该入口创建 motion_controller 节点。
std::shared_ptr<rclcpp::Node> create_motion_controller() {
  return std::make_shared<MotionControllerNode>();
}

}  // namespace fangan_core
