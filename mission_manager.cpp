#include "fangan_core/mission_manager.hpp"

#include <chrono>
#include <string>

#include "fangan_core/logic.hpp"
#include "fangan_core/race_context.hpp"
#include "fangan_core/race_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace fangan_core {

namespace {

std::string stage_to_string(MissionStage stage) {
  return to_string(stage);
}

class MissionManagerNode : public rclcpp::Node {
 public:
  MissionManagerNode() : Node("mission_manager") {
    RCLCPP_INFO(get_logger(), "mission_manager constructor entered");
    declare_parameter<bool>("allow_qr_detection_fallback", true);
    declare_parameter<std::string>("fallback_qr_text", "clockwise");
    declare_parameter<double>("parse_qr_hold_seconds", 0.25);
    declare_parameter<double>("qr_backup_duration", 0.8);
    declare_parameter<double>("qr_backup_stop_time", 0.0);
    declare_parameter<double>("turn_duration", 1.5);
    declare_parameter<double>("turn_to_gate_duration", 1.5);
    declare_parameter<double>("turn_to_gate_stop_time", 0.0);
    declare_parameter<double>("qr_forward_duration", 1.5);
    declare_parameter<double>("exit_gate_align_seconds", 15.0);
    declare_parameter<bool>("debug_stop_after_enter_yellow", true);
    declare_parameter<bool>("use_qr_hardcode_entry", false);
    declare_parameter<bool>("qr_hardcode_full_entry", true);
    declare_parameter<double>("qr_hardcode_backup_duration", 4.5);
    declare_parameter<double>("qr_hardcode_turn_duration", 12.0);
    declare_parameter<double>("qr_hardcode_forward_duration", 9.0);
    declare_parameter<double>("qr_hardcode_entry_right1_duration", 2.0);
    declare_parameter<double>("qr_hardcode_entry_straight_duration", 4.0);
    declare_parameter<double>("qr_hardcode_entry_right2_duration", 2.4);
    declare_parameter<double>("qr_hardcode_entry_final_straight_duration", 2.0);
    get_parameter("allow_qr_detection_fallback", allow_qr_detection_fallback_);
    get_parameter("fallback_qr_text", fallback_qr_text_);
    get_parameter("parse_qr_hold_seconds", parse_qr_hold_seconds_);
    get_parameter("qr_backup_duration", qr_backup_duration_);
    get_parameter("qr_backup_stop_time", qr_backup_stop_time_);
    get_parameter("turn_duration", turn_duration_);
    get_parameter("turn_to_gate_duration", turn_to_gate_duration_);
    get_parameter("turn_to_gate_stop_time", turn_to_gate_stop_time_);
    get_parameter("qr_forward_duration", qr_forward_duration_);
    get_parameter("exit_gate_align_seconds", exit_gate_align_seconds_);
    get_parameter("debug_stop_after_enter_yellow", debug_stop_after_enter_yellow_);
    get_parameter("use_qr_hardcode_entry", use_qr_hardcode_entry_);
    get_parameter("qr_hardcode_full_entry", qr_hardcode_full_entry_);
    get_parameter("qr_hardcode_backup_duration", qr_hardcode_backup_duration_);
    get_parameter("qr_hardcode_turn_duration", qr_hardcode_turn_duration_);
    get_parameter("qr_hardcode_forward_duration", qr_hardcode_forward_duration_);
    get_parameter("qr_hardcode_entry_right1_duration",
                  qr_hardcode_entry_right1_duration_);
    get_parameter("qr_hardcode_entry_straight_duration",
                  qr_hardcode_entry_straight_duration_);
    get_parameter("qr_hardcode_entry_right2_duration",
                  qr_hardcode_entry_right2_duration_);
    get_parameter("qr_hardcode_entry_final_straight_duration",
                  qr_hardcode_entry_final_straight_duration_);

    stage_pub_ = create_publisher<std_msgs::msg::String>("/race/stage", 10);
    direction_pub_ = create_publisher<std_msgs::msg::String>("/race/direction", 10);
    task_pub_ = create_publisher<std_msgs::msg::String>("/race/report_text", 10);

    start_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/start", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data) {
            return;
          }
          context_.start_received = true;
          context_.qr_text.clear();
          context_.qr_locked = false;
          context_.direction = TurnDirection::UNKNOWN;
          qr_reached_waiting_for_decode_ = false;
          transition_to(MissionStage::LINE_TO_QR);
        });

    qr_sub_ = create_subscription<std_msgs::msg::String>(
        "/qr_code_result", 10,
        [this](const std_msgs::msg::String::ConstSharedPtr msg) {
          if (msg->data.empty()) {
            return;
          }
          context_.qr_text = msg->data;
          context_.qr_locked = true;
          context_.direction = parse_turn_direction(context_.qr_text);
          RCLCPP_INFO(get_logger(), "qr decoded text='%s' direction=%s",
                      context_.qr_text.c_str(),
                      to_string(context_.direction).c_str());
          publish_task_text("QR task: " + context_.qr_text);
          if (context_.stage == MissionStage::LINE_TO_QR &&
              qr_reached_waiting_for_decode_) {
            RCLCPP_INFO(get_logger(),
                        "qr decoded after reach; continue with qr text: %s",
                        context_.qr_text.c_str());
            qr_reached_waiting_for_decode_ = false;
            transition_to(MissionStage::QR_DETECT);
          }
        });

    qr_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/qr_reached", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data) {
            return;
          }
          if (context_.stage == MissionStage::LINE_TO_QR) {
            if (!context_.qr_locked) {
              if (!allow_qr_detection_fallback_) {
                qr_reached_waiting_for_decode_ = true;
                RCLCPP_WARN(
                    get_logger(),
                    "qr reached but no decoded text yet; waiting for /qr_code_result");
                return;
              }
              context_.qr_text = fallback_qr_text_;
              context_.qr_locked = true;
              RCLCPP_WARN(
                  get_logger(),
                  "qr detection fallback active; using fallback qr text: %s",
                  context_.qr_text.c_str());
              publish_task_text("QR fallback task: " + context_.qr_text);
            }
            qr_reached_waiting_for_decode_ = false;
            transition_to(MissionStage::QR_DETECT);
          }
        });

    qr_hardcode_done_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/qr_hardcode_done", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data || !use_qr_hardcode_entry_) {
            return;
          }
          if (context_.stage == MissionStage::QR_HARDCODE_TURN_1) {
            RCLCPP_INFO(get_logger(),
                        "[MISSION] qr turn 1 complete -> QR_ENTRY_ADVANCE");
            transition_to(MissionStage::QR_ENTRY_ADVANCE);
          } else if (context_.stage == MissionStage::QR_ENTRY_ADVANCE) {
            RCLCPP_INFO(get_logger(),
                        "[MISSION] qr entry advance complete -> QR_HARDCODE_TURN_2");
            transition_to(MissionStage::QR_HARDCODE_TURN_2);
          } else if (context_.stage == MissionStage::QR_HARDCODE_TURN_2) {
            RCLCPP_INFO(get_logger(),
                        "[MISSION] qr turn 2 complete -> YELLOW_SEARCH");
            transition_to(MissionStage::YELLOW_SEARCH);
          } else if (context_.stage == MissionStage::QR_DETECT ||
              context_.stage == MissionStage::QR_HARDCODE_ENTRY ||
              context_.stage == MissionStage::BACK_FROM_QR ||
              context_.stage == MissionStage::TURN_TO_GATE ||
              context_.stage == MissionStage::FORWARD_TO_GATE) {
            if (!context_.qr_locked ||
                context_.direction == TurnDirection::UNKNOWN) {
              RCLCPP_WARN(
                  get_logger(),
                  "ignore qr_hardcode_done before valid qr decode: locked=%d direction=%s",
                  context_.qr_locked, to_string(context_.direction).c_str());
              return;
            }
            if (qr_hardcode_full_entry_) {
              RCLCPP_INFO(get_logger(),
                          "[MISSION] qr_hardcode_done -> YELLOW_LAP");
              transition_to(MissionStage::YELLOW_LAP);
            } else {
              RCLCPP_INFO(get_logger(),
                          "[MISSION] qr_hardcode_done -> YELLOW_SEARCH");
              transition_to(MissionStage::YELLOW_SEARCH);
            }
          }
        });

    yellow_entry_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/yellow_entry_reached", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data ||
              (context_.stage != MissionStage::FORWARD_TO_GATE &&
               context_.stage != MissionStage::YELLOW_SEARCH)) {
            return;
          }
          if (debug_stop_after_enter_yellow_) {
            RCLCPP_INFO(get_logger(), "[MISSION] FORWARD_TO_YELLOW -> PARK_FINISH");
            transition_to(MissionStage::STOP);
          } else {
            transition_to(MissionStage::YELLOW_LAP);
          }
        });

    gate_aligned_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/gate_aligned", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data || context_.stage != MissionStage::TURN_TO_GATE ||
              debug_stop_after_enter_yellow_) {
            return;
          }
          transition_to(MissionStage::FORWARD_TO_GATE);
        });

    loop_done_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/loop_complete", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data || context_.stage != MissionStage::YELLOW_LAP) {
            return;
          }
          transition_to(MissionStage::EXIT_GATE);
        });

    exit_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/exit_gate_ready", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data || context_.stage != MissionStage::EXIT_GATE) {
            return;
          }
          transition_to(MissionStage::FORWARD_EXIT_GATE);
        });

    line_reacquired_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/line_reacquired", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data ||
              context_.stage != MissionStage::FORWARD_EXIT_GATE) {
            return;
          }
          transition_to(MissionStage::LINE_BACK_HOME);
        });

    return_complete_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/return_complete", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data || context_.stage != MissionStage::LINE_BACK_HOME) {
            return;
          }
          transition_to(MissionStage::STOP);
        });

    report_done_sub_ = create_subscription<std_msgs::msg::String>(
        "/human/report_done", 10,
        [this](const std_msgs::msg::String::ConstSharedPtr msg) {
          if (msg->data.empty()) {
            return;
          }
          context_.report_text = msg->data;
          context_.human_reported = true;
        });

    park_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/park_zone_reached", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data || context_.stage != MissionStage::LINE_BACK_HOME) {
            return;
          }
          transition_to(MissionStage::STOP);
        });

    parked_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/race/parked", 10,
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (!msg->data || context_.stage != MissionStage::STOP) {
            return;
          }
          context_.parked = true;
        });

    heartbeat_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {
      const double stage_seconds = (now() - stage_enter_time_).seconds();
      if (context_.stage == MissionStage::QR_DETECT &&
          stage_seconds >= parse_qr_hold_seconds_) {
        if (!context_.qr_locked ||
            context_.direction == TurnDirection::UNKNOWN) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "hold QR_DETECT: waiting for valid decoded qr text, locked=%d direction=%s",
              context_.qr_locked, to_string(context_.direction).c_str());
          publish_stage();
          publish_direction();
          return;
        }
        if (use_qr_hardcode_entry_) {
          transition_to(MissionStage::QR_HARDCODE_TURN_1);
        } else {
          transition_to(MissionStage::BACK_FROM_QR);
        }
      } else if (context_.stage == MissionStage::QR_HARDCODE_ENTRY &&
                 stage_seconds >= qr_hardcode_total_duration()) {
        transition_to(qr_hardcode_full_entry_ ? MissionStage::YELLOW_LAP
                                              : MissionStage::YELLOW_SEARCH);
      } else if (context_.stage == MissionStage::BACK_FROM_QR &&
                 stage_seconds >= qr_backup_duration_ + qr_backup_stop_time_) {
        transition_to(MissionStage::TURN_TO_GATE);
      } else if (context_.stage == MissionStage::TURN_TO_GATE &&
                 stage_seconds >= turn_to_gate_duration_ + turn_to_gate_stop_time_) {
        transition_to(MissionStage::FORWARD_TO_GATE);
      } else if (context_.stage == MissionStage::FORWARD_TO_GATE &&
                 !debug_stop_after_enter_yellow_ &&
                 stage_seconds >= qr_forward_duration_) {
        transition_to(MissionStage::YELLOW_SEARCH);
      } else if (context_.stage == MissionStage::EXIT_GATE &&
                 stage_seconds >= exit_gate_align_seconds_) {
        transition_to(MissionStage::FORWARD_EXIT_GATE);
      }
      publish_stage();
      publish_direction();
    });

    publish_stage();
    publish_direction();
    RCLCPP_INFO(get_logger(), "mission_manager ready");
  }

 private:
  void transition_to(MissionStage next) {
    const auto previous = context_.stage;
    if (previous == next) {
      return;
    }
    if (next == MissionStage::QR_DETECT) {
      context_.direction = parse_turn_direction(context_.qr_text);
    }

    context_.stage = next;
    stage_enter_time_ = now();
    RCLCPP_INFO(get_logger(), "[MISSION] %s -> %s",
                stage_to_string(previous).c_str(),
                stage_to_string(context_.stage).c_str());
    publish_stage();
    publish_direction();
  }

  void publish_stage() {
    std_msgs::msg::String msg;
    msg.data = stage_to_string(context_.stage);
    stage_pub_->publish(msg);
  }

  void publish_direction() {
    std_msgs::msg::String msg;
    msg.data = to_string(context_.direction);
    direction_pub_->publish(msg);
  }

  void publish_task_text(const std::string &text) {
    std_msgs::msg::String msg;
    msg.data = text;
    task_pub_->publish(msg);
  }

  double qr_hardcode_total_duration() const {
    double duration = qr_hardcode_backup_duration_ + qr_hardcode_turn_duration_ +
                      qr_hardcode_forward_duration_;
    if (qr_hardcode_full_entry_) {
      duration += qr_hardcode_entry_right1_duration_ +
                  qr_hardcode_entry_straight_duration_ +
                  qr_hardcode_entry_right2_duration_ +
                  qr_hardcode_entry_final_straight_duration_;
    }
    return duration;
  }

  RaceContext context_;
  bool allow_qr_detection_fallback_{true};
  std::string fallback_qr_text_{"clockwise"};
  double parse_qr_hold_seconds_{0.25};
  double qr_backup_duration_{0.8};
  double qr_backup_stop_time_{0.0};
  double turn_duration_{1.5};
  double turn_to_gate_duration_{1.5};
  double turn_to_gate_stop_time_{0.0};
  double qr_forward_duration_{1.5};
  double exit_gate_align_seconds_{3.0};
  bool debug_stop_after_enter_yellow_{true};
  bool use_qr_hardcode_entry_{false};
  bool qr_hardcode_full_entry_{true};
  bool qr_reached_waiting_for_decode_{false};
  double qr_hardcode_backup_duration_{4.5};
  double qr_hardcode_turn_duration_{12.0};
  double qr_hardcode_forward_duration_{9.0};
  double qr_hardcode_entry_right1_duration_{2.0};
  double qr_hardcode_entry_straight_duration_{4.0};
  double qr_hardcode_entry_right2_duration_{2.4};
  double qr_hardcode_entry_final_straight_duration_{2.0};
  rclcpp::Time stage_enter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr qr_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr qr_hardcode_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gate_aligned_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr yellow_entry_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loop_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exit_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr line_reacquired_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr return_complete_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr report_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr park_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr parked_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> create_mission_manager() {
  return std::make_shared<MissionManagerNode>();
}

}  // namespace fangan_core
