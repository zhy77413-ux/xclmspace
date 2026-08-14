#include "fangan_core/report_manager.hpp"

#include <string>

#include "fangan_core/logic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace fangan_core {

namespace {

class ReportManagerNode : public rclcpp::Node {
 public:
  ReportManagerNode() : Node("report_manager") {
    RCLCPP_INFO(get_logger(), "report_manager constructor entered");
    report_pub_ = create_publisher<std_msgs::msg::String>("/race/report_text", 10);
    done_pub_ = create_publisher<std_msgs::msg::String>("/human/report_done", 10);

    result_sub_ = create_subscription<std_msgs::msg::String>(
        "/human/result", 10,
        [this](const std_msgs::msg::String::ConstSharedPtr msg) {
          if (msg->data.empty()) {
            return;
          }

          std_msgs::msg::String out;
          out.data = build_report_text(msg->data);
          report_pub_->publish(out);
          done_pub_->publish(out);
        });

    RCLCPP_INFO(get_logger(), "report_manager ready");
  }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr done_pub_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> create_report_manager() {
  return std::make_shared<ReportManagerNode>();
}

}  // namespace fangan_core
