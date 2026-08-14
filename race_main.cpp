#include <cstdio>
#include <memory>

#include "fangan_core/mission_manager.hpp"
#include "fangan_core/motion_controller.hpp"
#include "fangan_core/report_manager.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  std::fprintf(stderr, "[fangan_core] main entered\n");
  rclcpp::init(argc, argv);
  std::fprintf(stderr, "[fangan_core] rclcpp initialized\n");

  rclcpp::executors::MultiThreadedExecutor executor;

  auto mission_manager = fangan_core::create_mission_manager();
  std::fprintf(stderr, "[fangan_core] mission_manager created\n");
  auto motion_controller = fangan_core::create_motion_controller();
  std::fprintf(stderr, "[fangan_core] motion_controller created\n");
  auto report_manager = fangan_core::create_report_manager();
  std::fprintf(stderr, "[fangan_core] report_manager created\n");

  executor.add_node(mission_manager);
  std::fprintf(stderr, "[fangan_core] mission_manager added to executor\n");
  executor.add_node(motion_controller);
  std::fprintf(stderr, "[fangan_core] motion_controller added to executor\n");
  executor.add_node(report_manager);
  std::fprintf(stderr, "[fangan_core] report_manager added to executor\n");
  std::fprintf(stderr, "[fangan_core] executor spinning\n");

  executor.spin();
  std::fprintf(stderr, "[fangan_core] executor stopped\n");
  rclcpp::shutdown();
  std::fprintf(stderr, "[fangan_core] shutdown complete\n");
  return 0;
}
