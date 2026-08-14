#include <cstdio>

#include "fangan_core/motion_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  std::fprintf(stderr, "[fangan_core] yellow_ring_controller entered\n");
  rclcpp::init(argc, argv);
  auto controller = fangan_core::create_motion_controller();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
