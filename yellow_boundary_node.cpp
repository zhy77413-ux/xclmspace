// 大厅→黄「通道」：保留黄色/绿色质心话题用于旧入口逻辑。
// 黄区环道：额外发布黄绿边界和禁区信息，主控据此保持通道居中。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/header.hpp"

namespace yellow_boundary {

class YellowBoundaryNode : public rclcpp::Node {
 public:
  // 构造函数：声明/读取黄区视觉参数，建立图像订阅和黄/绿边界发布器。
  YellowBoundaryNode() : Node("yellow_boundary_node") {
    // sub_img_topic：输入图像话题；frame_id：输出质心消息坐标系名称。
    declare_parameter<std::string>("sub_img_topic", "/image");
    declare_parameter<std::string>("frame_id", "default_usb_cam");
    // roi_bottom_ratio：只检测图像底部该比例区域，单位为整幅图高度占比。
    declare_parameter<double>("roi_bottom_ratio", 0.48);
    // 黄色 HSV 阈值：h_min/h_max 为色相范围，s_min/v_min 为饱和度/亮度下限。
    declare_parameter<int>("h_min", 18);
    declare_parameter<int>("h_max", 38);
    declare_parameter<int>("s_min", 80);
    declare_parameter<int>("v_min", 80);
    // min_yellow_area：黄色连通域最小面积，单位为像素；morph_kernel：形态学核尺寸，单位为像素。
    declare_parameter<int>("min_yellow_area", 2500);
    declare_parameter<int>("morph_kernel", 5);
    // 多黄块/二维码黄边时，优先选「质心更靠近画面水平中心、面积又够大」的块：score = area/(1+k*|cx-w/2|)
    declare_parameter<double>("center_score_k", 0.012);
    // 对质心列做 EMA，减轻在左右大跳
    declare_parameter<double>("cx_ema_alpha", 0.35);
    // 纯黄通道是实心矩形(填充率高)；"黄白格"禁区是稀疏菱形(bbox 填充率低)
    // 该阈值之下的连通域被丢弃，避免把格子当通道
    declare_parameter<double>("min_fill_rate", 0.55);
    // 竖直长宽比下限(h/w)：通道是竖长矩形；格子拼成的块近似横条(h/w 小)
    declare_parameter<double>("min_aspect_h_over_w", 0.55);
    // 绿色 HSV/形态约束：用于识别黄区环道边界和绿色禁入区域。
    declare_parameter<double>("green_roi_bottom_ratio", 1.0);
    declare_parameter<int>("green_h_min", 35);
    declare_parameter<int>("green_h_max", 85);
    declare_parameter<int>("green_s_min", 50);
    declare_parameter<int>("green_v_min", 50);
    declare_parameter<int>("min_green_area", 2500);
    declare_parameter<double>("green_min_fill_rate", 0.25);
    declare_parameter<double>("green_min_aspect_h_over_w", 0.10);
    // 白色 HSV 阈值：用于和黄色重叠判断黄白格子禁入逻辑。
    declare_parameter<int>("white_h_min", 0);
    declare_parameter<int>("white_h_max", 180);
    declare_parameter<int>("white_s_max", 60);
    declare_parameter<int>("white_v_min", 180);
    // ROI 比例参数：取图像纵向范围，单位为高度占比，用于统计近处黄/绿/白区域比例。
    declare_parameter<double>("roi_y_start_ratio", 0.55);
    declare_parameter<double>("roi_y_end_ratio", 0.95);
    declare_parameter<double>("near_roi_y_start_ratio", 0.75);
    declare_parameter<double>("grid_near_roi_start_ratio", 0.60);
    // scan_row_ratios：多条横向扫描线位置，单位为高度占比；scan_band_height：扫描带高度，单位为像素。
    declare_parameter<std::vector<double>>("scan_row_ratios", {0.60, 0.70, 0.80, 0.90});
    declare_parameter<int>("scan_band_height", 12);
    // lane_width_min/max：黄色通道宽度合法范围，单位为像素。
    declare_parameter<int>("lane_width_min", 60);
    declare_parameter<int>("lane_width_max", 500);
    // 面积比例阈值：用于判断黄色通道、绿色禁区和黄白格子是否可见。
    declare_parameter<double>("yellow_min_area_ratio", 0.03);
    declare_parameter<double>("green_forbidden_threshold", 0.08);
    declare_parameter<double>("grid_forbidden_threshold", 0.08);

    get_parameter("sub_img_topic", sub_img_topic_);
    get_parameter("frame_id", frame_id_);
    get_parameter("roi_bottom_ratio", roi_bottom_ratio_);
    get_parameter("h_min", h_min_);
    get_parameter("h_max", h_max_);
    get_parameter("s_min", s_min_);
    get_parameter("v_min", v_min_);
    get_parameter("min_yellow_area", min_yellow_area_);
    get_parameter("morph_kernel", morph_kernel_);
    get_parameter("center_score_k", center_score_k_);
    get_parameter("cx_ema_alpha", cx_ema_alpha_);
    get_parameter("min_fill_rate", min_fill_rate_);
    get_parameter("min_aspect_h_over_w", min_aspect_h_over_w_);
    get_parameter("green_roi_bottom_ratio", green_roi_bottom_ratio_);
    get_parameter("green_h_min", green_h_min_);
    get_parameter("green_h_max", green_h_max_);
    get_parameter("green_s_min", green_s_min_);
    get_parameter("green_v_min", green_v_min_);
    get_parameter("min_green_area", min_green_area_);
    get_parameter("green_min_fill_rate", green_min_fill_rate_);
    get_parameter("green_min_aspect_h_over_w", green_min_aspect_h_over_w_);
    get_parameter("white_h_min", white_h_min_);
    get_parameter("white_h_max", white_h_max_);
    get_parameter("white_s_max", white_s_max_);
    get_parameter("white_v_min", white_v_min_);
    get_parameter("roi_y_start_ratio", roi_y_start_ratio_);
    get_parameter("roi_y_end_ratio", roi_y_end_ratio_);
    get_parameter("near_roi_y_start_ratio", near_roi_y_start_ratio_);
    get_parameter("grid_near_roi_start_ratio", grid_near_roi_start_ratio_);
    // 如果配置了近处格子 ROI 起点，则复用为近处白/黄统计区域起点。
    if (grid_near_roi_start_ratio_ > 0.0 && grid_near_roi_start_ratio_ < 1.0) {
      near_roi_y_start_ratio_ = grid_near_roi_start_ratio_;
    }
    get_parameter("scan_row_ratios", scan_row_ratios_);
    get_parameter("scan_band_height", scan_band_height_);
    get_parameter("lane_width_min", lane_width_min_);
    get_parameter("lane_width_max", lane_width_max_);
    get_parameter("yellow_min_area_ratio", yellow_min_area_ratio_);
    get_parameter("green_forbidden_threshold", green_forbidden_threshold_);
    get_parameter("grid_forbidden_threshold", grid_forbidden_threshold_);

    pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/yellow_boundary_center", 10);
    green_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/green_boundary_center", 10);
    lane_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/yellow_lane_info", 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        sub_img_topic_, rclcpp::SensorDataQoS(),
        std::bind(&YellowBoundaryNode::onImage, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "yellow_boundary: sub=%s pub=/yellow_boundary_center /green_boundary_center /yellow_lane_info",
                sub_img_topic_.c_str());
  }

 private:
  // 在指定 HSV 阈值和底部 ROI 中寻找颜色质心；返回最佳连通域的 x 坐标和面积。
  bool findColorCenter(const cv::Mat &bgr,
                       double roi_bottom_ratio,
                       int h_min,
                       int h_max,
                       int s_min,
                       int v_min,
                       int min_area,
                       double min_fill_rate,
                       double min_aspect_h_over_w,
                       double &cx_raw,
                       double &best_area) const {
    const int h = bgr.rows;
    const int w = bgr.cols;
    // y0：底部 ROI 起始行，roi_bottom_ratio 为高度占比。
    const int y0 = std::max(0, static_cast<int>(h * (1.0 - roi_bottom_ratio)));
    const cv::Rect roi(0, y0, w, h - y0);
    cv::Mat slice = bgr(roi);

    cv::Mat hsv;
    cv::cvtColor(slice, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, 255, 255), mask);

    // 形态学开/闭运算：去噪并填补小孔，核尺寸单位为像素。
    const int k = std::max(1, morph_kernel_ | 1);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double w_f = static_cast<double>(w);
    double best_score = -1.0;
    best_area = 0.0;
    cx_raw = 0.0;
    for (const auto &c : contours) {
      const double a = std::abs(cv::contourArea(c));
      // 面积太小的颜色块视为噪声，不参与通道/边界判断。
      if (a < static_cast<double>(min_area)) {
        continue;
      }
      const cv::Rect br = cv::boundingRect(c);
      const double bbox_area = std::max(1.0, static_cast<double>(br.width * br.height));
      const double fill_rate = a / bbox_area;
      const double aspect = static_cast<double>(br.height) /
                            std::max(1.0, static_cast<double>(br.width));
      // 填充率和纵横比过滤：排除黄白格子、横向反光块等非通道形状。
      if (fill_rate < min_fill_rate || aspect < min_aspect_h_over_w) {
        continue;
      }
      const cv::Moments m = cv::moments(c);
      // 零面积矩无法计算质心，直接跳过。
      if (m.m00 <= 1e-6) {
        continue;
      }
      const double cx_roi = m.m10 / m.m00;
      const double dist = std::abs(cx_roi - 0.5 * w_f);
      // 评分兼顾面积和居中程度：优先选择靠近画面中心的大色块。
      const double score = a / (1.0 + center_score_k_ * dist);
      if (score > best_score) {
        best_score = score;
        best_area = a;
        cx_raw = cx_roi;
      }
    }

    return best_score >= 0.0;
  }

  // 对质心做指数滑动平均，减少边界检测在帧间跳变。
  double smoothValue(double raw, bool &initialized, double &smooth) const {
    if (!initialized) {
      smooth = raw;
      initialized = true;
      return smooth;
    }
    const double a = std::clamp(cx_ema_alpha_, 0.05, 0.99);
    smooth = a * raw + (1.0 - a) * smooth;
    return smooth;
  }

  // 统计指定 ROI 中非零像素占比，返回 0~1 的面积比例。
  static double ratioNonZero(const cv::Mat &mask, const cv::Rect &roi) {
    if (roi.width <= 0 || roi.height <= 0) {
      return 0.0;
    }
    const double area = static_cast<double>(roi.width * roi.height);
    return static_cast<double>(cv::countNonZero(mask(roi))) / std::max(1.0, area);
  }

  // 计算中位数：融合多条扫描线结果，降低单行噪声影响。
  static double median(std::vector<double> values) {
    if (values.empty()) {
      return -1.0;
    }
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    double result = values[mid];
    if (values.size() % 2 == 0) {
      std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
      result = 0.5 * (result + values[mid - 1]);
    }
    return result;
  }

  // 从黄色 mask 的多条横向扫描带中提取左右边界、中心线和通道宽度。
  bool extractYellowLane(const cv::Mat &yellow_mask,
                         int width,
                         int height,
                         double &left_x,
                         double &right_x,
                         double &center_x,
                         double &lane_width) const {
    std::vector<double> lefts;
    std::vector<double> rights;
    std::vector<double> centers;
    std::vector<double> widths;

    const int band_height = std::max(3, scan_band_height_);
    const int min_width = std::max(5, lane_width_min_);
    for (const double row_ratio : scan_row_ratios_) {
      // row_ratio：扫描行在图像高度中的比例；band_height：上下扩展的统计带高度，单位像素。
      const int row = std::clamp(static_cast<int>(row_ratio * height), 0, height - 1);
      const int y0 = std::max(0, row - band_height / 2);
      const int y1 = std::min(height, y0 + band_height);
      const cv::Mat band = yellow_mask(cv::Rect(0, y0, width, y1 - y0));

      int best_left = -1;
      int best_right = -1;
      int best_width = 0;
      int run_left = -1;
      for (int x = 0; x < width; ++x) {
        // 每一列黄色像素占比超过 25% 时认为该列属于黄色通道。
        const double filled =
            static_cast<double>(cv::countNonZero(band.col(x))) /
            std::max(1, band.rows);
        const bool yellow_col = filled >= 0.25;
        if (yellow_col && run_left < 0) {
          run_left = x;
        }
        if ((!yellow_col || x == width - 1) && run_left >= 0) {
          const int run_right = yellow_col && x == width - 1 ? x : x - 1;
          const int run_width = run_right - run_left + 1;
          // 只保留宽度在合法范围内的最长连续黄色段，作为该扫描行的通道。
          if (run_width >= min_width && run_width > best_width) {
            best_left = run_left;
            best_right = run_right;
            best_width = run_width;
          }
          run_left = -1;
        }
      }

      if (best_left >= 0 && best_right > best_left &&
          best_width >= lane_width_min_ && best_width <= lane_width_max_) {
        lefts.push_back(static_cast<double>(best_left));
        rights.push_back(static_cast<double>(best_right));
        centers.push_back(0.5 * static_cast<double>(best_left + best_right));
        widths.push_back(static_cast<double>(best_width));
      }
    }

    // 没有任何扫描行识别到通道时，输出无效值给主控降级处理。
    if (centers.empty()) {
      left_x = -1.0;
      right_x = -1.0;
      center_x = -1.0;
      lane_width = 0.0;
      return false;
    }

    left_x = median(lefts);
    right_x = median(rights);
    center_x = median(centers);
    lane_width = median(widths);
    return true;
  }

  // 发布黄区车道综合信息：黄色通道、绿色禁区、黄白格子和近处比例。
  void publishLaneInfo(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                       const cv::Mat &bgr) {
    const int h = bgr.rows;
    const int w = bgr.cols;
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat yellow_mask;
    cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_),
                cv::Scalar(h_max_, 255, 255), yellow_mask);
    cv::Mat green_mask;
    cv::inRange(hsv, cv::Scalar(green_h_min_, green_s_min_, green_v_min_),
                cv::Scalar(green_h_max_, 255, 255), green_mask);
    cv::Mat white_mask;
    cv::inRange(hsv, cv::Scalar(white_h_min_, 0, white_v_min_),
                cv::Scalar(white_h_max_, white_s_max_, 255), white_mask);

    // 对黄/绿 mask 做形态学滤波，减少零散噪点对面积比例的影响。
    const int k = std::max(1, morph_kernel_ | 1);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(green_mask, green_mask, cv::MORPH_CLOSE, kernel);

    const int roi_y0 = std::clamp(static_cast<int>(roi_y_start_ratio_ * h), 0, h - 1);
    const int roi_y1 = std::clamp(static_cast<int>(roi_y_end_ratio_ * h), roi_y0 + 1, h);
    const cv::Rect roi(0, roi_y0, w, roi_y1 - roi_y0);
    const int near_y0 = std::clamp(static_cast<int>(near_roi_y_start_ratio_ * h), 0, h - 1);
    const cv::Rect near_roi(0, near_y0, w, h - near_y0);
    const cv::Rect near_left(0, near_y0, w / 2, h - near_y0);
    const cv::Rect near_right(w / 2, near_y0, w - w / 2, h - near_y0);
    const cv::Rect near_center(w / 4, near_y0, w / 2, h - near_y0);

    double left_x = -1.0;
    double right_x = -1.0;
    double lane_center_x = -1.0;
    double lane_width = 0.0;
    const bool lane_found =
        extractYellowLane(yellow_mask, w, h, left_x, right_x, lane_center_x, lane_width);
    const double yellow_area_ratio = ratioNonZero(yellow_mask, roi);
    const double green_area_ratio = ratioNonZero(green_mask, roi);
    const double green_left_ratio = ratioNonZero(green_mask, near_left);
    const double green_right_ratio = ratioNonZero(green_mask, near_right);
    const double green_center_ratio = ratioNonZero(green_mask, near_center);
    const double near_yellow_ratio = ratioNonZero(yellow_mask, near_roi);
    const double near_white_ratio = ratioNonZero(white_mask, near_roi);
    const double yellow_left_ratio = ratioNonZero(yellow_mask, near_left);
    const double yellow_right_ratio = ratioNonZero(yellow_mask, near_right);
    const double yellow_center_ratio = ratioNonZero(yellow_mask, near_center);
    const double white_left_ratio = ratioNonZero(white_mask, near_left);
    const double white_right_ratio = ratioNonZero(white_mask, near_right);
    const double center_white_ratio = ratioNonZero(white_mask, near_center);
    // 黄白格子禁入逻辑：同一区域同时有黄色和白色时，用二者较小值表示重叠强度。
    const double grid_left_ratio = std::min(yellow_left_ratio, white_left_ratio);
    const double grid_right_ratio = std::min(yellow_right_ratio, white_right_ratio);
    const double grid_center_ratio = std::min(yellow_center_ratio, center_white_ratio);
    const double grid_max_ratio =
        std::max({grid_left_ratio, grid_right_ratio, grid_center_ratio});
    const bool grid_detected =
        grid_max_ratio >= grid_forbidden_threshold_ ||
        (near_white_ratio >= grid_forbidden_threshold_ &&
         near_yellow_ratio >= yellow_min_area_ratio_ &&
         center_white_ratio >= grid_forbidden_threshold_ * 0.5);
    // yellow_visible/green_visible：给主控提供黄通道和绿色边界是否可用的二值信号。
    const bool yellow_visible = lane_found && yellow_area_ratio >= yellow_min_area_ratio_;
    const bool green_visible = green_area_ratio >= green_forbidden_threshold_;

    std_msgs::msg::Float64MultiArray out;
    // /yellow_lane_info 数据顺序固定，motion_controller 按索引读取，不能随意调整。
    // 新字段只能追加在末尾，避免破坏已有索引。
    out.data = {
        lane_center_x,
        left_x,
        right_x,
        lane_width,
        yellow_area_ratio,
        green_left_ratio,
        green_right_ratio,
        grid_detected ? 1.0 : 0.0,
        yellow_visible ? 1.0 : 0.0,
        green_visible ? 1.0 : 0.0,
        green_area_ratio,
        near_white_ratio,
        green_center_ratio,
        grid_left_ratio,
        grid_right_ratio,
        grid_center_ratio,
        grid_max_ratio,
        near_yellow_ratio,
        yellow_left_ratio,
        yellow_right_ratio};
    lane_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[YELLOW_LANE] left=%.1f right=%.1f center=%.1f width=%.1f yellow_ratio=%.3f green_left_ratio=%.3f green_right_ratio=%.3f green_center_ratio=%.3f grid_left=%.3f grid_right=%.3f grid_center=%.3f grid_detected=%d",
        left_x, right_x, lane_center_x, lane_width, yellow_area_ratio,
        green_left_ratio, green_right_ratio, green_center_ratio,
        grid_left_ratio, grid_right_ratio, grid_center_ratio, grid_detected);
  }

  // 图像回调：解码 ROS 图像，发布黄区车道信息，并分别发布黄/绿质心。
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    // 空图像直接丢弃，避免 OpenCV 访问非法数据。
    if (msg->width == 0 || msg->height == 0 || msg->data.empty()) {
      return;
    }

    cv::Mat bgr;
    // 支持 bgr8/rgb8 两种常见 USB 相机格式，其它格式不参与检测。
    if (msg->encoding == "bgr8") {
      bgr = cv::Mat(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
                    const_cast<uint8_t *>(msg->data.data()), msg->step);
    } else if (msg->encoding == "rgb8") {
      cv::Mat rgb(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
                  const_cast<uint8_t *>(msg->data.data()), msg->step);
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    } else {
      return;
    }

    publishLaneInfo(msg, bgr);

    double cx_raw = 0.0;
    double best_area = 0.0;
    // 黄色质心：用于黄通道入口判断和进黄区闭环对准。
    if (findColorCenter(bgr, roi_bottom_ratio_, h_min_, h_max_, s_min_, v_min_,
                        min_yellow_area_, min_fill_rate_, min_aspect_h_over_w_,
                        cx_raw, best_area)) {
      geometry_msgs::msg::PointStamped out;
      out.header.stamp = msg->header.stamp;
      out.header.frame_id = frame_id_;
      out.point.x = smoothValue(cx_raw, yellow_ema_initialized_, smooth_yellow_cx_);
      out.point.y = best_area;
      out.point.z = 0.0;
      pub_->publish(out);
    }

    double green_cx_raw = 0.0;
    double green_area = 0.0;
    // 绿色质心：用于黄区环道中保持与绿色禁区/边界的安全距离。
    if (findColorCenter(bgr, green_roi_bottom_ratio_, green_h_min_, green_h_max_,
                        green_s_min_, green_v_min_, min_green_area_,
                        green_min_fill_rate_, green_min_aspect_h_over_w_,
                        green_cx_raw, green_area)) {
      geometry_msgs::msg::PointStamped out;
      out.header.stamp = msg->header.stamp;
      out.header.frame_id = frame_id_;
      out.point.x = smoothValue(green_cx_raw, green_ema_initialized_, smooth_green_cx_);
      out.point.y = green_area;
      out.point.z = 0.0;
      green_pub_->publish(out);
    }
  }

  std::string sub_img_topic_;
  std::string frame_id_;
  double roi_bottom_ratio_{0.48};
  int h_min_{12};
  int h_max_{48};
  int s_min_{35};
  int v_min_{45};
  int min_yellow_area_{2500};
  int morph_kernel_{5};
  double center_score_k_{0.012};
  double cx_ema_alpha_{0.35};
  double min_fill_rate_{0.55};
  double min_aspect_h_over_w_{0.55};
  double green_roi_bottom_ratio_{1.0};
  int green_h_min_{35};
  int green_h_max_{85};
  int green_s_min_{50};
  int green_v_min_{50};
  int min_green_area_{2500};
  double green_min_fill_rate_{0.25};
  double green_min_aspect_h_over_w_{0.10};
  int white_h_min_{0};
  int white_h_max_{180};
  int white_s_max_{60};
  int white_v_min_{180};
  double roi_y_start_ratio_{0.55};
  double roi_y_end_ratio_{0.95};
  double near_roi_y_start_ratio_{0.75};
  double grid_near_roi_start_ratio_{0.60};
  std::vector<double> scan_row_ratios_{0.60, 0.70, 0.80, 0.90};
  int scan_band_height_{12};
  int lane_width_min_{60};
  int lane_width_max_{500};
  double yellow_min_area_ratio_{0.03};
  double green_forbidden_threshold_{0.08};
  double grid_forbidden_threshold_{0.08};
  bool yellow_ema_initialized_{false};
  bool green_ema_initialized_{false};
  double smooth_yellow_cx_{0.0};
  double smooth_green_cx_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr green_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace yellow_boundary

// 节点入口：初始化 ROS 2，运行黄色/绿色边界识别节点。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<yellow_boundary::YellowBoundaryNode>());
  rclcpp::shutdown();
  return 0;
}
