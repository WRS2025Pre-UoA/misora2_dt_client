#ifndef DT_CLIENT_COMPONENT_HPP
#define DT_CLIENT_COMPONENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <thread>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;
namespace fs = std::filesystem;

#include "misora2_dt_client/cv_mat_type_adapter.hpp"
#include "misora2_dt_client/gui_tool.hpp"

namespace dt_client_component
{
class DTClient : public rclcpp::Node
{
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;
    public:
    std::string result_data, qr_id;
    cv::Mat result_image, qr_image;
    std::map<std::string, bool> flag_list = {{"qr",false},{"other",false}};// 何が届いたか
    bool should_close_send = false;
    bool should_close_back = false;
    bool send_dt = false;

    // std_msgs::msg::Bool msg_B;
    explicit DTClient(const rclcpp::NodeOptions &options);
    DTClient() : DTClient(rclcpp::NodeOptions{}) {}

    private:
    void open_window();// 確認画面を表示
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr receive_qr_id_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr receive_data_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr receive_data_p_;
    rclcpp::Subscription<MyAdaptedType>::SharedPtr receive_image_;
    rclcpp::Subscription<MyAdaptedType>::SharedPtr receive_qr_image_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr receive_flag_;// 画面立ち上げの信号
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr send_flag_;
    // 送信用ノードに起動信号を渡す
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr send_publisher_;
};
} // namespace dt_client_component

#endif // DT_CLIENT_COMPONENT_HPP