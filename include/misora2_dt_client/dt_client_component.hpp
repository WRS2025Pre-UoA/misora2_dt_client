#ifndef DT_CLIENT_COMPONENT_HPP
#define DT_CLIENT_COMPONENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;

#include "misora2_dt_client/cv_mat_type_adapter.hpp"

namespace dt_client_component
{
class DTClient : public rclcpp::Node
{
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;
    public:
    std::string result_data, id;
    cv::Mat result_image;

    explicit DTClient(const rclcpp::NodeOptions &options);
    DTClient() : DTClient(rclcpp::NodeOptions{}) {}

    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr receive_id_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr receive_data_;
    rclcpp::Subscription<MyAdaptedType>::SharedPtr receive_image_;
};
} // namespace dt_client_component

#endif // DT_CLIENT_COMPONENT_HPP