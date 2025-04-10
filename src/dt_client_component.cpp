#include "misora2_dt_client/dt_client_component.hpp"

namespace dt_client_component
{
DTClient::DTClient(const rclcpp::NodeOptions &options)
    : Node("dt_client",options)
{
    // P1,P2,P3,P4,P6モード
    this->declare_parameter("mode", "P6");
    std::string param = this->get_parameter("mode").as_string();

    receive_data_ = this->create_subscription<std_msgs::msg::String>("result_data",10,
        [this](const std_msgs::msg::String::SharedPtr msg){
            result_data = msg->data;
        });
    receive_id_ = this->create_subscription<std_msgs::msg::String>("id",10,
        [this](const std_msgs::msg::String::SharedPtr msg){
            id = msg->data;
        });
    receive_image_ = this->create_subscription<MyAdaptedType>("result_image",10,
        [this](const std::unique_ptr<cv::Mat> msg){
            result_image = std::move(*msg);
            RCLCPP_INFO_STREAM(this->get_logger(),"Receive address: " << &(msg->data));
        });
    
}

}//namespace dt_client_component


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dt_client_component::DTClient)