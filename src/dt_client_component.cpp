#include "misora2_dt_client/dt_client_component.hpp"

namespace dt_client_component
{
DTClient::DTClient(const rclcpp::NodeOptions &options)
    : Node("dt_client",options)
{
    // P1,P2,P3,P4,P6モード
    this->declare_parameter("mode", "P6");
    std::string param = this->get_parameter("mode").as_string();

    // create_save_folder(param);// 報告する内容をすべて格納するフォルダ作成

    receive_data_ = this->create_subscription<std_msgs::msg::String>("result_data",1,
        [this](const std_msgs::msg::String::SharedPtr msg){
            result_data = msg->data;
            flag_list["other"] = true;
        });
    receive_qr_id_ = this->create_subscription<std_msgs::msg::String>("qr_id",1,
        [this](const std_msgs::msg::String::SharedPtr msg){
            qr_id = msg->data;
            flag_list["qr"] = true;
        });
    receive_image_ = this->create_subscription<MyAdaptedType>("result_image",10,
        [this](const cv::Mat msg){
            result_image = msg;
            // flag_list["other"] = true;
            RCLCPP_INFO_STREAM(this->get_logger(),"Receive address: " << &(msg) << ", Size: " << result_image.size() << ", channels: " << result_image.channels());
            RCLCPP_INFO_STREAM(this->get_logger(),"flag other: " << flag_list["other"]);
        });
    receive_qr_image_ = this->create_subscription<MyAdaptedType>("result_qr_image",10,
        [this](const cv::Mat msg){
            qr_image = msg;
            // flag_list["qr"] = true;
            RCLCPP_INFO_STREAM(this->get_logger(),"Receive address: " << &(msg) << ", Size: " << qr_image.size() << ", channels: " << qr_image.channels());
            RCLCPP_INFO_STREAM(this->get_logger(),"flag qr: " << flag_list["qr"]);
        });
    receive_flag_ = this->create_subscription<std_msgs::msg::Bool>("startUp",1,
        [this](const std_msgs::msg::Bool::SharedPtr msg){
            // RCLCPP_INFO_STREAM(this->get_logger(),"receive: " << msg->data);
            if(msg->data){
                // RCLCPP_INFO_STREAM(this->get_logger(),"OPEN");
                std::thread([this]() {
                    open_window();
                }).detach();
            }
            else if(!msg->data){
                // RCLCPP_INFO_STREAM(this->get_logger(),"CLOSE");
                should_close_back = true;  // open_windowのループを止める
                cv::destroyAllWindows();
            }

        });

    // デジタルツインへ送信したか否かのflagをguiへ送信publisher
    send_flag_ = this->create_publisher<std_msgs::msg::Bool>("send_flag",1);
    send_publisher_ = this->create_publisher<std_msgs::msg::Bool>("send_trigger",1);
}

void DTClient::open_window(){
    should_close_back = false;
    should_close_send = false;

    DrawTool window(1280,720,0);// 黒画像作成
    Button back(cv::Point(50,650),cv::Size(150,50));
    Button send(cv::Point(1080,650),cv::Size(150,50));
    window.drawButton_new(back, "back", cv::Scalar(255,255,255), -1, cv::LINE_AA, cv::Scalar(0,0,0), 2);
    window.drawButton_new(send, "send", cv::Scalar(255,255,255), -1, cv::LINE_AA, cv::Scalar(0,0,0), 2);

    // 受け取った画像-------------------------
    // image1の枠
    cv::Point pos1 = cv::Point(40,40);
    cv::Size size1 = cv::Size(500,375);
    cv::Mat image_to_show1;
    std::string text_to_show1;

    if (flag_list["qr"] and not(qr_image.empty())) {
        RCLCPP_INFO_STREAM(this->get_logger(),"plot qr_image");
        image_to_show1 = qr_image;
        text_to_show1 = qr_id;
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(),"plot null_image1");
        DrawTool white(640, 480, cv::Scalar(255,255,255));
        white.drawText("null image1", cv::Point(250,208), 1, 0, 2);
        image_to_show1 = white.getImage();
        text_to_show1 = "null data1";
    }

    window.drawImage(image_to_show1, pos1, size1);
    Button text_box1(cv::Point(pos1.x, pos1.y + size1.height), cv::Size(size1.width, 100));
    window.drawButton_new(text_box1, text_to_show1, 0, -1, cv::LINE_AA, cv::Scalar(255,255,255), 2);
    // image2の枠
    cv::Point pos2 = cv::Point(740,40);
    cv::Size size2 = cv::Size(500,375);
    cv::Mat image_to_show2;
    std::string text_to_show2;

    if (flag_list["other"] and not(result_image.empty())) {
        RCLCPP_INFO_STREAM(this->get_logger(),"plot result_image");
        image_to_show2 = result_image;
        text_to_show2 = result_data;
        // RCLCPP_INFO_STREAM(this->get_logger(),"Size: " << image_to_show2.size());
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(),"plot null_image2");
        DrawTool white(640, 480, cv::Scalar(255,255,255));
        white.drawText("null image2", cv::Point(250,208), 1, 0, 2);
        image_to_show2 = white.getImage();
        text_to_show2 = "null data2";
    }

    window.drawImage(image_to_show2, pos2, size2);
    RCLCPP_INFO_STREAM(this->get_logger(),"setting result_image or null_image");
    Button text_box2(cv::Point(pos2.x, pos2.y + size2.height), cv::Size(size2.width, 100));
    window.drawButton_new(text_box2, text_to_show2, 0, -1, cv::LINE_AA, cv::Scalar(255,255,255), 2);
    // --------------------------------------
    cv::Mat confirm_window = window.getImage();

    // ボタンの範囲
    cv::Rect back_rect(back.pos, back.size);
    cv::Rect send_rect(send.pos, send.size);

    // ウィンドウ表示
    cv::namedWindow("window", cv::WINDOW_AUTOSIZE);
    cv::imshow("window", confirm_window);

    // マウスイベント処理用データ構造
    struct CallbackData {
        cv::Rect* backBtn;
        cv::Rect* sendBtn;
        rclcpp::Logger logger;
        dt_client_component::DTClient* instance;
    };
 
    CallbackData cbData = {&back_rect, &send_rect, this->get_logger(), this};
    
    cv::setMouseCallback("window", [](int event, int x, int y,int /*flags*/, void* userdata) {
        CallbackData* data = static_cast<CallbackData*>(userdata);
        std_msgs::msg::Bool msg_B;
        auto* instance = data->instance;  // ← thisの代わり
        if (event == cv::EVENT_LBUTTONDOWN) {
            cv::Point point(x, y);
            if (data->backBtn->contains(point)) {
                // RCLCPP_INFO_STREAM(data->logger, "Back button clicked.");

                msg_B.data = false;

                instance->send_flag_->publish(msg_B);
                instance->should_close_back = true;  // ウィンドウを閉じるフラグ
                
            } else if (data->sendBtn->contains(point)) {
                // RCLCPP_INFO_STREAM(data->logger, "Send button clicked.");
                if(instance->flag_list["qr"] == true and instance->flag_list["other"] == true){
                    instance->send_dt = true;

                    msg_B.data = true;
                    instance->send_flag_->publish(msg_B);
                    // 
                    // ここに送信処理を書く
                    // 関数 指定したディレクトリにsaveする
                    // 関数 デジタルツインへ報告する
                    instance->send_publisher_->publish(msg_B);
                    // 初期化
                    instance->flag_list["qr"] = false;
                    instance->flag_list["other"] = false;
                    instance->qr_id = "";
                    instance->result_data = "";
                    // instance->pressure_result_data = 0.0;
                    instance->qr_image.release();
                    instance->result_image.release();
                    instance->should_close_send = true;  // ウィンドウを閉じるフラグ
                }
            }
        }
    }, &cbData);

    // イベントループ（マウスイベントを処理し続ける）
    while (true) {
        cv::imshow("window", confirm_window);

        cv::waitKey(30);
        if (should_close_back or should_close_send){
            DrawTool window(1280,720,0);// 黒画像作成
            if(should_close_back)window.drawButton(back, "back", cv::Scalar(0,0,255), -1, cv::LINE_AA, 1, cv::Scalar(25,255,255));
            if(should_close_send)window.drawButton(send, "send", cv::Scalar(0,0,255), -1, cv::LINE_AA, 1, cv::Scalar(25,255,255));
            cv::Mat confirm_window = window.getImage();
            cv::imshow("window", confirm_window);
            cv::waitKey(50);
            cv::destroyAllWindows();
            break;
        }

    }
}

}//namespace dt_client_component

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dt_client_component::DTClient)