import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64, Bool

import cv2 
from cv_bridge import CvBridge, CvBridgeError

import uuid
import csv
import os
import requests
import json
from datetime import datetime

from misora2_custom_msg.msg import Digital

def image_resize(image, width=640):# MISORAの方にこの処理はいらない
    h, w = image.shape[:2]
    height = round(h * (width / w))
    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
    return image

def save_local(qr, value, img, robot_id, mission, result_folder='save_folder/', logger=print):
    mission_folder = os.path.join(result_folder, mission)
    csv_file = os.path.join(mission_folder, 'data.csv')
    image_directory = os.path.join(mission_folder, 'image')

    # result_folderが存在しない場合は自動作成
    if not os.path.exists(mission_folder):
        os.makedirs(mission_folder, exist_ok=True)
        logger(f"{mission_folder} を作成しました。")

    # CSVファイルが存在しない場合は作成し、ヘッダーを書き込み
    if not os.path.exists(csv_file):
        with open(csv_file, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'QR', 'Value',
                            'Image Filename', 'Robot ID'])
        logger(f"{csv_file} を作成し、ヘッダーを追加しました。")

    # imageディレクトリが存在しない場合は作成
    if not os.path.exists(image_directory):
        os.makedirs(image_directory, exist_ok=True)
        logger(f"{image_directory} ディレクトリを作成しました。")

    # タイムスタンプの生成（ISO 8601形式）
    timestamp = datetime.now().isoformat()

    # ディレクトリ内の画像ファイル数をカウントして次のファイル名を決定
    existing_images = [f for f in os.listdir(
                        image_directory) if f.endswith('.jpg')]
    row_count = len(existing_images) + 1  # 現在の画像数に1を足して次の番号を決定

    # 画像ファイル名の作成
    image_filename = f'result_{row_count}.jpg'
    image_path = os.path.join(image_directory, image_filename)
    logger(f"{image_directory}に画像ファイル{image_filename}を生成しました。")

    # 画像ファイルの保存
    cv2.imwrite(image_path, img)

    # CSVファイルへの追記モードで書き込み
    with open(csv_file, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            timestamp,
            qr,
            value,
            image_filename,
            robot_id
        ])


class ClientNodeValue(Node):
    def __init__(self):
        super().__init__('client_node_value')

        #通信クライアントの初期化
        self.declare_parameter('host', '')
        self.declare_parameter('robot_id', '')
        self.declare_parameter('mac_id', '%012x'%uuid.getnode())
        self.declare_parameter('mission', '')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.mac_id = self.get_parameter('mac_id').get_parameter_value().string_value
        self.mission = self.get_parameter('mission').get_parameter_value().string_value
        self.get_logger().info(f"MAC ID : {self.mac_id}")

        self.client_ready = True  # ← 初期化
        # try:
        #     self.client_ready = self._initialize_client(self.host, self.robot_id, self.mac_id, self.get_logger().info)
        # except Exception as e:
        #     self.get_logger().error(str(e))

        # subscriber作成
        # 報告内容が更新されるたびに送信用変数を更新する
        self.receive_data_ = self.create_subscription(Digital, 'digital_data', 
                                                self.receive_data_callback, 10)

        # 500msごとにtimer_callbackを呼ぶ
        # self.timer = self.create_timer(0.5, self.send_to_dt_callback)
        # self.result_data_ = self.create_subscription(String, 'result_data', 
        #                                                 self.receive_data_callback, 10)
        # # self.result_pressure_ = self.create_subscription(Float64, 'result_data_p', self.receive_data_p_callback, 10)
        # self.result_image_ = self.create_subscription(Image, 'result_image', 
        #                                                 self.receive_image_callback, 10)

        # # 送信するという信号を受け取る
        # self.send_trigger_ = self.create_subscription(Bool, 'send_trigger', self.send_to_dt_callback, 10)

        # sensor_msgs/Image->cv_imageの変換で使用
        self.bridge = CvBridge()
        # 使用する変数の初期化
        self.result_image = None  # 画像を空にする（Noneに設定）
        self.qr_id = ""           # QR IDを空にリセット
        self.result_data = ""     # 結果データを空にリセット


    def _initialize_client(self, host, robot_id, mac_id, logger=print): # 配布されたrobot_registration_example.pyに該当
        values = {"rob_id": robot_id, "mac_id": mac_id}
        url = 'https://'+host+'/WRS2025/api/set_mac_id.php' # to register the robot mac ID

        self.session = requests.Session() # sesstionが終わるようにメソッド変数として持たせる
        response = self.session.post(url, json=values)
        if response.status_code != requests.codes.ok:
            raise Exception(f"Request Status Error: {response.status_code}")
            return False

        if response.json()['status'] == 'success':
            logger("Start Session")
            logger(json.dumps(response.json(), indent=4))
            return True
        else:
            raise Exception(f"Request Error: {response.json()}")
            return False

    
    def request(self, id, value, img, logger=print):
        logger(id, value)
        
        url_report_values = 'https://'+self.host+'/WRS2025/api/notify_entity_status.php' # to send equipment value & image 
        # 画像処理後の報告-----------------------------------------------
        values = {
            "eqpt_nm": id,
            "value": value,
            "mac_id": self.mac_id
        }
        files = {'image': ("result.jpg", img)}

        response = requests.post(url_report_values, files=files, data=values)
        if response.status_code != requests.codes.ok:
            raise Exception(
                f"Request Status Error: {response.status_code}")
        if json.loads(response.text)['status'] == 'success':
            logger('post success')
        else:
            raise Exception(f"Request Error: {response.text}")
        # -------------------------------------------------------------
        
    def receive_data_callback(self, msg):
        self.result_data = msg.result
        self.qr_id = msg.id
        self.result_image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")

        send_to_dt_callback(self)

    def send_to_dt_callback(self):
        # 条件を満たしていれば送信処理を行う
        if (self.result_image is not None and self.result_image.size > 0 and self.qr_id != "" and self.result_data != ""):

            # jpegに変換(サイズ変更が必要かも)
            before_converted_image = image_resize(self.result_image.copy())
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 100]

            result, encoded_image = cv2.imencode(
                '.jpg', before_converted_image, encode_param)

            if not result:
                self.get_logger().error('Could not encode image!')

            image_prepared = encoded_image.tobytes()

            # ローカルへの保存
            save_local(self.qr_id, str(self.result_data), self.result_image,
                    self.robot_id, self.mission, logger=self.get_logger().info)
            
            # 送信
            if self.client_ready != False:
                try:
                    self.request(self.qr_id, str(self.result_data), image_prepared)
                except Exception as e:
                    self.get_logger().error("Failed sending to RMS: " + str(e))
            else:
                self.get_logger().error("No connection to RMS")

            # 送信後の初期化
            self.result_image = None  # 画像を空にする（Noneに設定）
            self.qr_id = ""           # QR IDを空にリセット
            self.result_data = ""     # 結果データを空にリセット

        else:
            self.get_logger().warn("Skipping send process: missing data")
# str(self.result_data)