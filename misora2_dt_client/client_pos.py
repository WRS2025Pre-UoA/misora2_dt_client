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

from misora2_custom_msg.msg import Pos

class ClientNodePos(Node):
    def __init__(self):
        super().__init__('client_node_pos')

        #通信クライアントの初期化
        self.declare_parameter('host', '')
        self.declare_parameter('robot_id', '')
        self.declare_parameter('mac_id', '%012x'%uuid.getnode())
        self.declare_parameter('mission', '')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.mac_id = self.get_parameter('mac_id').get_parameter_value().string_value
        self.mission = self.get_parameter('mission').get_parameter_value().string_value
        self.client_ready = True  # ← 初期化
        # try:
        #     self.client_ready = self._initialize_client(self.host, self.robot_id, self.mac_id, self.get_logger().info)
        # except Exception as e:
        #     self.get_logger().error(str(e))

        # subscriber作成
        self.pos_sub = self.create_subscription(
            Pos,                # ← カスタムメッセージ型
            'pos_data',        # ← トピック名
            self.pos_callback,  # ← コールバック関数
            10                  # ← キューサイズ
        )
        self.session = requests.Session()
        
    def _initialize_client(self, host, robot_id, mac_id, logger=print): # 配布されたrobot_registration_example.pyに該当
        values = {"rob_id": robot_id, "mac_id": mac_id}
        url = 'https://'+host+'/WRS2025/api/set_mac_id.php' # to register the robot mac ID
        # logger(robot_id)
        # logger(mac_id)
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

    
    def pos_callback(self, msg):
        self.request_pos(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw)

    def request_pos(self, x, y, z, roll, pitch, yaw):
        url_report_pos = 'https://' + self.host + '/WRS2025/api/notify_rob_pos.php'
        # 小数3桁に丸め
        x = round(x, 3)
        y = round(y, 3)
        z = round(z, 3)
        roll = round(roll, 3)
        pitch = round(pitch, 3)
        yaw = round(yaw, 3)
        self.get_logger().info(
            f"x: {x:.3f}, y: {y:.3f}, z: {z:.3f}, roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {yaw:.3f}"
        )
        if self.client_ready:
            values = {
                "mac_id": self.mac_id,
                "rob_pos": { "x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw }
            }
            try:
                response = self.session.post(url_report_pos, json=values)
                if response.status_code != requests.codes.ok:
                    self.get_logger().error(f"Request Status Error: {response.status_code}")
                elif response.json().get('status') == 'success':
                    self.get_logger().info('Position post success')
                else:
                    self.get_logger().error(f"Request Error: {response.text}")
            except Exception as e:
                self.get_logger().error("Failed sending to RMS: " + str(e))
        else:
            self.get_logger().error("No connection to RMS")