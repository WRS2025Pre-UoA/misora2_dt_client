#!/usr/bin/env python3 

import rclpy
from misora2_dt_client.client import ClientNode

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()