#!/usr/bin/env python3 

import rclpy
from misora2_dt_client.client_value import ClientNodeValue

def main(args=None):
    rclpy.init(args=args)
    node = ClientNodeValue()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()