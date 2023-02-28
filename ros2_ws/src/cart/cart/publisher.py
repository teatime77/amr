# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import time
import math
import numpy as np
import socket

import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# サーバーIPとポート番号
IPADDR = "192.168.0.120"
PORT = 80


def little_int(v, i):
    return v[i] + 256 * v[i+1]

def find_head(data):
    for i in range(len(data)):
        if data[i] == 0xAA and data[i+1] == 0x55:
            return i

    return -1

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 1000)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.start_time = time.time()
        self.prev_time = self.start_time


        # ソケット作成
        self.sock = socket.socket(socket.AF_INET)


        # サーバーへ接続
        print("connecting")
        self.sock.connect((IPADDR, PORT))

        print("connected")

        # byte 形式でデータ送信
        # self.sock.send("hello\n".encode("utf-8"))
        # print("send hello")

        self.sock.send("\n".encode("utf-8"))
        print("send empty")

        self.data = bytes()
        self.ranges_list = []
        self.FSA, self.LSA, self.ranges = None, None, None

    def timer_callback(self):

        bs = self.sock.recv(1024)
        if bs == b'':
            print("end")
            return

        data = self.data + bs

        while 10 <= len(data):

            if not (data[0] == 0xAA and data[1] == 0x55):
                print('find')
                idx = find_head(data)
                if idx == -1:
                    break

                data = data[idx:]

                assert data[0] == 0xAA and data[1] == 0x55
            
                if len(data) < 10:
                    print(len(data))
                    break


            CT  = int(data[2]) & 0x01
            LSN = int(data[3])

            if len(data) < 10 + 2 * LSN:
                break

            FSA = little_int(data, 4)
            LSA = little_int(data, 6)
            CS  = little_int(data, 8)

            assert CS == int(data[8]) + int(data[9]) * 256

            ranges = [0.0] * LSN
            cs = 0x55AA ^ little_int(data, 2) ^ FSA ^ LSA
            for i in range(LSN):
                r = little_int(data, 10 + 2 * i)

                # X2 DEVELOPMENT MANUAL: Distance solution formula:
                #   https://www.ydlidar.com/Public/upload/files/2022-06-21/YDLIDAR%20X2%20Development%20Manual%20V1.2(211228).pdf
                ranges[i] = 0.001 * float(r) / 4.0

                cs ^= r

            # X2 DEVELOPMENT MANUAL: Starting angle solution formula & End angle solution formula
            FSA = (FSA >> 1) / 64.0
            LSA = (LSA >> 1) / 64.0
            # print()
            # self.get_logger().info(f'head {hex(data[2])} {LSN} angle:{round(FSA)} - {round(LSA)} CS:{CS - cs}')
            assert CS == cs

            data = data[10 + 2 * LSN:]

            if cs != CS or LSA <= FSA:
                continue

            else :
                if self.FSA is None:
                    # 最初の場合

                    self.FSA = FSA

                elif FSA < self.LSA:
                    # 角度が減った場合

                    if len(self.ranges_list) != 0:
                        all_ranges = [x for l in self.ranges_list for x in l]
                        self.publish_scan(self.FSA, self.LSA, all_ranges)

                        self.ranges_list = []

                    self.FSA = FSA

                self.LSA = LSA
                self.ranges_list.append(ranges)

        self.data = data

        return

    def publish_scan(self, FSA, LSA, ranges):

        msg = LaserScan()

        LSN = len(ranges)
        # header:
        #   stamp:
        #     sec: 0
        #     nanosec: 0

        current_time = time.time()
        t = current_time - self.start_time
        msg.header.stamp.sec = math.floor(t)
        msg.header.stamp.nanosec = int(1000 * 1000 * 1000 * (t - msg.header.stamp.sec))
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = FSA * math.pi / 180.0
        msg.angle_max = LSA * math.pi / 180.0
        msg.angle_increment = (msg.angle_max - msg.angle_min) / LSN

        msg.scan_time   = current_time - self.prev_time
        msg.time_increment = msg.scan_time / LSN
        msg.range_min   = 0.0
        msg.range_max   = 10.0
        # base = (self.i % 30) / 30.0
        # msg.ranges = [ 5.0 + math.sin(2.0 * math.pi * (base + i / float(LSN))  ) for i in range(LSN) ]
        msg.ranges = ranges
        msg.intensities = []


        self.publisher_.publish(msg)

        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.get_logger().info('Publishing: "%d"' % self.i)
        self.get_logger().info(f'LSN:{LSN} angle:{round(FSA)} - {round(LSA)} LSN:{LSN}')

        self.prev_time = current_time
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
