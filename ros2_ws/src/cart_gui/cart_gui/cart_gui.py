import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32

import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QSpinBox, QLabel, QFormLayout
from PyQt5.QtGui import QIcon
import matplotlib.pyplot as plt 
 
encoderR_data = []
encoderL_data = []

wheel_radius = 0.3

class MainWindow(QWidget):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent) # 初期化

        self.resize(400, 300) # ウィンドウの大きさの設定(横幅, 縦幅)
        self.move(400, 300) # ウィンドウを表示する場所の設定(横, 縦)
        self.setWindowTitle('PyQt5 sample GUI') # ウィンドウのタイトルの設定
        btn = QPushButton('Hello World PyQt5', self) # ボタンウィジェット作成

        velR = QSpinBox(minimum=1, maximum=100, value=20)
        velL = QSpinBox(minimum=1, maximum=100, value=20)

        layout = QFormLayout()

        layout.addRow(btn)
        layout.addRow(QLabel("速度 R"), velR)
        layout.addRow(QLabel("速度 L"), velL)

        self.setLayout(layout)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.encoderR_sub = self.create_subscription(Float32, '/encoder_R', self.encoderR_callback, 10)
        self.encoderL_sub = self.create_subscription(Float32, '/encoder_L', self.encoderL_callback, 10)


        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        self.encoderR_sub
        self.encoderL_sub
        self.subscription  # prevent unused variable warning

        self.fig, self.ax = plt.subplots(1, 1)

        x = np.arange(-np.pi, np.pi, 0.1)
        y = np.sin(x)        
        self.lines1, = self.ax.plot([0, 1], [0, 1])
        self.lines2, = self.ax.plot([0, 1], [0, 1])

        self.min_y = 0
        self.max_y = 0

    def draw_lines(self, lines, data):
        x = list(range(len(data)))
        y = np.array(data)

        self.min_y = min(self.min_y, y.min() * 1.1)
        self.max_y = max(self.max_y, y.max() * 1.1)

        self.ax.set_xlim((0, len(data)))
        self.ax.set_ylim((self.min_y, self.max_y))

        lines.set_data(x, y)

    def encoderR_callback(self, msg):
        self.get_logger().info('enc R: "%.1f"' % msg.data)
        encoderR_data.append(msg.data)

        self.draw_lines(self.lines1, encoderR_data)

    def encoderL_callback(self, msg):
        self.get_logger().info('enc L: "%.1f"' % msg.data)
        encoderL_data.append(msg.data)

        self.draw_lines(self.lines2, encoderL_data)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        pass



    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    app = QApplication(sys.argv) #PyQtで必ず呼び出す必要のあるオブジェクト
    main_window = MainWindow() #ウィンドウクラスのオブジェクト生成
    main_window.show() #ウィンドウの表示

    while True:
        rclpy.spin_once(minimal_publisher)
        app.processEvents()

        plt.pause(.01)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

