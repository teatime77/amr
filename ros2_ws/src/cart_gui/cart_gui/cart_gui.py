import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32

import sys
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QSlider, QSpinBox, QLabel, QFormLayout
from PyQt5.QtGui import QIcon
import matplotlib.pyplot as plt 
 
encoderR_data = []
encoderL_data = []

wheel_radius = 0.3

class MainWindow(QWidget):
    def __init__(self, node, parent=None):
        super(MainWindow, self).__init__(parent) # 初期化

        self.resize(400, 300) # ウィンドウの大きさの設定(横幅, 縦幅)
        self.move(400, 300) # ウィンドウを表示する場所の設定(横, 縦)
        self.setWindowTitle('PyQt5 sample GUI') # ウィンドウのタイトルの設定

        self.pwmVel = QSlider(Qt.Orientation.Horizontal)
        self.pwmVel.setRange(-15, 15)
        self.pwmVel.setValue(0)
        self.pwmVel.resize(300, 20)
        self.pwmVel.valueChanged.connect(node.cartVelChanged)

        self.pwmDir = QSlider(Qt.Orientation.Horizontal)
        self.pwmDir.setRange(-255, 255)
        self.pwmDir.setValue(0)
        self.pwmDir.resize(300, 20)

        self.btn = QPushButton('Hello World PyQt5', self) # ボタンウィジェット作成

        velR = QSpinBox(minimum=1, maximum=100, value=20)
        velL = QSpinBox(minimum=1, maximum=100, value=20)

        layout = QFormLayout()

        layout.addRow(QLabel("PWM Vel"), self.pwmVel)
        layout.addRow(QLabel("PWM Dir"), self.pwmDir)


        layout.addRow(self.btn)
        layout.addRow(QLabel("速度 R"), velR)
        layout.addRow(QLabel("速度 L"), velL)

        self.setLayout(layout)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.cartVel_pub = self.create_publisher(Float32, 'cartVel', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.encoderR_sub = self.create_subscription(Float32, '/encoder_R', self.encoderR_callback, 10)
        self.encoderL_sub = self.create_subscription(Float32, '/encoder_L', self.encoderL_callback, 10)

        self.encoderR_sub
        self.encoderL_sub

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

    def cartVelChanged(self):
        cart_vel = 0.01 * main_window.pwmVel.value()

        msg = Float32()
        msg.data = cart_vel
        self.cartVel_pub.publish(msg)        

    def encoderR_callback(self, msg):
        encoderR_data.append(msg.data)

        self.draw_lines(self.lines1, encoderR_data)

    def encoderL_callback(self, msg):
        encoderL_data.append(msg.data)

        self.draw_lines(self.lines2, encoderL_data)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        pass



    def timer_callback(self):
        pass

    def pushed_button1(self):
        pass

def main(args=None):
    global main_window

    rclpy.init(args=args)

    app = QApplication(sys.argv) #PyQtで必ず呼び出す必要のあるオブジェクト

    minimal_publisher = MinimalPublisher()

    main_window = MainWindow(minimal_publisher) #ウィンドウクラスのオブジェクト生成

    main_window.btn.clicked.connect(minimal_publisher.pushed_button1)

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

