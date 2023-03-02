import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import sys
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QSlider, QSpinBox, QLabel, QFormLayout
from PyQt5.QtGui import QIcon
import matplotlib.pyplot as plt 
 
encoderR_data = []
encoderL_data = []
acc_x = []
acc_y = []

vel_x = []
vel_y = []

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
        self.pwmDir.setRange(-100, 100)
        self.pwmDir.setValue(0)
        self.pwmDir.resize(300, 20)
        self.pwmDir.valueChanged.connect(node.cartDirChanged)

        self.btn = QPushButton('Stop', self)
        self.btn.clicked.connect(node.stopClicked)

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
        self.cartDir_pub = self.create_publisher(Float32, 'cartDir', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.encoderR_sub = self.create_subscription(Float32, '/encoder_R', self.encoderR_callback, 10)
        self.encoderL_sub = self.create_subscription(Float32, '/encoder_L', self.encoderL_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/demo/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/demo/odom', self.odom_callback, 10)

        self.encoderR_sub
        self.encoderL_sub

        self.fig = plt.figure()

        self.ax1 = self.fig.add_subplot(2, 1, 1)
        self.ax1.set_ylim((0, 0))

        x = np.arange(-np.pi, np.pi, 0.1)
        y = np.sin(x)        
        self.lines_Vr, = self.ax1.plot([0, 1], [0, 1], label='Vr')
        self.lines_Vl, = self.ax1.plot([0, 1], [0, 1], label='Vl')
        self.lines_Vx, = self.ax1.plot([0, 1], [0, 1], label='Vx')
        self.lines_Vy, = self.ax1.plot([0, 1], [0, 1], label='Vy')

        plt.legend()

        self.ax2 = self.fig.add_subplot(2, 1, 2)

        self.ax2.set_ylim((0, 0))


        self.imu_lines_acc_x, = self.ax2.plot([0, 1], [0, 1], label='acc x')
        self.imu_lines_acc_y, = self.ax2.plot([0, 1], [0, 1], label='acc y')

        self.acc_x_offset = 0
        self.acc_y_offset = 0

        plt.legend()

    def draw_lines(self, ax, lines, data):
        x = list(range(len(data)))
        y = np.array(data)

        (min_y, max_y) = ax.get_ylim()

        min_y = min(min_y, y.min() * 1.1)
        max_y = max(max_y, y.max() * 1.1)

        ax.set_xlim((0, len(data)))
        ax.set_ylim((min_y, max_y))

        lines.set_data(x, y)

    def cartVelChanged(self):
        cart_vel = 0.01 * main_window.pwmVel.value()

        msg = Float32()
        msg.data = cart_vel
        self.cartVel_pub.publish(msg)        

    def cartDirChanged(self):
        cart_dir = 0.001 * main_window.pwmDir.value()

        msg = Float32()
        msg.data = cart_dir
        self.cartDir_pub.publish(msg)        

    def stopClicked(self):
        main_window.pwmVel.setValue(0)
        main_window.pwmDir.setValue(0)

    def encoderR_callback(self, msg):
        encoderR_data.append(msg.data)

        self.draw_lines(self.ax1, self.lines_Vr, encoderR_data)

    def encoderL_callback(self, msg):
        encoderL_data.append(msg.data)

        self.draw_lines(self.ax1, self.lines_Vl, encoderL_data)

    def imu_callback(self, msg):
        global acc_x, acc_y

        if len(acc_x) == 50:
            self.acc_x_offset = sum(acc_x) / len(acc_x)
            self.acc_y_offset = sum(acc_y) / len(acc_y)

            acc_x = [x - self.acc_x_offset for x in acc_x]
            acc_y = [y - self.acc_y_offset for y in acc_y]

            self.ax2.set_ylim((0, 0))

        acc_x.append(msg.linear_acceleration.x - self.acc_x_offset)
        acc_y.append(msg.linear_acceleration.y - self.acc_y_offset)

        self.draw_lines(self.ax2, self.imu_lines_acc_x, acc_x)
        self.draw_lines(self.ax2, self.imu_lines_acc_y, acc_y)

    def odom_callback(self, msg):
        vel_x.append(msg.twist.twist.linear.x)
        vel_y.append(msg.twist.twist.linear.y)

        # self.get_logger().info('odom: %.2f %.2f' % (msg.twist.twist.linear.x, msg.twist.twist.linear.y))

        self.draw_lines(self.ax1, self.lines_Vx, vel_x)
        self.draw_lines(self.ax1, self.lines_Vy, vel_y)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        pass



    def timer_callback(self):
        pass

def main(args=None):
    global main_window

    rclpy.init(args=args)

    app = QApplication(sys.argv) #PyQtで必ず呼び出す必要のあるオブジェクト

    minimal_publisher = MinimalPublisher()

    main_window = MainWindow(minimal_publisher) #ウィンドウクラスのオブジェクト生成

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

