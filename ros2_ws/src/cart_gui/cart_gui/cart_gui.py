import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
from PyQt5.QtGui import QIcon
 
class MainWindow(QWidget):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent) # 初期化
        self.initUI() # UIの初期化
    def initUI(self): # UIの初期化をするメソッド
        self.resize(400, 300) # ウィンドウの大きさの設定(横幅, 縦幅)
        self.move(400, 300) # ウィンドウを表示する場所の設定(横, 縦)
        self.setWindowTitle('PyQt5 sample GUI') # ウィンドウのタイトルの設定
        #self.setWindowIcon(QIcon('xxxx.jpg')) # ウィンドウ右上のアイコンの設定
        btn = QPushButton('Hello World PyQt5', self) # ボタンウィジェット作成
        btn.resize(btn.sizeHint()) # ボタンのサイズの自動設定
        btn.move(100, 50) # ボタンの位置設定(ボタンの左上の座標)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)



    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

