# amr
AMR - autonomous mobile robot


ソースをGitHubからクローンします。
```sh
git clone https://github.com/teatime77/amr.git
```


### dockerのビルドと実行

dockerをビルドします。
```sh
cd amr/docker
docker build -t fortress .
```

dockerを実行します。
```sh
docker run -v ~/prj/amr:/amr -it --rm --name fortress-c fortress /bin/bash
```

### docker内で実行

```sh
cd /amr/ros2_ws/src
ros2 pkg create --build-type ament_cmake cpp_cart
```


```sh
cd /amr/ros2_ws
```

ROS2の依存関係をチェックします。
```sh
rosdep install -i --from-path src --rosdistro humble -y
```

cartパッケージをビルドします。
```sh
colcon build --symlink-install --packages-select cpp_cart
```

cartパッケージの実行に必要な環境変数を読み込みます。
```sh
. install/setup.bash 
```

cartパッケージを実行します。
```sh
ros2 launch launch/cpp_cart_launch.py headless:=False
```

## Arduino IDE

[ツール]-[ライブラリーを管理] を開く。
**Adafruit MPU6050** で検索します。

**Adafruit MPU6050 by Adafruit** をインストールします。

「ライブラリの依存関係をインストール」のダイアログが表示されるので、「全てをインストール」をクリックします。

## NAV2デモ

[Getting Started — Navigation 2 1.0.0 documentation](https://navigation.ros.org/getting_started/index.html#running-the-example)

1. launchする。

```sh
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

2. rviz2の **2D Pose Estimate** をクリック。

3. ドラッグして、初期位置と方向を指定する。<br/>
ここでLidarが表示されなければ、 1.からやり直す。

4. **Nav2 to Goal** をクリック。

5. ドラッグして、ゴールの位置と方向を指示する。<br/>
ナビゲーションが開始する。

---

## cpp_cart + rviz2


1. launchする。

```sh
ros2 launch launch/cpp_cart_launch.py
```

2. ESP32との無線接続を確認する。<br/>
接続エラーの場合は、 1.からやり直す。

3.  rvizのFixed Frameをbase_linkにする。

4. Addボタンをクリックし、By topicタブでLaserScanを追加する。

5. LaserScanのTopicのReliability Policyで Best Effortを選択する。<br/>
LaserScanが表示されるのを確認する。

## ros2_control_demos

[ros-controls/ros2_control_demos: This repository aims at providing examples to illustrate ros2_control and ros2_controllers](https://github.com/ros-controls/ros2_control_demos)


1. launchする。

```sh
ros2 launch ros2_control_demo_bringup diffbot.launch.py
```

2. 速度のコマンドを送る。

```sh
ros2 topic pub --rate 30 /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
 x: 0.7
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.0"
```