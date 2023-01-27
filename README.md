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
colcon build --packages-select cart
```

cartパッケージの実行に必要な環境変数を読み込みます。
```sh
. install/setup.bash 
```

cartパッケージを実行します。
```sh
ros2 launch launch/cart_launch.py headless:=False
```
