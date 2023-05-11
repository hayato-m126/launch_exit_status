# launch_exit_status

ROS 2 launchの終了ステータスを検証するリポジトリ

## 参考リンク

https://ubuntu.com/blog/ros2-launch-required-nodes

## セットアップ手順

autoware foundationのautowareにこのリポジトリを一緒に入れてビルドする

```shell
mkdir -p $HOME/ros_ws
cd $HOME/ros_ws
git clone https://github.com/autowarefoundation/autoware.git awf
cd awf
mkdir -p src/simulator
cd src/simulator
git clone 
vcs import src < autoware.repos
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO
./setup-dev-env.sh
colcon build --symlink-install --catkin-skip-building-tests --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
```

## 動かし方
