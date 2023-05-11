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

終了ステータスが0(正常に終わる)

```bash
cd $HOME/ros_ws/awf
source install/setup.bash
ros2 launch launch_exit_status multi_shutdown.launch.py
echo $?
```

終了ステータスが1(異常)

```bash
cd $HOME/ros_ws/awf
source install/setup.bash
ros2 launch launch_exit_status multi_shutdown_with_autoware.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
echo $?
```

## 困ってること

何故か、autowareと組み合わせると終了スタータスが1になってしまう。

[driving_log_replayer](https://github.com/tier4/driving_log_replayer)では、評価が終了したときにノードがshutdownして、launchが終了する仕組みを使用している。

また、driving_log_replayerをクラウドで実行する[Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction)では、[wasim](https://docs.web.auto/developers-guides/wasim/introduction)がdriving_log_replayerを起動するRunnerとなっている。
wasimでは、driving_log_replayerのlaunchの終了ステータスを取って、成否を判定しているので、ノードが異常終了した場合以外は、終了ステータスは0になってくれないと、wasimの結果が間違って出てしまう。
