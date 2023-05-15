# launch_exit_status

ROS 2 launchの終了ステータスを検証するリポジトリ

## 本リポジトリの目的

[driving_log_replayer](https://github.com/tier4/driving_log_replayer)でlaunchの終了ステータスが0(正常)にならないことがある。
異常終了するのはlaunchに[required node](https://ubuntu.com/blog/ros2-launch-required-nodes)が複数ある場合で、required nodeが1つの場合は0になる。

launchの仕様上、複数設定すると異常終了になるのか、autoware固有の問題なのか明らかにすることを目的とする。

## セットアップ手順

autoware foundationのautowareにこのリポジトリを一緒に入れてビルドする

```shell
mkdir -p $HOME/ros_ws
cd $HOME/ros_ws
git clone https://github.com/autowarefoundation/autoware.git awf
cd awf
mkdir -p src/simulator
cd src/simulator
git clone https://github.com/hayato-m126/launch_exit_status.git
cd $HOME/ros_ws/awf
vcs import src < autoware.repos
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO
./setup-dev-env.sh
colcon build --symlink-install --catkin-skip-building-tests --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
```

## 終了ステータスの確認

まず、複数のrequired nodeを使用すると終了ステータスが1になるのが、ROS 2の仕様なのか、autowareの問題なのかを切り分ける

### autowareのlaunchを使用しない場合

demo_node_pyのtalkerとlistenerをrequired指定した[launch](./launch/multi_required.launch.py)を利用する。

10回publishしたら終わる。
talkerが終わったあとに、shutting down launched systemが走って、listnerが終了して、もう一度shutting down launched systemが走っている。
終了ステータスは0になっている。

```bash
hyt@dpc1909014-2204:~/ros_ws/awf$ cd $HOME/ros_ws/awf
hyt@dpc1909014-2204:~/ros_ws/awf$ source install/setup.bash
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status multi_shutdown.launch.py
[INFO] [launch]: All log files can be found below /home/hyt/.ros/log/2023-05-12-14-05-14-378302-dpc1909014-2204-160739
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [talker_qos-1]: process started with pid [160740]
[INFO] [listener_qos-2]: process started with pid [160742]
[listener_qos-2] 1683867914.619783 [77] listener_q: determined enp4s0 (udp/10.0.53.59) as highest quality interface, selected for automatic interface.
[talker_qos-1] 1683867914.620260 [77] talker_qos: determined enp4s0 (udp/10.0.53.59) as highest quality interface, selected for automatic interface.
[talker_qos-1] [INFO] [1683867914.652606364] [talker]: Best effort talker
[listener_qos-2] [INFO] [1683867914.652905745] [listener]: Best effort listener
[talker_qos-1] [INFO] [1683867915.654654440] [talker]: Publishing: "Hello World: 0"
[listener_qos-2] [INFO] [1683867915.655384473] [listener]: I heard: [Hello World: 0]
[talker_qos-1] [INFO] [1683867916.655796884] [talker]: Publishing: "Hello World: 1"
[listener_qos-2] [INFO] [1683867916.657873365] [listener]: I heard: [Hello World: 1]
[talker_qos-1] [INFO] [1683867917.655823132] [talker]: Publishing: "Hello World: 2"
[listener_qos-2] [INFO] [1683867917.657938686] [listener]: I heard: [Hello World: 2]
[talker_qos-1] [INFO] [1683867918.655828157] [talker]: Publishing: "Hello World: 3"
[listener_qos-2] [INFO] [1683867918.657929749] [listener]: I heard: [Hello World: 3]
[talker_qos-1] [INFO] [1683867919.655338138] [talker]: Publishing: "Hello World: 4"
[listener_qos-2] [INFO] [1683867919.656955253] [listener]: I heard: [Hello World: 4]
[talker_qos-1] [INFO] [1683867920.655481521] [talker]: Publishing: "Hello World: 5"
[listener_qos-2] [INFO] [1683867920.657447015] [listener]: I heard: [Hello World: 5]
[talker_qos-1] [INFO] [1683867921.654423789] [talker]: Publishing: "Hello World: 6"
[listener_qos-2] [INFO] [1683867921.654931959] [listener]: I heard: [Hello World: 6]
[talker_qos-1] [INFO] [1683867922.655791421] [talker]: Publishing: "Hello World: 7"
[listener_qos-2] [INFO] [1683867922.657880723] [listener]: I heard: [Hello World: 7]
[talker_qos-1] [INFO] [1683867923.655933516] [talker]: Publishing: "Hello World: 8"
[listener_qos-2] [INFO] [1683867923.658173490] [listener]: I heard: [Hello World: 8]
[talker_qos-1] [INFO] [1683867924.655817502] [talker]: Publishing: "Hello World: 9"
[listener_qos-2] [INFO] [1683867924.657927637] [listener]: I heard: [Hello World: 9]
[INFO] [talker_qos-1]: process has finished cleanly [pid 160740]
[INFO] [launch]: process[talker_qos-1] was required: shutting down launched system
[INFO] [listener_qos-2]: sending signal 'SIGINT' to process[listener_qos-2]
[INFO] [listener_qos-2]: process has finished cleanly [pid 160742]
[INFO] [launch]: process[listener_qos-2] was required: shutting down launched system
hyt@dpc1909014-2204:~/ros_ws/awf$ echo $?
0
```

よって、ROS 2の仕様上は、複数のrequired nodeを指定しても問題はなく、autowareのlaunchと組み合わせると何故か問題が起こることがわかる。

### autowareのlaunchを使用する場合

[driving_log_replayer](https://github.com/tier4/driving_log_replayer)で、required nodeが一つだけなら正常終了することがわかっている。
複数個のreuqired nodeを設定した場合、異常になる原因が、[driving_log_replayer](https://github.com/tier4/driving_log_replayer)にあるノードの問題ではないことを確認するために、前述のtalkerとlistnerに入れ替えた場合でも起こることを確認する。

[multi_shutdown_with_autoware.launch.py](./launch/multi_shutdown_with_autoware.launch.py)を使用

1. composable nodeのload_nodeが殺されてWARINGが出ている。多分これが原因で終了ステータス1になっていそう
2. perceptionに関しては、engineファイルが生成されていればこの問題は発生しない模様


#### サービス待ち

```bash
hyt@dpc1909014-2204:~/ros_ws/awf$ cd $HOME/ros_ws/awf
hyt@dpc1909014-2204:~/ros_ws/awf$ source install/setup.bash
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status multi_shutdown_with_autoware.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
 ros2 launch launch_exit_status multi_shutdown_with_autoware.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
...
長いので省略 logs/multi_shutdown_with_loggging_simualtor_launch.txt参照
...
hyt@dpc1909014-2204:~/ros_ws/awf$ echo $?
1
```

怪しくみえるのは以下の部分、サービスを待っているが、shutdownによって殺されている。
`ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/map/sample sensor_model:=sample_sensor_kit vehicle_model:=sample_vehicle`を叩いてCtrl+Cで終了したときの[ログ](logs/manually_launch_logging_simulator_launch.txt)と比べると、Ctrl+Cのときにはこの部分出てない

```shell
[INFO] [talker_qos-1]: process has finished cleanly [pid 167516]
[INFO] [launch]: process[talker_qos-1] was required: shutting down launched system
[INFO] [rviz2-70]: sending signal 'SIGINT' to process[rviz2-70]
[INFO] [component_container_mt-69]: sending signal 'SIGINT' to process[component_container_mt-69]
[INFO] [routing_adaptor-68]: sending signal 'SIGINT' to process[routing_adaptor-68]
[INFO] [initial_pose_adaptor-67]: sending signal 'SIGINT' to process[initial_pose_adaptor-67]
[INFO] [web_server.py-66]: sending signal 'SIGINT' to process[web_server.py-66]
[INFO] [component_container_mt-65]: sending signal 'SIGINT' to process[component_container_mt-65]
[WARNING] [launch_ros.actions.load_composable_nodes]: Abandoning wait for the '/perception/traffic_light_recognition/traffic_light_node_container/_container/load_node' service response, due to shutdown.
[WARNING] [launch_ros.actions.load_composable_nodes]: Abandoning wait for the '/perception/traffic_light_recognition/traffic_light_node_container/_container/load_node' service response, due to shutdown.
[WARNING] [launch_ros.actions.load_composable_nodes]: Abandoning wait for the '/perception/traffic_light_recognition/traffic_light_node_container/_container/load_node' service response, due to shutdown.
Future exception was never retrieved
future: <Future finished exception=InvalidHandle('cannot use Destroyable because destruction was requested')>
Traceback (most recent call last):
  File "/usr/lib/python3.10/concurrent/futures/thread.py", line 58, in run
    result = self.fn(*self.args, **self.kwargs)
  File "/opt/ros/humble/lib/python3.10/site-packages/launch_ros/actions/load_composable_nodes.py", line 197, in _load_in_sequence
    self._load_node(next_load_node_request, context)
  File "/opt/ros/humble/lib/python3.10/site-packages/launch_ros/actions/load_composable_nodes.py", line 119, in _load_node
    while not self.__rclpy_load_node_client.wait_for_service(timeout_sec=1.0):
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/client.py", line 180, in wait_for_service
    return self.service_is_ready()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/client.py", line 159, in service_is_ready
    with self.handle:
rclpy._rclpy_pybind11.InvalidHandle: cannot use Destroyable because destruction was requested
[INFO] [component_container-64]: sending signal 'SIGINT' to process[component_container-64]
[INFO] [planning_evaluator-63]: sending signal 'SIGINT' to process[planning_evaluator-63]
[INFO] [planning_validator_node-62]: sending signal 'SIGINT' to process[planning_validator_node-62]
[INFO] [component_container-61]: sending signal 'SIGINT' to process[component_container-61]
[INFO] [component_container_mt-60]: sending signal 'SIGINT' to process[component_container_mt-60]
[INFO] [rtc_auto_mode_manager_node-59]: sending signal 'SIGINT' to process[rtc_auto_mode_manager_node-59]
[WARNING] [launch_ros.actions.load_composable_nodes]: Abandoning wait for the '/planning/scenario_planning/parking/parking_container/_container/load_node' service response, due to shutdown.
Future exception was never retrieved
future: <Future finished exception=InvalidHandle('cannot use Destroyable because destruction was requested')>
Traceback (most recent call last):
  File "/usr/lib/python3.10/concurrent/futures/thread.py", line 58, in run
    result = self.fn(*self.args, **self.kwargs)
  File "/opt/ros/humble/lib/python3.10/site-packages/launch_ros/actions/load_composable_nodes.py", line 197, in _load_in_sequence
    self._load_node(next_load_node_request, context)
  File "/opt/ros/humble/lib/python3.10/site-packages/launch_ros/actions/load_composable_nodes.py", line 119, in _load_node
    while not self.__rclpy_load_node_client.wait_for_service(timeout_sec=1.0):
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/client.py", line 180, in wait_for_service
    return self.service_is_ready()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/client.py", line 159, in service_is_ready
    with self.handle:
rclpy._rclpy_pybind11.InvalidHandle: cannot use Destroyable because destruction was requested
```

サービスの待機は、ros2 bag playをして、topicを流さないと進まないように見えるので、perceptionとplanningをfalseにして動かないようにして起動する。

```bash
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status multi_shutdown_with_autoware.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit planning:=false
...
長いので省略
logs/mulit_shutdown_planning_perception_off参照
...
hyt@dpc1909014-2204:~/ros_ws/awf$ echo $?
1
```

モジュールをoffにしたにも関わらずstatusが1になる。
[driving_log_replayer](https://github.com/tier4/driving_log_replayer)において、複数のnodeのrequiredを複数入れると何故かexit status 1になることがわかっているので、listenerのノードからon_exitを取り除いて、on_exitを一つにしたlaunchを使用する。

```bash
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status single_shutdown_with_autoware.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
 ros2 launch launch_exit_status multi_shutdown_with_autoware.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit perception:=false planning:=false
...
長いので省略
logs/single_shutdown_planning_perception_off参照
...
hyt@dpc1909014-2204:~/ros_ws/awf$ echo $?
0
```

## わからないこと

- multi_shutdown.launch.pyで2個on_exitを設定してもlogging_simulator.launch.xmlを呼ばなければexit statusは0であることが確認できる
- 何故かlogging_simulator.launch.xmlと一緒に動かすと複数個のon_exitでexit statusが1になってしまう。

## 困ってること

[diving_log_replayer](https://github.com/tier4/driving_log_replayer)では、評価が終了したときにノードがshutdownして、launchが終了する仕組みを使用している。

また、driving_log_replayerをクラウドで実行する[Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction)では、[wasim](https://docs.web.auto/developers-guides/wasim/introduction)がdriving_log_replayerを起動するRunnerとなっている。
wasimでは、driving_log_replayerのlaunchの終了ステータスを取って、成否を判定しているので、ノードが異常終了した場合以外は、終了ステータスは0になってくれないと、wasimの結果が間違って出てしまう。
