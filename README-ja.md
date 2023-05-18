# launch_exit_status

ROS 2 launchの終了ステータスを検証するリポジトリ

## 本リポジトリの目的

[driving_log_replayer](https://github.com/tier4/driving_log_replayer)でlaunchの終了ステータスが0(正常)にならないことがある。
異常終了するのはlaunchに[required node](https://ubuntu.com/blog/ros2-launch-required-nodes)が複数ある場合で、required nodeが1つの場合は0になる。

driving_log_replayerをクラウドで実行する[Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction)では、[wasim](https://docs.web.auto/developers-guides/wasim/introduction)がdriving_log_replayerを起動するRunnerとなっている。
wasimでは、driving_log_replayerのlaunchの終了ステータスを取って、シミュレーションの成否を判定する。
なので、ノードが評価終了を検知してlaunchのシャットダウンまで進んだのであれば終了ステータスは0である必要がある。

なので、launchの仕様の問題で、終了ステータスが1になるのか、autoware固有の問題なのか明らかにすることを目的とする。

## 結論と疑問点

以降の検証で明らかになったこと、事象としては確認出来たが、なぜそうなるのかわからない疑問点をまとめておく。

### 結論

1. required nodeを複数作っても終了ステータスは0で終わる
2. logging_simulator.launch.xmlをincludeしてもrequired nodeが1個なら終了ステータス0で終わる

### 疑問

1. logging_simulator.launch.xmlをlaunch.actions.Shutdown()で終了させると、planningがexceptionを吐く
2. logging_simulator.launch.xmlをincludeしてもrequired nodeが複数だと終了ステータスが1になる
3. logging_simulator.launch.xmlをノードより先に起動すると、ノードが起動せずにロックされる。driving_log_replayerでは何故か動いている
4. logging_simulator.launch.xmlをincludeすると、groupでscoped trueにしてもargumentが外に出てくる

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

よって、ROS 2の仕様上は、複数のrequired nodeを指定しても問題はない。(結論1)

### autowareのlaunchを使用する場合

[driving_log_replayer](https://github.com/tier4/driving_log_replayer)で、required nodeが一つだけなら正常終了することがわかっている。
複数個のreuqired nodeを設定した場合、異常になる原因が、[driving_log_replayer](https://github.com/tier4/driving_log_replayer)にあるノードの問題ではないことを確認するために、前述のtalkerとlistnerに入れ替えた場合でも起こることを確認する。

まず正常終了(0)になるであろうlaunchを作って、reuqiredをつけることで、挙動が変わること示す。
[single_required_with_aw.launch.py](./launch/single_required_with_aw.launch.py)を使用

#### コンテナのload_nodeで警告が出て例外を吐く

```bash
hyt@dpc1909014-2204:~/ros_ws/awf$ cd $HOME/ros_ws/awf
hyt@dpc1909014-2204:~/ros_ws/awf$ source install/setup.bash
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status single_required_with_aw.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit planning:=true
...
長いので省略 logs/single_required_with_aw.txt参照
...
hyt@dpc1909014-2204:~/ros_ws/awf$ echo $?
1
```

何故か異常終了している。
[ログ](./logs/single_required_with_aw.txt)を見ると、サービスのFutureでexceptionが出ている。
includeせずに直接`ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/map/sample sensor_model:=sample_sensor_kit vehicle_model:=sample_vehicle`した場合の[ログ](./logs/manually_launch_logging_simulator_launch.txt)を見ると、同じようにload_nodeの警告が出ているものの、Futureの例外は吐いていない（よくわからない）

```shell
[WARNING] [launch_ros.actions.load_composable_nodes]: Abandoning wait for the '/planning/scenario_planning/parking/parking_container/_container/load_node' service response, due to shutdown.
[INFO] [component_container_mt-69]: sending signal 'SIGINT' to process[component_container_mt-69]
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
[INFO] [routing_adaptor-68]: sending signal 'SIGINT' to process[routing_adaptor-68]
[INFO] [initial_pose_adaptor-67]: sending signal 'SIGINT' to process[initial_pose_adaptor-67]
[INFO] [web_server.py-66]: sending signal 'SIGINT' to process[web_server.py-66]
[INFO] [component_container_mt-65]: sending signal 'SIGINT' to process[component_container_mt-65]
[INFO] [component_container-64]: sending signal 'SIGINT' to process[component_container-64]
[INFO] [planning_evaluator-63]: sending signal 'SIGINT' to process[planning_evaluator-63]
[INFO] [planning_validator_node-62]: sending signal 'SIGINT' to process[planning_validator_node-62]
[INFO] [component_container-61]: sending signal 'SIGINT' to process[component_container-61]
[INFO] [component_container_mt-60]: sending signal 'SIGINT' to process[component_container_mt-60]
[INFO] [rtc_auto_mode_manager_node-59]: sending signal 'SIGINT' to process[rtc_auto_mode_manager_node-59]
[INFO] [routing_adaptor-68]: process has finished cleanly [pid 1441767]
[INFO] [component_container_mt-58]: sending signal 'SIGINT' to process[component_container_mt-58]
[INFO] [initial_pose_adaptor-67]: process has finished cleanly [pid 1441761]
[INFO] [motion_velocity_smoother-57]: sending signal 'SIGINT' to process[motion_velocity_smoother-57]
[INFO] [web_server.py-66]: process has finished cleanly [pid 1441724]
[INFO] [external_velocity_limit_selector-56]: sending signal 'SIGINT' to process[external_velocity_limit_selector-56]
[INFO] [planning_evaluator-63]: process has finished cleanly [pid 1441708]
[INFO] [scenario_selector-55]: sending signal 'SIGINT' to process[scenario_selector-55]
[INFO] [planning_validator_node-62]: process has finished cleanly [pid 1441706]
[INFO] [goal_pose_visualizer-54]: sending signal 'SIGINT' to process[goal_pose_visualizer-54]
[INFO] [mission_planner-53]: sending signal 'SIGINT' to process[mission_planner-53]
[INFO] [component_container-61]: process has finished cleanly [pid 1441677]
[INFO] [traffic_light_map_visualizer_node-52]: sending signal 'SIGINT' to process[traffic_light_map_visualizer_node-52]
[INFO] [component_container_mt-69]: process has finished cleanly [pid 1441785]
[INFO] [component_container_mt-51]: sending signal 'SIGINT' to process[component_container_mt-51]
[INFO] [traffic_light_map_based_detector_node-50]: sending signal 'SIGINT' to process[traffic_light_map_based_detector_node-50]
[INFO] [motion_velocity_smoother-57]: process has finished cleanly [pid 1441560]
[INFO] [map_based_prediction-49]: sending signal 'SIGINT' to process[map_based_prediction-49]
[WARNING] [launch_ros.actions.load_composable_nodes]: Abandoning wait for the '/perception/traffic_light_recognition/traffic_light_node_container/_container/load_node' service response, due to shutdown.
[WARNING] [launch_ros.actions.load_composable_nodes]: Abandoning wait for the '/perception/traffic_light_recognition/traffic_light_node_container/_container/load_node' service response, due to shutdown.
[INFO] [rtc_auto_mode_manager_node-59]: process has finished cleanly [pid 1441578]
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

perceptionに関しては、onnxの変換のログが大量に出ているので、engineファイルが生成できてないことによる問題と認識して、engineが出力されるまでしばらく放置した。
[ログ](./logs/generate_engine_file.txt.txt)を見る限り、engineの出力が完了したら、perceptionのload_nodeの警告はでなくなった。
planningについては問題は解決しなかった。

logging_simulator.launch.pyを直接起動して、Ctrl+Cで止める分には終了ステータスは0だが、includeして、launch.actions.Shutdown()で終了させると、planningがexceptionを吐く(疑問1)

#### planning offにして正常終了させる

planning起動しているとエラー吐くのでモジュールを呼ばないことで回避する

```bash
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status single_required_with_aw.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit planning:=false
...
長いので省略
logs/single_required_with_aw_planning_off参照
...
hyt@dpc1909014-2204:~/ros_ws/awf$ echo $?
0
```

planningがoffでreuqired nodeが1個なら正常終了になる(結論2)

#### reuqired nodeを複数にする

正常終了するsingle_required_with_aw.launch.pyのlistenerのon_exitのコメントを外してrequired nodeにしたlaunchを起動する

```bash
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status multi_required_with_aw.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit planning:=false
...
長いので省略
logs/multi_required_with_aw_planning_off参照
...
hyt@dpc1909014-2204:~/ros_ws/awf$ echo $?
1
```

違いは、reuqired nodeの数だけでstatusが1になる。
autowareを抜いたmulti_required.launch.pyではstatus 0なので、やはりautowareのlaunchを入れるか入れないかで挙動が違う(疑問2)

#### launch.LaunchDescriptionでlogging_simulator.launch.xmlのあとに渡す処理が呼ばれない

single_required_with_aw_not_working.launchを利用する。LaunchDescriptionの配列にlogging_simulator.launch.xmlをtalkerとlistenerよりも先に書くと、talkerとlistenerが呼ばれない。

```shell
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status single_required_with_aw_not_working.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit planning:=false rviz:=true
...
本来はtalkerが10回publishされたら終わるが、talkerが呼ばれないのでずっと終わらない
...
```

diriving_log_replyaerの[launch](https://github.com/tier4/driving_log_replayer/blob/develop/driving_log_replayer/launch/perception.launch.py#L72-L75)では、ノードより先に渡しているが動作している。

driving_log_replyaerではautowareが用意しているrvizの起動をoffにして独自のrvizを出すようにしているので、rvizをoffにしてみたが変わらない。

```shell
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status single_required_with_aw_not_working.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit planning:=false rviz:=false
...
rvizが起動されないだけでtalker,listenerが呼ばれないのは変わらない
...
```

driving_log_replayerでは問題なく動作しているのに、このリポジトリではLaunchDescriptionの配列にlogging_simulator.launch.xmlをtalkerとlistenerよりも先に書くと、talkerとlistenerが呼ばれない。(疑問3)

#### logging_simulator.launch.xmlのargumentが伝搬する

groupでくくってもlogging_simulator.launch.xmlをincludeするとargが出てきてしまうlogging_simulator.launch.xmlの方でscoped=falseを指定したら上でどうやっても伝搬を防げない？(疑問4)

```shell
~/ros_ws/awf main*
❯ ros2 launch launch_exit_status scope_sample.launch.xml -s
Arguments (pass arguments as '<name>:=<value>'):

    'map_path':
        point cloud and lanelet2 map directory path

    'vehicle_model':
        vehicle model name

    'sensor_model':
        sensor model name

    'vehicle_id':
        vehicle specific ID
        (default: EnvVar('VEHICLE_ID'))

    'vehicle':
        launch vehicle
        (default: 'true')
...
長いので省略
```
