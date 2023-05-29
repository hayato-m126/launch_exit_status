# launch_exit_status

This repository verifies the exit status of ROS 2 launch.

Japanese documentation is available [here](./README-ja.md).

## Purpose of this repository

The exit status of a launch may not be 0 (normal) in [driving_log_replayer](https://github.com/tier4/driving_log_replayer).
The abnormal termination occurs when there are multiple [required nodes](https://ubuntu.com/blog/ros2-launch-required-nodes) in the launch, and the status becomes 0 when there is only one required node.

In [Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction), which runs driving_log_replayer in the cloud, [wasim](https:// docs.web.auto/developers-guides/wasim/introduction) is the Runner that starts driving_log_replayer.
In wasim, the success or failure of the simulation is determined by taking the exit status of the driving_log_replayer launch.
So if the node has detected the end of the evaluation and proceeded to the launch shutdown, the exit status should be 0.

Therefore, the purpose is to clarify whether the exit status becomes 1 due to a problem in the launch specification or whether it is an autoware-specific problem.

## Solution

The following questions remain, but for now, a workaround is to use [ShutdownOnce](https://github.com/tier4/scenario_simulator_v2/blob/ master/test_runner/scenario_test_runner/scenario_test_runner/shutdown_once.py), a modification of launch.actions.Shutdown.

## Conclusions and questions

This section summarises what has been clarified in the subsequent verification and the questions that have been confirmed in the event

### Conclusions

1. even if multiple required nodes are created, the exit status will be 0
2. even if logging_simulator.launch.xml is included, if there is only one required node, the exit status will be 0

### Questions

1. when logging_simulator.launch.xml is terminated by launch.actions.Shutdown(), planning throws exception
2. even if logging_simulator.launch.xml is included, exit status is 1 if there are multiple required nodes.
3. if logging_simulator.launch.xml is started before the node, the node is locked without starting. driving_log_replayer is working for some reason.
4. when logging_simulator.launch.xml is included, the argument comes out even if scoped true in group.

## Setup instructions

Put this repository together in autoware foundation's autoware and build the workspace.

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

## Check exit status

First, judge whether the exit status of 1 when using multiple required nodes is a ROS 2 specification or an autoware problem.

### Without autoware launch

Use the [launch](./launch/multi_required.launch.py) file that loads the talker and listener of demo_node_py specified as required.

After 10 publishes, the TALKER node shuts down.
After the shutdown of the TALKER, a SHUTTING DOWN LAUNCHED SYSTEM is running, the LISTNER is terminated and a SHUTTING DOWN LAUNCHED SYSTEM is running again.
The exit status is 0, as shown in the console log below.

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

Therefore, according to the ROS 2 specification, there is no problem in specifying multiple required nodes. (Conclusion 1)

### When using autoware launch

It is known that [driving_log_replayer](https://github.com/tier4/driving_log_replayer) will terminate normally if there is only one reuqired node.
To check that the anomaly is not caused by a problem with the node in driving_log_replayer when multiple reuqired nodes are set up, check that this also happens when the nodes are replaced with the aforementioned talker and listener.
First create a launch that will result in a normal termination (0). Afterwards, it is shown that it changes to an abnormal end (1) by adding the reuqired node.

Use [single_required_with_aw.launch.py](./launch/single_required_with_aw.launch.py) to check.

#### Container's load_node will raise a warning and throw an exception

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

For some reason it is terminating abnormally.
[The log](./logs/s/single_required_with_aw.txt) shows an exception in the Future of the service.
When calling logging_simulator.launch.xml directly, [the log](./logs/manually_launch_logging_simulator_launch.txt) shows the same load_node warning, but no Future exception is thrown.

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

As for PERCEPTION, there was a lot of logging of the conversion of ONNX, so I recognised the problem as being due to the engine file not being generated, and left it for a while until the engine was output.
[The log](./logs/generate_engine_file.txt.txt) shows that once the output of the engine was complete, the warning about the load_node in the perception stopped.
The problem was not solved for PLANNING.

The exit status is 0 for directly invoking logging_simulator.launch.py and stopping it with Ctrl+C.
However, if I include it in another launch and terminate it with launch.actions.Shutdown(), planning throws an exception (question 1).

#### Planning off and terminate normally

Avoid errors by not calling the planning module.

```bash
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status single_required_with_aw.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit planning:=false
...
Omitted because of its length
refer logs/single_required_with_aw_planning_off
...
hyt@dpc1909014-2204:~/ros_ws/awf$ echo $?
0
```

If planning is off and there is one reuqired node, it will end normally (Conclusion 2).

#### make multiple reuqired node

Uncomment on_exit of listener in single_required_with_aw.launch.py that exits successfully and start launch with required node

```bash
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status multi_required_with_aw.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit planning:=false
...
Omitted because of its length
refer logs/multi_required_with_aw_planning_off
...
hyt@dpc1909014-2204:~/ros_ws/awf$ echo $?
1
```

Just by setting the listener node to the required node, the status becomes 1.
In multi_required.launch.py without autoware, the status is 0, so the behavior is still different depending on whether autoware launch is included or not (question 2).

#### The process passed after logging_simulator.launch.xml in launch.LaunchDescription is not called

Use single_required_with_aw_not_working.launch.
If logging_simulator.launch.xml is written in the LaunchDescription array before the talker and listener, the talker and listener are not called.

```shell
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status single_required_with_aw_not_working.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit planning:=false rviz:=true
...
Originally, it ends when the talker has been PUBLISHED 10 times, but it doesn't end all the time because the talker is not called.
...
```

[The launch file in driving_log_replayer](https://github.com/tier4/driving_log_replayer/blob/develop/driving_log_replayer/launch/perception.launch.py#L72-L75) is working, although `logging_simulator.launch.xml` is passed before the node.

In driving_log_replyaer, the rviz provided by autoware is turned off to produce its own rviz, so I tried turning off rviz, but it did not change.

```shell
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch launch_exit_status single_required_with_aw_not_working.launch.py map_path:=$HOME/map/sample vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit planning:=false rviz:=false
...
Just because rviz is not activated does not change the fact that talker,listener is not called.
...
```

If I write logging_simulator.launch.xml in the LaunchDescription array before the talker and listener in this repository, even though it works fine in driving_log_replayer, the talker and listener are not called. (Question 3)

#### logging_simulator.launch.xml argument propagates

If I include logging_simulator.launch.xml even if I wrap it in groups, I still get argument.
If I specify scoped=false in the logging_simulator.launch.xml, how can I prevent propagation above? (Question 4)

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
Omitted because of its length
```
