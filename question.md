# 疑問点

groupでくくってもlogging_simulator.launch.xmlをincludeするとargが出てきてしまうlogging_simulator.launch.xmlの方でscoped=falseを指定したら上でどうやっても伝搬を防げない？

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
