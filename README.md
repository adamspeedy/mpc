# MPC Controller

## Using CSV-file saving
To subscribe to an odom topic and record the values to a csv file, adjust the topic names and file paths in the record.py file and then use the following command:
```
ros2 run nmpc_pkg record
```
To use this recorded odometry with the default controller adjust the relevant topics and file paths in the controller_node.py file.
```
ros2 run nmpc_pkg nmpc_controller_node
```

## launching:
**Still working on this bit**, but if we want to use the /set_goal and /path options, we can use:
```
ros2 launch nmpc_pkg nmpc_controller.launch.py use_navigan:=true
```