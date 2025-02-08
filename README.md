## Diable Interface

### Login
- user accout: diablo
- password: diablo123

### Usage

#### Bringup
```bash=
$ roscore
$ rosrun diablo_sdk movement_ctrl_example # core interface
```
#### Basic Motion
- stand/sit
```bash
$ rostopic pub -1 /cmd_stand std_msgs/Bool "data: true" # stand: true; sit: false 
```

#### navigation
```bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py  # keyboard
```