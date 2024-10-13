# ejecting robot
To scan ports to find IP addresses, use:

```bash
nmap -sP 192.168.*.62
```

To build:

```bash
 source install/setup.bash && colcon build
```

To run for one camera:

```bash
ros2 run ejecting_robot camera_publisher --ros-args -p camera_index:=1 rstp_url:="http://192.168.83.6:81/stream" 
```