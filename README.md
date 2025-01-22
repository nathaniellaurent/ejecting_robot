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
ros2 run ejecting_robot camera_publisher --ros-args -p camera_index:=1 -p rstp_url:=http://192.168.83.6:81/stream
```

To run for both cameras (IP addresses stored in launch file):

```bash
ros2 launch ejecting_robot ejecting_robot.launch
```

To run serial agent:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

To start wifi:

```bash
sudo create_ap -n wlp0s20f3 robotWifi ce53432524
```

To run wifi agent:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
To run tf2 world:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world
```
To run rviz world:

```bash
ros2 run rviz2 rviz2
```



Dependencies

```bash
sudo apt install libeigen3-dev
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```