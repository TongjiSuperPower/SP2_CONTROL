# SP2_CONTROL

## Features

- 基于ros2_control开发
- 支持Ignition-Fortress(humble)仿真环境
- SP_CONTROL在ROS2上的重写，使用逻辑相近
- 更易使用的底层设备接口

## Ignition仿真环境配置

**NOTE:** 如果使用虚拟机，请关闭设置中的3D加速功能，否则可能引起渲染错误。

1. [安装Ignition-Fortress(Humble)版本](https://gazebosim.org/docs/fortress/install_ubuntu)

   ```
   sudo apt-get update
   sudo apt-get install lsb-release wget gnupg
   sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
   sudo apt-get update
   sudo apt-get install ignition-fortress
   ```

2. [安装ign_ros2_control插件包](https://github.com/ros-controls/gz_ros2_control/tree/humble)

   ```
   sudo apt-get update
   sudo apt-get install ros-humble-ign-ros2-control
   ```

3. [安装ros_gz包](https://github.com/gazebosim/ros_gz/tree/humble)

   ```
   sudo apt-get update
   sudo apt install ros-humble-ros-gz
   ```

## Ignition仿真环境测试

1. 运行命令打开仿真环境，等待模型及控制器加载完成

   ```
   ros2 launch sp2_description robot_ignition_sim.launch.py
   ```

2. 发布运动控制话题`effort_controllers/cmd_vel_unstamped`

   ```
   ros2 topic pub effort_controllers/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

