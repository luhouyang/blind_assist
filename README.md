# ROBOCUP

## Install Ros2 Humble (Ubuntu 22.04 LTS)

[**Ros2  Humble**](https://docs.ros.org/en/humble/Installation.html)
```
locale

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

```
source /opt/ros/humble/setup.bash
ros2 topic list
```

Add `source /opt/ros/humble/setup.bash` to ~/.bashrc file to source ros2 automatically on all new terminals

## Add Groups

```
sudo usermod -aG video $USER
sudo usermod -aG audio $USER
sudo usermod -a -G dialout $USER
```

Restart to take effect

## Install Dependencies

**Ros2 Packages**
```
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt-get install ros-humble-rviz2 ros-humble-tf2-ros ros-humble-tf2-tools
```

[**RealSense SDK**](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
```
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

sudo apt-get install librealsense2-dkms

sudo apt-get install librealsense2-utils

sudo apt-get install librealsense2-dev

sudo apt-get install librealsense2-dbg
```

Reconnect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.

[**RealSense Ros2 Wrapper**](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file)
```
sudo apt install ros-humble-realsense2-*
```

[**YDLidar SDK**](https://github.com/YDLIDAR/YDLidar-SDK)
1. Run steps in terminal (home dir) (Ubuntu):

    ```
    sudo apt install cmake pkg-config
    sudo apt-get install python swig
    sudo apt-get install python-pip
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    cd YDLidar-SDK
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```

[**YDLidar ROS2 Wrapper**](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble)
1. New terminal (home dir):
    ```
    git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
    cd ydlidar_ros2_driver
    colcon build --symlink-install
    ```

1. Edit ~/.bashrc file:
    ```
    source ~/ydlidar_ros2_driver/install/setup.bash
    ```

1. Configure YDLidar parameters. Navigate to `~/ydlidar_ros2_driver/params/ydlidar.yaml` refer to [DATASET](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/Dataset.md), for `X2/X2L` parameters are as below.
    ```
    ydlidar_ros2_driver_node:
      ros__parameters:
        port: /dev/serial/by-path/pci-0000:00:0c.0-usb-0:2:1.0-port0
        frame_id: laser_frame
        ignore_array: ""
        baudrate: 115200
        lidar_type: 1
        device_type: 0
        sample_rate: 3
        intensity_bit: 0
        abnormal_check_count: 4
        fixed_resolution: true
        reversion: true
        inverted: true
        auto_reconnect: true
        isSingleChannel: true
        intensity: false
        support_motor_dtr: false
        angle_max: 180.0
        angle_min: -180.0
        range_max: 64.0
        range_min: 0.01
        frequency: 10.0
        invalid_range_is_inf: false
    ```


## `~/.bashrc` file

```
#
# PREDEFINED CONFIGURATIONS
#

source /opt/ros/humble/setup.bash
source ~/ydlidar_ros2_driver/install/setup.bash
# path to ydlidar setup file assuming package is build from source, with project root dir at HOME
```

## Build & Run

1. Resolve Dependencies
    ```
    rosdep install -i --from-path src --rosdistro humble -y
    ```

1. Run colcon build in workspace root
    ```
    colcon build --symlink-install
    ```

1. Source the setup file. *Source the other packages too, or add them to ~/.bashrc*
    ```
    source install/setup.bash
    ```

1. Connect lidar and real-sense camera with usb cables.

1. Run real-sense node in new terminal.
    ```
    ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
    ```

1. Run lidar node in new terminal. ENSURE the `port` parameter in the `params` file is correct.
    ```
    ros2 launch ydlidar_ros2_driver ydlidar_launch.py 
    ```
   
1. Run ros2 launch
    ```
    ros2 launch main_package launch_sim.launch.py
    ```

## VS Code Preferences (User JSON)

* yapf (EeyoreLee)

* XML (Red Hat)

```json
{
  "workbench.colorTheme": "Default High Contrast",
  "[python]": {
    "editor.formatOnSaveMode": "file",
    "editor.formatOnSave": true,
    "editor.defaultFormatter": "eeyore.yapf",
    "editor.formatOnType": false,
  },
  "yapf.args": [
    "--style={based_on_style: google, column_limit: 80, indent_width: 4}",
    "--style={based_on_style: pep8; DEDENT_CLOSING_BRACKETS: True}",
    "--style={based_on_style: pep8; ALLOW_SPLIT_BEFORE_DICT_VALUE: False, SPLIT_ALL_COMMA_SEPARATED_VALUES: True}"
  ],
  "redhat.telemetry.enabled": true,
  "[xml]": {
    "editor.defaultFormatter": "redhat.vscode-xml",
    "editor.tabSize": 4,
    "editor.trimAutoWhitespace": true,
    "editor.formatOnSaveMode": "file",
    "editor.formatOnSave": true,
  },
  "xml.format.maxLineWidth": 120,
  
}
```

## Main Package Template

* https://github.com/joshnewans/my_bot
