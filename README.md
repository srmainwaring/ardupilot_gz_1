# ardupilot_gz

Adapted from the [`ros_gz_project_template`](https://github.com/gazebosim/ros_gz_project_template) project integrating ROS 2 and Gazebo.

## Included packages

* `ardupilot_gz_description` - Contains the SDF description of the simulated
  system.

* `ardupilot_gz_gazebo` - Contains Gazebo specific code and configurations such
  as system plugins.

* `ardupilot_gz_application` - Contains ROS 2 specific code and configuration.

* `ardupilot_gz_bringup` - Contains launch files and high level utilities.


## Install

### Dependencies

1. Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)

1. Install [Gazebo Garden](https://gazebosim.org/docs/garden)

1. Install necessary tools

    `sudo apt install python3-vcstool python3-colcon-common-extensions git wget`

### Usage

1. Create a workspace, for example:

    ```
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

1. Clone the project:

    ```
    git clone https://github.com/srmainwaring/ardupilot_gz.git
    ```

1. Set the Gazebo version to Garden:

    ```
    export GZ_VERSION=garden
    ```

1. Install ROS dependencies

    ```
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y -i
    ```

1. Build and install

    ```
    cd ~/ros2_ws
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

### Run

1. Source the workspace

    `. ~/ros2_ws/install/setup.sh`

1. Launch the simulation

    `ros2 launch ardupilot_gz_bringup bringup_iris.launch.py`

1. Launch the a GCS

    `mavproxy.py --console --map`

1. Inspect topics

    ```bash
    $ ros2 topic list
    /ROS2_BatteryState0
    /ROS2_NavSatFix0
    /ROS2_Time
    /clicked_point
    /clock
    /goal_pose
    /initialpose
    /iris/odometry
    /joint_states
    /parameter_events
    /robot_description
    /rosout
    /tf
    /tf_static
    ```


## Notes

1. Additional dependencies

    `ros_gz` has a dependency on `gps_msgs` included in

    ```bash
    git clone https://github.com/swri-robotics/gps_umd.git -b ros2-devel
    ```

    Add `COLCON_IGNORE` to `gpsd_client` as this package is not required and
    will not build on macOS. 

2. sdformat_urdf

    On macOS the `robot_state_publisher` node cannot load the
    `sdformat_urdf_plugin` plugin unless the
    suffix is changed:

    ```bash
    cd ./install/sdformat_urdf/lib
    ln -s libsdformat_urdf_plugin.so libsdformat_urdf_plugin.dylib
    ```

3. Model URIs

    The `sdformat_urdf` plugin requires that the `<uri>` element use
    the `package` prefix for a resource to be located by RViz.

    Ensure the environment variable `SDF_PATH` is set:
    
    ```bash
    export SDF_PATH=$GZ_SIM_RESOURCE_PATH
    ```

    This is checked in the launch file as `SDF_PATH` is not usually set
    by the `ament` environment hooks. 

4. SDFormat environment variables

      - SDF_PROTOCOL_VERSION
      - SDF_VERSION
      - SDF_MAJOR_VERSION
      - SDF_MINOR_VERSION
      - SDF_PATCH_VERSION
      - SDF_PKG_VERSION
      - SDF_VERSION_FULL
      - SDF_VERSION_NAMESPACE
      - SDF_VERSION_HEADER
      - SDF_SHARE_PATH = CMAKE_INSTALL_FULL_DATAROOTDIR
      - SDF_VERSION_PATH
      - SDF_PATH - set to GZ_SIM_RESOURCE_PATH

