# ardupilot_gz

This project contains ROS 2 packages for simulating models controlled
by ArduPilot SITL with DDS support in Gazebo.

The project is adapted from the [`ros_gz_project_template`](https://github.com/gazebosim/ros_gz_project_template) project.

## Included packages

* `ardupilot_gz_description` - Contains the SDFormat description of the simulated
  system.

* `ardupilot_gz_gazebo` - Contains Gazebo specific code such as system plugins.

* `ardupilot_gz_application` - Contains ROS 2 specific code and configuration.

* `ardupilot_gz_bringup` - Contains launch files and high level utilities.


## Prerequisites

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- Install [Gazebo Garden](https://gazebosim.org/docs/garden)

## Install

#### 1. Create a workspace folder

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```

#### 2. Get the project source

```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/srmainwaring/ardupilot_gz/wips/wip-iris-camera/ros2_gz.repos
vcs import --recursive < ros2_gz.repos
```

#### 3. Set the Gazebo version to Garden:

```bash
export GZ_VERSION=garden
```

#### 4. Update ROS dependencies

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --rosdistro humble --from-paths src -i -r -y
```

#### 5. Build

```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_TESTING=ON
```

#### 6. Test

```bash
source ./install/setup.bash
colcon test --packages-select ardupilot_dds_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup 
colcon test-result --all --verbose
```

## Usage

#### 1. Source the workspace

```bash
source ~/ros2_ws/install/setup.sh
```

#### 2. Launch the simulation

```bash
ros2 launch ardupilot_gz_bringup bringup_iris.launch.py
```

#### 3. Launch a GCS (MAVPorxy)

```bash
mavproxy.py --console --map
```

#### 4. Inspect topics

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

#### 1. Additional dependencies

`ros_gz` has a dependency on `gps_msgs` included in

```bash
git clone https://github.com/swri-robotics/gps_umd.git -b ros2-devel
```

Add `COLCON_IGNORE` to `gpsd_client` as this package is not required and
will not build on macOS. 

#### 2. `sdformat_urdf`

On macOS the `robot_state_publisher` node cannot load the
`sdformat_urdf_plugin` plugin unless the
suffix is changed:

```bash
cd ./install/sdformat_urdf/lib
ln -s libsdformat_urdf_plugin.so libsdformat_urdf_plugin.dylib
```

#### 3. Model URIs

The `sdformat_urdf` plugin requires that the `<uri>` element use
the `package` prefix for a resource to be located by RViz.


#### 4. SDFormat environment variables

The `sdformat_urdf` plugin uses the `sdformat13` libraries to parse the
model description. `sdformat13` relies on the environment variable
`SDF_PATH` to resolve model resources. This is usually set in `gz-sim7`,
however when using the plugins standalone, for instance in the bring-up
launch files, `SDF_PATH` must be set otherwise the plugin will not resolve
the models and their dependencies.

```bash
source ~/ros2_ws/install/setup.sh
export SDF_PATH=$GZ_SIM_RESOURCE_PATH
```

This is checked in the launch file as `SDF_PATH` is not usually set
by the `ament` environment hooks.

