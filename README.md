# Parse pbstream file from Google Cartographer

This is a work in progress.

## How to build

Clone this to a `catkin` workspace

` catkin build read_calibrated`

This project uses some internal headers from `cartographer`, so it expects that you have `cartographer` repository cloned somewhere in your system.
The current `CMakeLists.txt` assumes this repository is in the same workspace as `cartographer`. If this is not the case, change `cartographer_INCLUDE_INTERNAL_DIRS` in `CMakeLists.txt` file to reflect the true location.

It also needs `Protobuf` library.

## How to run

There are 3 executables

- `read_calibrated_node` reads out IMU calibration and GPS offset results
- `read_calibrated_filter` filters out submaps in the trajectory ID supplied, whose submap ID is larger than the submap ID supplied
- `read_calibrated_convert` converts one trajectory's poses to a ROS bag file
