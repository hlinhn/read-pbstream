#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"
#include <bits/stdint-intn.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <string>

ros::Time
convertFromTrajectoryTime(int64_t timestamp)
{
  int64_t ns_since_unix_epoch =
      (timestamp - cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
  ros::Time ros_time;
  ros_time.fromNSec(ns_since_unix_epoch);
  return ros_time;
}

int
main(int argc, char* argv[])
{
  if (argc < 3)
  {
    std::cerr << "Read trajectory from pbstream file and convert it to bag file of positions. Supply a filename and "
                 "the trajectory ID\n";
    return 1;
  }

  std::string state_filename(argv[1]);
  cartographer::io::ProtoStreamReader stream(state_filename);

  auto pose_graph = cartographer::io::DeserializePoseGraphFromFile(state_filename);
  int trajectory_id = std::stoi(argv[2]);
  auto trajectory = pose_graph.trajectory(trajectory_id);

  rosbag::Bag bag;
  bag.open("/home/linh/Downloads/bags/convert_from_trajectory.bag", rosbag::bagmode::Write);
  int count_sub = 0;
  nav_msgs::Odometry odom;
  odom.header.frame_id = "map";

  for (const auto node : trajectory.node())
  {
    count_sub++;
    odom.header.stamp = convertFromTrajectoryTime(node.timestamp());
    odom.pose.pose.position.x = node.pose().translation().x();
    odom.pose.pose.position.y = node.pose().translation().y();
    odom.pose.pose.position.z = node.pose().translation().z();
    bag.write("/global_odom", odom.header.stamp, odom);
  }
  std::cout << count_sub << std::endl;
  bag.close();
  return 0;
}
