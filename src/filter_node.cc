#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>
#include <cartographer/io/serialization_format_migration.h>
#include <cartographer/transform/transform.h>
#include <cstdlib>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

using namespace cartographer;
using namespace cartographer::io;
using mapping::proto::SerializedData;

std::map<int, std::vector<std::pair<int, int>>>
readExcludedSubmaps(const std::string& file_path)
{
  std::map<int, std::vector<std::pair<int, int>>> to_exclude;
  const auto yaml = YAML::LoadFile(file_path);
  const auto& submaps = yaml["submaps"];
  for (auto it = submaps.begin(); it != submaps.end(); ++it)
  {
    const auto submap_yaml = *it;
    const auto trajectory_id = submap_yaml["id"].as<int>();

    std::vector<std::pair<int, int>> ranges;
    const auto& ranges_yaml = submap_yaml["ranges"];
    for (auto range_it = ranges_yaml.begin(); range_it != ranges_yaml.end(); ++range_it)
    {
      const auto one_range = *range_it;
      const auto start = one_range["start"].as<int>();
      const auto end = one_range["end"].as<int>();
      ranges.emplace_back(start, end);
    }
    to_exclude.insert({trajectory_id, ranges});
  }
  return to_exclude;
}

int
findNearestNodeId(transform::Rigid3d submap_pose, mapping::proto::Trajectory trajectory_proto)
{
  double max_distance = 1.0;
  int node_index = -1;
  for (const auto& node_proto : trajectory_proto.node())
  {
    double distance = (transform::ToRigid3(node_proto.pose()).translation() - submap_pose.translation()).norm();
    if (distance < max_distance)
    {
      max_distance = distance;
      node_index = node_proto.node_index();
    }
  }
  return node_index;
}

std::map<int, std::vector<std::pair<int, int>>>
findExcludedNodes(std::map<int, std::vector<std::pair<int, int>>> excluded_submaps,
                  mapping::proto::PoseGraph pose_graph_proto,
                  mapping::MapById<mapping::SubmapId, transform::Rigid3d> submap_poses)
{
  std::map<int, std::vector<std::pair<int, int>>> to_exclude;
  for (const auto& trajectory_proto : pose_graph_proto.trajectory())
  {
    auto trajectory_id = trajectory_proto.trajectory_id();
    if (excluded_submaps.count(trajectory_id) == 0)
    {
      continue;
    }
    std::vector<std::pair<int, int>> excluded_node;
    for (const auto& range : excluded_submaps[trajectory_id])
    {
      auto start = findNearestNodeId(submap_poses.at({trajectory_id, range.first}), trajectory_proto);
      auto end = findNearestNodeId(submap_poses.at({trajectory_id, range.second}), trajectory_proto);
      excluded_node.emplace_back(start, end);
    }
    to_exclude.emplace(trajectory_id, excluded_node);
  }
  return to_exclude;
}

bool
excludeThis(int trajectory_id, int index, std::map<int, std::vector<std::pair<int, int>>> exclude_list)
{
  if (exclude_list.count(trajectory_id) == 0)
  {
    return false;
  }
  const auto list = exclude_list[trajectory_id];
  for (const auto range : list)
  {
    if (index >= range.first && index <= range.second)
    {
      return true;
    }
  }
  return false;
}

int
main(int argc, char* argv[])
{
  if (argc < 3)
  {
    std::cout << "USAGE: ./read_calibrated_filter input_pbstream to_exclude_list output_pbstream\n";
    return 1;
  }
  auto file_resolver = ::absl::make_unique<common::ConfigurationFileResolver>(
      std::vector<std::string> {std::string("/opt/ros/melodic/share/cartographer") + "/configuration_files"});
  const std::string kCode = R"text(
      include "map_builder.lua"
      MAP_BUILDER.use_trajectory_builder_3d = true
      return MAP_BUILDER)text";
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(kCode, std::move(file_resolver));
  const auto options = mapping::CreateMapBuilderOptions(&lua_parameter_dictionary);

  common::ThreadPool thread_pool(1);
  mapping::PoseGraph3D pose_graph(options.pose_graph_options(),
                                  absl::make_unique<mapping::optimization::OptimizationProblem3D>(
                                      options.pose_graph_options().optimization_problem_options()),
                                  &thread_pool);

  auto exclude_list = readExcludedSubmaps(argv[2]);

  std::string state_filename(argv[1]);
  cartographer::io::ProtoStreamReader stream(state_filename);
  cartographer::io::ProtoStreamDeserializer deserializer(&stream);

  mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const auto& all_builder_options_proto = deserializer.all_trajectory_builder_options();

  // We need to either remap trajectory id or make sure trajectory_builder_options here can be sort of a map between
  // trajectory id and options
  std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds> trajectory_builder_options;
  int max_trajectory_id = 0;
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i)
  {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
    if (trajectory_proto.trajectory_id() > max_trajectory_id)
    {
      max_trajectory_id = trajectory_proto.trajectory_id();
    }
  }

  trajectory_builder_options.resize(max_trajectory_id + 1);
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i)
  {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
    const auto& options_with_sensor_ids_proto = all_builder_options_proto.options_with_sensor_ids(i);
    trajectory_builder_options[trajectory_proto.trajectory_id()] = options_with_sensor_ids_proto;
    pose_graph.FreezeTrajectory(trajectory_proto.trajectory_id());
  }

  mapping::MapById<mapping::SubmapId, transform::Rigid3d> submap_poses;
  int submap_count_orig = 0;
  int submap_count_new = 0;
  std::set<mapping::SubmapId> excluded_submaps;
  for (const mapping::proto::Trajectory& trajectory_proto : pose_graph_proto.trajectory())
  {
    for (const mapping::proto::Trajectory::Submap& submap_proto : trajectory_proto.submap())
    {
      submap_poses.Insert(mapping::SubmapId {trajectory_proto.trajectory_id(), submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
      submap_count_orig++;
      if (excludeThis(trajectory_proto.trajectory_id(), submap_proto.submap_index(), exclude_list))
      {
        excluded_submaps.insert({trajectory_proto.trajectory_id(), submap_proto.submap_index()});
        // std::cout << "Excluding " << trajectory_proto.trajectory_id() << " " << submap_proto.submap_index()
        //           << std::endl;
        continue;
      }
      // std::cout << "Adding " << trajectory_proto.trajectory_id() << " " << submap_proto.submap_index() << std::endl;
      submap_count_new++;
    }
  }

  std::cout << "Submap count: " << submap_count_orig << " " << submap_count_new << std::endl;
  // std::cout << "Submap poses count: " << submap_poses.size() << std::endl;

  mapping::MapById<mapping::NodeId, transform::Rigid3d> node_poses;
  std::set<mapping::NodeId> excluded_nodes;

  int count_constraints = 0;
  int count_intra = 0;

  for (const auto& constraint : pose_graph_proto.constraint())
  {
    count_constraints++;
    if (constraint.tag() != mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP)
    {
      continue;
    }
    count_intra++;
    if (excluded_submaps.count({constraint.submap_id().trajectory_id(), constraint.submap_id().submap_index()}) == 0)
    {
      continue;
    }
    excluded_nodes.insert({constraint.node_id().trajectory_id(), constraint.node_id().node_index()});
  }

  std::cout << count_constraints << " " << count_intra << " " << std::endl;
  int node_count = 0;
  int node_count_new = 0;

  auto closest_nodes = findExcludedNodes(exclude_list, pose_graph_proto, submap_poses);

  for (const mapping::proto::Trajectory& trajectory_proto : pose_graph_proto.trajectory())
  {
    for (const mapping::proto::Trajectory::Node& node_proto : trajectory_proto.node())
    {
      node_count += 1;
      if (excludeThis(trajectory_proto.trajectory_id(), node_proto.node_index(), closest_nodes))
      {
        excluded_nodes.insert({trajectory_proto.trajectory_id(), node_proto.node_index()});
        continue;
      }
      node_count_new++;
      node_poses.Insert(mapping::NodeId {trajectory_proto.trajectory_id(), node_proto.node_index()},
                        transform::ToRigid3(node_proto.pose()));
    }
  }
  std::cout << "Node count: " << node_count << " " << node_count_new << std::endl;
  // std::cout << "Node poses count: " << node_poses.size() << std::endl;

  // Set global poses of landmarks.
  for (const auto& landmark : pose_graph_proto.landmark_poses())
  {
    pose_graph.SetLandmarkPose(landmark.landmark_id(), transform::ToRigid3(landmark.global_pose()), true);
  }

  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto))
  {
    switch (proto.data_case())
    {
      case SerializedData::kPoseGraph:
        LOG(FATAL) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(FATAL) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
      case SerializedData::kSubmap:
      {
        const mapping::SubmapId submap_id {proto.submap().submap_id().trajectory_id(),
                                           proto.submap().submap_id().submap_index()};
        if (excluded_submaps.count(submap_id) > 0)
        {
          break;
        }
        try
        {
          pose_graph.AddSubmapFromProto(submap_poses.at(submap_id), proto.submap());
        }
        catch (std::out_of_range& e)
        {
          std::cout << "Requested submap id is " << submap_id.trajectory_id << " " << submap_id.submap_index
                    << std::endl;
        }
        break;
      }
      case SerializedData::kNode:
      {
        const mapping::NodeId node_id {proto.node().node_id().trajectory_id(), proto.node().node_id().node_index()};
        if (excluded_nodes.count(node_id) > 0)
        {
          break;
        }

        try
        {
          const transform::Rigid3d& node_pose = node_poses.at(node_id);
          pose_graph.AddNodeFromProto(node_pose, proto.node());
        }
        catch (std::out_of_range& e)
        {
          std::cout << "Requested node id is " << node_id.trajectory_id << " " << node_id.node_index << std::endl;
        }
        break;
      }
      case SerializedData::kTrajectoryData:
      {
        pose_graph.SetTrajectoryDataFromProto(proto.trajectory_data());
        break;
      }
      case SerializedData::kImuData:
      {
        break;
      }
      case SerializedData::kOdometryData:
      {
        break;
      }
      case SerializedData::kFixedFramePoseData:
      {
        break;
      }
      case SerializedData::kLandmarkData:
      {
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: " << proto.GetTypeName();
    }
  }

  std::cout << "Add constraints\n";

  for (const auto& constraint : pose_graph_proto.constraint())
  {
    if (constraint.tag() != mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP)
    {
      continue;
    }
    mapping::NodeId node_id {constraint.node_id().trajectory_id(), constraint.node_id().node_index()};
    mapping::SubmapId submap_id {constraint.submap_id().trajectory_id(), constraint.submap_id().submap_index()};
    if (excluded_submaps.count(submap_id) > 0)
    {
      continue;
    }
    if (excluded_nodes.count(node_id) > 0)
    {
      continue;
    }
    pose_graph.AddNodeToSubmap(node_id, submap_id);
  }

  std::cout << "Pose graph has " << pose_graph.GetTrajectoryData().size() << " trajectory" << std::endl;
  // std::cout << "Trajectory builder options has " << trajectory_builder_options.size() << std::endl;
  io::ProtoStreamWriter output(argv[3]);
  std::cout << "Writing file\n";
  io::WritePbStream(pose_graph, trajectory_builder_options, &output, true);

  return 0;
}
