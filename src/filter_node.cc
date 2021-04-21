#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>
#include "cartographer/io/internal/mapping_state_serialization.h"
#include <cartographer/io/serialization_format_migration.h>
#include <cartographer/transform/transform.h>
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"

using namespace cartographer;
using namespace cartographer::io;
using mapping::proto::SerializedData;

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cout << "USAGE: ./read_calibrated_filter input_pbstream submap_index trajectory_id output_pbstream\n";
        return 1;
    }
    auto file_resolver = ::absl::make_unique<common::ConfigurationFileResolver>(
        std::vector<std::string>{std::string("/opt/ros/melodic/share/cartographer") +
                "/configuration_files"});
    const std::string kCode = R"text(
      include "map_builder.lua"
      MAP_BUILDER.use_trajectory_builder_3d = true
      return MAP_BUILDER)text";
    cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
        kCode, std::move(file_resolver));
    const auto options =
        mapping::CreateMapBuilderOptions(&lua_parameter_dictionary);

    common::ThreadPool thread_pool(1);
    mapping::PoseGraph3D pose_graph(
        options.pose_graph_options(),
        absl::make_unique<mapping::optimization::OptimizationProblem3D>(
            options.pose_graph_options().optimization_problem_options()),
        &thread_pool);

    int exclude = std::stoi(argv[2]);
    int trajectory_id = std::stoi(argv[3]);

    std::string state_filename(argv[1]);
    cartographer::io::ProtoStreamReader stream(state_filename);
    cartographer::io::ProtoStreamDeserializer deserializer(&stream);

    mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
    const auto& all_builder_options_proto =
        deserializer.all_trajectory_builder_options();

    std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
        trajectory_builder_options;
    for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i) {
        auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
        const auto& options_with_sensor_ids_proto =
            all_builder_options_proto.options_with_sensor_ids(i);
        trajectory_builder_options.push_back(options_with_sensor_ids_proto);
    }

    for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
        constraint_proto.mutable_submap_id()->set_trajectory_id(
            constraint_proto.submap_id().trajectory_id());
        constraint_proto.mutable_node_id()->set_trajectory_id(
            constraint_proto.node_id().trajectory_id());
    }

    mapping::MapById<mapping::SubmapId, transform::Rigid3d> submap_poses;
    int submap_count_orig = 0;
    int submap_count_new = 0;
    transform::Rigid3d excluded_submap;
    std::set<int> excluded_submaps;
    for (const mapping::proto::Trajectory& trajectory_proto :
             pose_graph_proto.trajectory()) {
        for (const mapping::proto::Trajectory::Submap& submap_proto :
                 trajectory_proto.submap()) {
            submap_count_orig++;
            if (trajectory_proto.trajectory_id() == trajectory_id && submap_proto.submap_index() >= exclude)
            {
                if (submap_proto.submap_index() == exclude)
                {
                    excluded_submap = transform::ToRigid3(submap_proto.pose());
                }
                excluded_submaps.insert(submap_proto.submap_index());
                continue;
            }
            submap_count_new++;
            submap_poses.Insert(mapping::SubmapId{trajectory_proto.trajectory_id(),
                        submap_proto.submap_index()},
                transform::ToRigid3(submap_proto.pose()));
        }
    }

    std::cout << submap_count_orig << " " << submap_count_new << std::endl;

    double longest_distance = 15.0;
    int node_id = -1;
    mapping::MapById<mapping::NodeId, transform::Rigid3d> node_poses;
    std::set<int> excluded_nodes;
    for (const mapping::proto::Trajectory& trajectory_proto :
             pose_graph_proto.trajectory()) {
        if (trajectory_proto.trajectory_id() == trajectory_id)
        {
            for (const mapping::proto::Trajectory::Node& node_proto :
                     trajectory_proto.node()) {
                double distance = (transform::ToRigid3(node_proto.pose()).translation() -
                                   excluded_submap.translation()).norm();
                if (distance < longest_distance)
                {
                    longest_distance = distance;
                    node_id = node_proto.node_index();
                }
            }
        }
    }

    int node_count = 0;
    int node_count_new = 0;
    std::cout << longest_distance << " " << node_id << std::endl;
    for (const mapping::proto::Trajectory& trajectory_proto :
             pose_graph_proto.trajectory()) {
        for (const mapping::proto::Trajectory::Node& node_proto :
                 trajectory_proto.node()) {
            node_count += 1;
            if (trajectory_proto.trajectory_id() == trajectory_id && node_proto.node_index() >= node_id)
            {
                excluded_nodes.insert(node_proto.node_index());
                continue;
            }
            node_count_new++;
            node_poses.Insert(mapping::NodeId{trajectory_proto.trajectory_id(),
                        node_proto.node_index()},
                transform::ToRigid3(node_proto.pose()));
        }
    }
    std::cout << node_count << " " << node_count_new << std::endl;
    auto mutable_constraint = pose_graph_proto.constraint();
    std::cout << mutable_constraint.size() << " ";
    mutable_constraint.erase(std::remove_if(mutable_constraint.begin(), mutable_constraint.end(),
                                            [trajectory_id, excluded_nodes, excluded_submaps]
                                            (const auto& cons){
                                                bool submap_in = cons.submap_id().trajectory_id() == trajectory_id
                                                    && excluded_submaps.count(cons.submap_id().submap_index()) > 0;
                                                bool node_in = cons.node_id().trajectory_id() == trajectory_id
                                                    && excluded_nodes.count(cons.node_id().node_index()) > 0;
                                                return submap_in || node_in;
                                            }),
                             mutable_constraint.end());
    std::cout << mutable_constraint.size() << std::endl;

    // Set global poses of landmarks.
    for (const auto& landmark : pose_graph_proto.landmark_poses()) {
        pose_graph.SetLandmarkPose(landmark.landmark_id(),
                                   transform::ToRigid3(landmark.global_pose()),
                                   true);
    }

    mapping::MapById<mapping::SubmapId, mapping::proto::Submap>
        submap_id_to_submap;
    mapping::MapById<mapping::NodeId, mapping::proto::Node> node_id_to_node;
    SerializedData proto;
    while (deserializer.ReadNextSerializedData(&proto)) {
        switch (proto.data_case()) {
        case SerializedData::kPoseGraph:
            LOG(FATAL) << "Found multiple serialized `PoseGraph`. Serialized "
                "stream likely corrupt!.";
        case SerializedData::kAllTrajectoryBuilderOptions:
            LOG(FATAL) << "Found multiple serialized "
                "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                "corrupt!.";
        case SerializedData::kSubmap: {
            if (proto.submap().submap_id().trajectory_id() == trajectory_id
                && excluded_submaps.count(proto.submap().submap_id().submap_index()) > 0)
            {
                break;
            }
            if (proto.submap().has_submap_3d())
            {
                proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
                    proto.submap().submap_id().trajectory_id());
                submap_id_to_submap.Insert(
                    mapping::SubmapId{proto.submap().submap_id().trajectory_id(),
                            proto.submap().submap_id().submap_index()},
                    proto.submap());
            }
            break;
        }
        case SerializedData::kNode: {
            if (proto.node().node_id().trajectory_id() == trajectory_id &&
                excluded_nodes.count(proto.node().node_id().node_index()) > 0) {
                break;
            }

            proto.mutable_node()->mutable_node_id()->set_trajectory_id(
                proto.node().node_id().trajectory_id());
            const mapping::NodeId node_id(proto.node().node_id().trajectory_id(),
                                          proto.node().node_id().node_index());
            const transform::Rigid3d& node_pose = node_poses.at(node_id);
            pose_graph.AddNodeFromProto(node_pose, proto.node());
            node_id_to_node.Insert(node_id, proto.node());

            break;
        }
        case SerializedData::kTrajectoryData: {
            proto.mutable_trajectory_data()->set_trajectory_id(
                proto.trajectory_data().trajectory_id());
            pose_graph.SetTrajectoryDataFromProto(proto.trajectory_data());
            break;
        }
        case SerializedData::kImuData: {
            pose_graph.AddImuData(proto.imu_data().trajectory_id(),
                                  sensor::FromProto(proto.imu_data().imu_data()));
            break;
        }
        case SerializedData::kOdometryData: {
            pose_graph.AddOdometryData(
                proto.odometry_data().trajectory_id(),
                sensor::FromProto(proto.odometry_data().odometry_data()));
            break;
        }
        case SerializedData::kFixedFramePoseData: {
            pose_graph.AddFixedFramePoseData(
                proto.fixed_frame_pose_data().trajectory_id(),
                sensor::FromProto(
                    proto.fixed_frame_pose_data().fixed_frame_pose_data()));
            break;
        }
        case SerializedData::kLandmarkData: {
            pose_graph.AddLandmarkData(
                proto.landmark_data().trajectory_id(),
                sensor::FromProto(proto.landmark_data().landmark_data()));
            break;
        }
        default:
            LOG(WARNING) << "Skipping unknown message type in stream: "
                         << proto.GetTypeName();
        }
    }

    for (const auto& submap_id_submap : submap_id_to_submap) {
        pose_graph.AddSubmapFromProto(submap_poses.at(submap_id_submap.id),
                                      submap_id_submap.data);
    }

    pose_graph.AddSerializedConstraints(mapping::FromProto(mutable_constraint));

    io::ProtoStreamWriter output(argv[4]);

    io::WritePbStream(pose_graph, trajectory_builder_options, &output,
                      true);

    return 0;
}
