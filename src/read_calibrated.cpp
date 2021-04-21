#include <read_calibrated/read_calibrated.h>

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Need a filename\n";
    return 1;
  }

  std::string state_filename(argv[1]);
  cartographer::io::ProtoStreamReader stream(state_filename);
  cartographer::io::ProtoStreamDeserializer deserializer(&stream);
  cartographer::mapping::proto::SerializedData proto;

  while (deserializer.ReadNextSerializedData(&proto)) {
/*
    if (proto.data_case() == cartographer::mapping::proto::SerializedData::kFixedFramePoseData)
    {
      std::cout << proto.fixed_frame_pose_data().fixed_frame_pose_data().pose().translation().x() << " "
      << proto.fixed_frame_pose_data().fixed_frame_pose_data().pose().translation().y() << " "
      << proto.fixed_frame_pose_data().fixed_frame_pose_data().pose().translation().z() << std::endl;
      std::cout << proto.fixed_frame_pose_data().fixed_frame_pose_data().pose().rotation().x() << " "
      << proto.fixed_frame_pose_data().fixed_frame_pose_data().pose().rotation().y() << " "
      << proto.fixed_frame_pose_data().fixed_frame_pose_data().pose().rotation().z() << " "
      << proto.fixed_frame_pose_data().fixed_frame_pose_data().pose().rotation().w() << std::endl;

      break;
    }
*/
    if (proto.data_case() == cartographer::mapping::proto::SerializedData::kTrajectoryData) {
      std::cout << proto.trajectory_data().trajectory_id() << std::endl;
      std::cout << proto.trajectory_data().imu_calibration().x() << " "
                << proto.trajectory_data().imu_calibration().y() << " "
                << proto.trajectory_data().imu_calibration().z() << " "
                << proto.trajectory_data().imu_calibration().w() << std::endl;
      std::cout << proto.trajectory_data().fixed_frame_origin_in_map().translation().x() << " "
                << proto.trajectory_data().fixed_frame_origin_in_map().translation().y() << " "
                << proto.trajectory_data().fixed_frame_origin_in_map().translation().z() << std::endl;

      std::cout << proto.trajectory_data().fixed_frame_origin_in_map().rotation().x() << " "
                << proto.trajectory_data().fixed_frame_origin_in_map().rotation().y() << " "
                << proto.trajectory_data().fixed_frame_origin_in_map().rotation().z() << " "
                << proto.trajectory_data().fixed_frame_origin_in_map().rotation().w() << std::endl;

      break;
    }//*/
  }
/*
  auto proto = deserializer.pose_graph();
//    if (proto.data_case() == cartographer::mapping::proto::SerializedData::kPoseGraph)
//    {
  std::cout << proto.trajectory().size() << std::endl;

  for (auto trajectory: proto.trajectory()) {
    //auto trajectory = proto.trajectory()[0];
  std::cout << trajectory.node().size() << std::endl;
  auto node = trajectory.node()[trajectory.node().size() - 1];
      std::cout << node.pose().translation().x() << " "
                << node.pose().translation().y() << " "
                << node.pose().translation().z() << std::endl
                << node.pose().rotation().x() << " "
                << node.pose().rotation().y() << " "
                << node.pose().rotation().z() << " "
                << node.pose().rotation().w() << std::endl;
  auto first_node = trajectory.node()[0];
      std::cout << first_node.pose().translation().x() << " "
                << first_node.pose().translation().y() << " "
                << first_node.pose().translation().z() << std::endl
                << first_node.pose().rotation().x() << " "
                << first_node.pose().rotation().y() << " "
                << first_node.pose().rotation().z() << " "
                << first_node.pose().rotation().w() << std::endl;
  }
  /*
  for (auto node : trajectory.node())
  {
    if (node.node_index() <= 10)
    {
      std::cout << node.pose().translation().x() << " "
                << node.pose().translation().y() << " "
                << node.pose().translation().z() << std::endl
                << node.pose().rotation().x() << " "
                << node.pose().rotation().y() << " "
                << node.pose().rotation().z() << " "
                << node.pose().rotation().w() << std::endl;
    }

    if (node.node_index() > 10)
    {
    break;
    }
    }
  */
//      break;
//}

//}
  return 0;
}
