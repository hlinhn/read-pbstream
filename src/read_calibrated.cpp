#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"

int
main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cerr << "Read IMU calibration and GPS offset from a .pbstream file. Supply a filename\n";
    return 1;
  }

  std::string state_filename(argv[1]);
  cartographer::io::ProtoStreamReader stream(state_filename);
  cartographer::io::ProtoStreamDeserializer deserializer(&stream);
  cartographer::mapping::proto::SerializedData proto;

  while (deserializer.ReadNextSerializedData(&proto))
  {
    if (proto.data_case() == cartographer::mapping::proto::SerializedData::kTrajectoryData)
    {
      std::cout << "==============================\n";
      std::cout << "Trajectory ID: " << proto.trajectory_data().trajectory_id() << std::endl;
      std::cout << "\tIMU calibration\n";
      std::cout << "\t\t" << proto.trajectory_data().imu_calibration().x() << " "
                << proto.trajectory_data().imu_calibration().y() << " " << proto.trajectory_data().imu_calibration().z()
                << " " << proto.trajectory_data().imu_calibration().w() << std::endl;
      std::cout << "\tGPS offset to map\n";
      std::cout << "\t\t" << proto.trajectory_data().fixed_frame_origin_in_map().translation().x() << " "
                << proto.trajectory_data().fixed_frame_origin_in_map().translation().y() << " "
                << proto.trajectory_data().fixed_frame_origin_in_map().translation().z() << std::endl;

      std::cout << "\t\t" << proto.trajectory_data().fixed_frame_origin_in_map().rotation().x() << " "
                << proto.trajectory_data().fixed_frame_origin_in_map().rotation().y() << " "
                << proto.trajectory_data().fixed_frame_origin_in_map().rotation().z() << " "
                << proto.trajectory_data().fixed_frame_origin_in_map().rotation().w() << std::endl;
    }
  }
  return 0;
}
