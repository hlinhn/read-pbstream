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
    if (proto.data_case() == cartographer::mapping::proto::SerializedData::kTrajectoryData) {
      std::cout << proto.trajectory_data().trajectory_id() << std::endl;
      std::cout << proto.trajectory_data().imu_calibration().x() << " "
		<< proto.trajectory_data().imu_calibration().y() << " "
		<< proto.trajectory_data().imu_calibration().z() << " "
		<< proto.trajectory_data().imu_calibration().w() << std::endl;
      
      break;
    }
  }
  return 0;
}
