#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"
#include "cartographer/mapping/id.h"
#include <string>

int
main(int argc, char* argv[])
{
  if (argc < 4)
  {
    std::cerr << "Supply a pbstream file and submap ID\n";
    return 1;
  }

  std::string state_filename(argv[1]);
  cartographer::io::ProtoStreamReader stream(state_filename);
  cartographer::io::ProtoStreamDeserializer deserializer(&stream);
  cartographer::mapping::proto::SerializedData proto;

  int trajectory_id = std::stoi(argv[2]);
  int submap_id = std::stoi(argv[3]);

  while (deserializer.ReadNextSerializedData(&proto))
  {
    if (proto.data_case() == cartographer::mapping::proto::SerializedData::kSubmap)
    {
      if (proto.submap().submap_id().trajectory_id() == trajectory_id
          && proto.submap().submap_id().submap_index() == submap_id)
      {
        std::cout << "==============================\n";
        auto hybrid_grid = proto.submap().submap_3d().high_resolution_hybrid_grid();
        std::cout << hybrid_grid.resolution() << std::endl;
        break;
      }
    }
  }
  return 0;
}
