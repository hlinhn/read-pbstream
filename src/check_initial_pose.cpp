#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/probability_values.h"
#include <bits/stdint-uintn.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
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

        int max_x = 0;
        int max_y = 0;
        int min_x = 0;
        int min_y = 0;

        for (int i = 0; i < hybrid_grid.x_indices_size(); i++)
        {
          if (hybrid_grid.x_indices(i) > max_x)
          {
            max_x = hybrid_grid.x_indices(i);
          }
          if (hybrid_grid.x_indices(i) < min_x)
          {
            min_x = hybrid_grid.x_indices(i);
          }
          if (hybrid_grid.y_indices(i) > max_y)
          {
            max_y = hybrid_grid.y_indices(i);
          }
          if (hybrid_grid.y_indices(i) < min_y)
          {
            min_y = hybrid_grid.y_indices(i);
          }
        }
        std::cout << max_x << " " << min_x << " " << max_y << " " << min_y << std::endl;

        cv::Mat image = cv::Mat::zeros(max_x - min_x + 1, max_y - min_y + 1, CV_8UC3);

        float min = 100.0;
        float max = 0.0;
        for (int i = 0; i < hybrid_grid.x_indices_size(); i++)
        {
          auto val = cartographer::mapping::ValueToProbability(hybrid_grid.values(i));
          image.at<cv::Vec<uint8_t, 3>>(hybrid_grid.x_indices(i) - min_x, hybrid_grid.y_indices(i) - min_y)[0] =
              val * 255;
          if (val > max)
          {
            max = val;
          }
          if (val < min)
          {
            min = val;
          }
        }
        std::cout << max << " " << min << std::endl;
        cv::imwrite("/home/linh/test_hybrid.png", image);
        break;
      }
    }
  }
  return 0;
}
