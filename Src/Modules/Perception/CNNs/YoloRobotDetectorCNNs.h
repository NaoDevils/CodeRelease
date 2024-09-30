#include <vector>

namespace YoloRobotDetectorCNNLower
{
  extern const unsigned int input_height;
  extern const unsigned int input_width;
  extern const unsigned int input_channel;
  extern const unsigned int output_height;
  extern const unsigned int output_width;
  extern const unsigned int output_channel;
  extern const unsigned int num_of_boxes;
  extern const unsigned int num_of_classes;
  extern const unsigned int num_of_coords;
  extern const std::vector<float> anchors;

  void cnn(float* in_0, float* out_0);
} // namespace YoloRobotDetectorCNNLower

namespace YoloRobotDetectorCNNUpper
{
  extern const unsigned int input_height;
  extern const unsigned int input_width;
  extern const unsigned int input_channel;
  extern const unsigned int output_height;
  extern const unsigned int output_width;
  extern const unsigned int output_channel;
  extern const unsigned int num_of_boxes;
  extern const unsigned int num_of_classes;
  extern const unsigned int num_of_coords;
  extern const std::vector<float> anchors;

  void cnn(float* in_0, float* out_0);
} // namespace YoloRobotDetectorCNNUpper
