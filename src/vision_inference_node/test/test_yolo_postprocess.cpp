#include <gtest/gtest.h>

#include <vector>

#include "vision_inference_node/yolo_postprocess.hpp"

namespace
{

using vision_inference_node::TensorValueType;
using vision_inference_node::TensorView;

TEST(YoloPostprocess, RestoresLetterboxCoordinates)
{
  const auto detection = vision_inference_node::restore_letterbox_box(
    160.0F, 240.0F, 320.0F, 320.0F, 0.9F, 2, 1280, 720, 640, 640);
  EXPECT_FLOAT_EQ(detection.x_min, 320.0F);
  EXPECT_FLOAT_EQ(detection.y_min, 200.0F);
  EXPECT_FLOAT_EQ(detection.x_max, 640.0F);
  EXPECT_FLOAT_EQ(detection.y_max, 360.0F);
  EXPECT_EQ(detection.class_id, 2);
}

TEST(YoloPostprocess, DecodesEndToEndOutputWithClassAwareNms)
{
  const std::vector<float> output{
    100.0F, 100.0F, 200.0F, 200.0F, 0.90F, 0.0F,
    105.0F, 105.0F, 205.0F, 205.0F, 0.80F, 0.0F,
    105.0F, 105.0F, 205.0F, 205.0F, 0.70F, 1.0F};
  const auto detections = vision_inference_node::decode_yolo_output(
    TensorView(output.data(), output.size(), TensorValueType::kFloat32),
    {1, 3, 6}, 640, 640, 640, 640, 0.25F, 0.45F, 10);
  ASSERT_EQ(detections.size(), 2U);
  EXPECT_EQ(detections[0].class_id, 0);
  EXPECT_FLOAT_EQ(detections[0].confidence, 0.90F);
  EXPECT_EQ(detections[1].class_id, 1);
}

TEST(YoloPostprocess, DecodesRawAnchorMajorOutput)
{
  const std::vector<float> output{
    320.0F, 320.0F, 100.0F, 80.0F, 0.90F,
    100.0F, 100.0F, 20.0F, 20.0F, 0.10F};
  const auto detections = vision_inference_node::decode_yolo_output(
    TensorView(output.data(), output.size(), TensorValueType::kFloat32),
    {1, 2, 5}, 640, 640, 640, 640, 0.25F, 0.45F, 10);
  ASSERT_EQ(detections.size(), 1U);
  EXPECT_FLOAT_EQ(detections[0].x_min, 270.0F);
  EXPECT_FLOAT_EQ(detections[0].y_min, 280.0F);
  EXPECT_FLOAT_EQ(detections[0].x_max, 370.0F);
  EXPECT_FLOAT_EQ(detections[0].y_max, 360.0F);
}

}  // namespace
