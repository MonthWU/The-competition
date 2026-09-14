#include "puzzle_perception_node/hsv_object_mask.hpp"

#include <iostream>

#include <opencv2/core.hpp>

int main()
{
  const puzzle_perception_node::HsvRange green{60, 90, 78, 201, 91, 245};
  const puzzle_perception_node::HsvRange white{0, 179, 0, 77, 160, 255};
  if (!puzzle_perception_node::valid_hsv_range(green) ||
    !puzzle_perception_node::valid_hsv_range(white))
  {
    std::cerr << "configured HSV range is invalid\n";
    return 1;
  }
  if (puzzle_perception_node::hsv_ranges_overlap(green, white)) {
    std::cerr << "green-paper and white-piece HSV ranges overlap\n";
    return 1;
  }

  cv::Mat hsv(1, 7, CV_8UC3);
  hsv.at<cv::Vec3b>(0, 0) = cv::Vec3b(60, 78, 91);    // green lower bound
  hsv.at<cv::Vec3b>(0, 1) = cv::Vec3b(90, 201, 245);  // green upper bound
  hsv.at<cv::Vec3b>(0, 2) = cv::Vec3b(0, 20, 230);    // white
  hsv.at<cv::Vec3b>(0, 3) = cv::Vec3b(75, 77, 200);   // white by low saturation
  hsv.at<cv::Vec3b>(0, 4) = cv::Vec3b(59, 150, 180);  // below green hue
  hsv.at<cv::Vec3b>(0, 5) = cv::Vec3b(75, 202, 180);  // above green saturation
  hsv.at<cv::Vec3b>(0, 6) = cv::Vec3b(0, 20, 159);    // below white value

  const cv::Mat green_mask = puzzle_perception_node::make_hsv_object_mask(hsv, green);
  const cv::Mat white_mask = puzzle_perception_node::make_hsv_object_mask(hsv, white);
  if (green_mask.empty() || white_mask.empty()) {
    std::cerr << "HSV mask generation failed\n";
    return 1;
  }
  const int expected_green[] = {255, 255, 0, 0, 0, 0, 0};
  const int expected_white[] = {0, 0, 255, 255, 0, 0, 0};
  for (int column = 0; column < hsv.cols; ++column) {
    if (green_mask.at<std::uint8_t>(0, column) != expected_green[column] ||
      white_mask.at<std::uint8_t>(0, column) != expected_white[column])
    {
      std::cerr << "object HSV classification mismatch at column " << column << '\n';
      return 1;
    }
  }
  cv::Mat overlap;
  cv::bitwise_and(green_mask, white_mask, overlap);
  if (cv::countNonZero(overlap) != 0) {
    std::cerr << "green and white masks selected the same synthetic pixel\n";
    return 1;
  }
  return 0;
}
