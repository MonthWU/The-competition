#include "puzzle_perception_node/polygon_refinement.hpp"

#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

int main()
{
  cv::Mat mask(180, 220, CV_8UC1, cv::Scalar(0));
  const std::vector<cv::Point> expected{{30, 35}, {185, 45}, {170, 145}, {45, 155}};
  cv::fillConvexPoly(mask, expected, cv::Scalar(255));
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  if (contours.size() != 1U) {
    std::cerr << "synthetic contour creation failed\n";
    return 1;
  }
  puzzle_perception_node::PolygonRefinementParams params;
  params.epsilon_min_px = 1.0;
  params.epsilon_max_px = 6.0;
  params.epsilon_steps = 6;
  params.maximum_rms_px = 1.0;
  params.maximum_residual_px = 2.0;
  const auto result = puzzle_perception_node::refine_polygon_from_contour(contours[0], params);
  if (!result.valid || result.vertices.size() != 4U) {
    std::cerr << "dense contour was not refined to four fitted intersections: " <<
      result.status << '\n';
    return 1;
  }
  for (const auto & vertex : result.vertices) {
    double nearest = 1e9;
    for (const auto & target : expected) {
      nearest = std::min(nearest, cv::norm(vertex - cv::Point2f(target)));
    }
    if (nearest > 2.0) {
      std::cerr << "refined vertex exceeded tolerance\n";
      return 1;
    }
  }

  cv::Mat obtuse_mask(180, 240, CV_8UC1, cv::Scalar(0));
  const std::vector<cv::Point> obtuse_expected{
    {25, 35}, {120, 35}, {180, 45}, {200, 150}, {35, 150}};
  cv::fillConvexPoly(obtuse_mask, obtuse_expected, cv::Scalar(255));
  contours.clear();
  cv::findContours(obtuse_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  if (contours.size() != 1U) {
    std::cerr << "obtuse contour creation failed\n";
    return 1;
  }
  params.epsilon_min_px = 1.0;
  params.epsilon_max_px = 3.0;
  params.epsilon_steps = 3;
  params.max_vertices = 5;
  const auto obtuse_result = puzzle_perception_node::refine_polygon_from_contour(
    contours[0], params);
  if (!obtuse_result.valid || obtuse_result.vertices.size() != obtuse_expected.size()) {
    std::cerr << "real obtuse polygon contour was not preserved: " <<
      obtuse_result.status << " vertices=" << obtuse_result.vertices.size() << '\n';
    return 1;
  }
  return 0;
}
