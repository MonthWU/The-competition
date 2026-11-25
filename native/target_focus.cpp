// Purpose: detect the nearest real circle in a bounded grayscale ROI using OpenCV C++.
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

/* ***
   函数名字：car26_nearest_circle
   函数作用：从灰度图的种子附近寻找最近的有效圆，并返回真实圆心与半径。
   参数意义与范围：
     gray：连续width*height字节的灰度图；非空指针，每像素uint8范围[0,255]。
     width：灰度图宽度；正整数，精定位当前为1920像素。
     height：灰度图高度；正整数，精定位当前为1080像素。
     seed_x：搜索种子的水平坐标；有限数值，范围[0,width)。
     seed_y：搜索种子的垂直坐标；有限数值，范围[0,height)。
     search：搜索ROI半边长；整数[4,540]像素，当前配置180。
     min_radius：允许的最小圆半径；整数[1,max_radius]像素。
     max_radius：允许的最大圆半径；整数[min_radius,search]像素。
     canny：边缘检测高阈值；有限数值[1,1000]，当前配置100。
     votes：霍夫圆候选阈值；有限数值[1,1000]，当前配置18。
     out：可写的三个double空间；非空指针，成功时依次输出x、y、半径。
   返回范围：1表示找到圆，0表示未找到，-1表示输入非法或处理失败。
*** */
extern "C" int car26_nearest_circle(const unsigned char* gray, int width, int height,
    double seed_x, double seed_y, int search, int min_radius, int max_radius,
    double canny, double votes, double* out) noexcept {
    if (!gray || !out || width <= 0 || height <= 0 || search < 4 || min_radius < 1 ||
        max_radius < min_radius || canny <= 0 || votes <= 0 || !std::isfinite(seed_x) ||
        !std::isfinite(seed_y) || seed_x < 0 || seed_y < 0 || seed_x >= width || seed_y >= height)
        return -1;
    try {
        const int x = std::max(0, static_cast<int>(seed_x) - search);
        const int y = std::max(0, static_cast<int>(seed_y) - search);
        const int right = std::min(width, static_cast<int>(seed_x) + search + 1);
        const int bottom = std::min(height, static_cast<int>(seed_y) + search + 1);
        cv::Mat image(height, width, CV_8UC1, const_cast<unsigned char*>(gray));
        cv::Mat smooth;
        cv::medianBlur(image(cv::Rect(x, y, right-x, bottom-y)), smooth, 5);
        cv::Mat edges;
        cv::Canny(smooth, edges, canny/2, canny);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges.clone(), contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        double best = std::numeric_limits<double>::infinity();
        // Prefer supported contours; a Hough vote alone may create a spurious center nearer the seed.
        for (const auto& contour : contours) {
            if (contour.size() < 12) continue;
            const auto ellipse = cv::fitEllipse(contour);
            const double minor = std::min(ellipse.size.width, ellipse.size.height)/2;
            const double major = std::max(ellipse.size.width, ellipse.size.height)/2;
            if (minor < min_radius || major > max_radius || minor/major < .65) continue;
            const double area_fraction = std::abs(cv::contourArea(contour))/(CV_PI*minor*major);
            if (area_fraction > 1.5) continue;
            const double perimeter = cv::arcLength(contour, true);
            if (perimeter <= 0 || (area_fraction >= .45 &&
                4*CV_PI*std::abs(cv::contourArea(contour))/(perimeter*perimeter) < .4)) continue;
            const double angle=ellipse.angle*CV_PI/180;
            const double ca=std::cos(angle), sa=std::sin(angle);
            double residual=0;
            std::vector<double> angles;
            for (const auto& point : contour) {
                const double px=point.x-ellipse.center.x, py=point.y-ellipse.center.y;
                const double u=(ca*px+sa*py)/(ellipse.size.width/2);
                const double v=(-sa*px+ca*py)/(ellipse.size.height/2);
                const double error=std::hypot(u,v)-1;
                residual+=error*error;
                angles.push_back(std::atan2(v,u));
            }
            if (std::sqrt(residual/contour.size()) > .1) continue;
            std::sort(angles.begin(),angles.end());
            double largest_gap=angles.front()+2*CV_PI-angles.back();
            for (std::size_t i=1;i<angles.size();++i)
                largest_gap=std::max(largest_gap,angles[i]-angles[i-1]);
            if (2*CV_PI-largest_gap < .35*2*CV_PI) continue;
            const double cx = ellipse.center.x+x, cy = ellipse.center.y+y;
            const double distance = std::hypot(cx-seed_x, cy-seed_y);
            if (distance <= search && distance < best) {
                best=distance; out[0]=cx; out[1]=cy; out[2]=(major+minor)/2;
            }
        }
        if (std::isfinite(best)) return 1;
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(smooth, circles, cv::HOUGH_GRADIENT, 1, 8,
                        canny, votes, min_radius, max_radius);
        for (const auto& circle : circles) {
            int supported=0, visible=0;
            for (int i=0; i<72; ++i) {
                const double angle=2*CV_PI*i/72;
                const int px=static_cast<int>(std::lround(circle[0]+circle[2]*std::cos(angle)));
                const int py=static_cast<int>(std::lround(circle[1]+circle[2]*std::sin(angle)));
                if (px<3 || py<3 || px>=edges.cols-3 || py>=edges.rows-3) continue;
                ++visible;
                if (cv::countNonZero(edges(cv::Rect(px-3,py-3,7,7)))>0) ++supported;
            }
            if (visible<60 || supported < .85*visible) continue;
            const double cx = circle[0] + x, cy = circle[1] + y;
            const double distance = std::hypot(cx-seed_x, cy-seed_y);
            if (distance <= search && distance < best) {
                best = distance;
                out[0] = cx; out[1] = cy; out[2] = circle[2];
            }
        }
        return std::isfinite(best) ? 1 : 0;
    } catch (...) { return -1; }
}
