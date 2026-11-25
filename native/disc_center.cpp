// Purpose: fit the large white disc from visible outer arcs, rejecting small holes and unsupported ellipses.
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace {
struct Model { cv::RotatedRect ellipse; double a, b, ca, sa; };

/* ***
函数名字：larger_contour
函数作用：按轮廓面积从大到小排列候选白色区域。
参数意义与范围：a、b：OpenCV轮廓点序列，坐标为处理图像素。
*** */
bool larger_contour(const std::vector<cv::Point>& a,const std::vector<cv::Point>& b) {
    return cv::contourArea(a)>cv::contourArea(b);
}

/* ***
函数名字：model
函数作用：检查椭圆尺度和轴比并缓存坐标变换。
参数意义与范围：e：拟合椭圆，坐标为处理图像素；size：处理图正宽高；p：17项有效配置；m：输出模型。
*** */
bool model(const cv::RotatedRect& e, cv::Size size, const double* p, Model& m) {
    const double a=e.size.width/2, b=e.size.height/2;
    if (!std::isfinite(e.center.x) || !std::isfinite(e.center.y) || !std::isfinite(a+b+e.angle)
        || std::min(a,b)*2 < p[5]*std::min(size.width,size.height)
        || std::max(a,b)*2 > p[6]*std::max(size.width,size.height)
        || std::min(a,b)/std::max(a,b) < p[7]
        || e.center.x < -size.width || e.center.x > 2*size.width
        || e.center.y < -size.height || e.center.y > 2*size.height) return false;
    const double angle=e.angle*CV_PI/180;
    m={e,a,b,std::cos(angle),std::sin(angle)};
    return true;
}

/* ***
函数名字：residual
函数作用：计算点到椭圆的近似径向残差及归一化弧角。
参数意义与范围：point：处理图边缘点；m：有效椭圆模型；angle：输出[-pi,pi]弧度。
*** */
double residual(cv::Point2f point, const Model& m, double& angle) {
    const double x=point.x-m.ellipse.center.x, y=point.y-m.ellipse.center.y;
    const double u=(m.ca*x+m.sa*y)/m.a, v=(-m.sa*x+m.ca*y)/m.b;
    angle=std::atan2(v,u);
    return std::abs(std::hypot(u,v)-1)*std::min(m.a,m.b);
}

/* ***
函数名字：inliers
函数作用：提取给定残差以内的边缘支持点，拒绝噪声及遮挡轮廓。
参数意义与范围：points：最多1200个处理图边缘点；m：有效模型；threshold：正像素阈值。
*** */
std::vector<cv::Point2f> inliers(const std::vector<cv::Point2f>& points, const Model& m, double threshold) {
    std::vector<cv::Point2f> result;
    for (auto point:points) { double angle; if (residual(point,m,angle)<=threshold) result.push_back(point); }
    return result;
}

/* ***
函数名字：quality
函数作用：核对可见轮廓支持、弧覆盖率、内部白色比例及残差，避免将遮挡或夹具作为大圆盘。
参数意义与范围：m：有效椭圆；points：当前支持点；mask：处理图白色掩膜；distance：到非图像边框轮廓的距离图；component：填充后的当前连通外轮廓掩膜；p：17项配置；out：依次输出残差均方根、覆盖角度、可见支持率、内部白色比例。
*** */
bool quality(const Model& m, const std::vector<cv::Point2f>& points, const cv::Mat& mask,
             const cv::Mat& distance, const cv::Mat& component, const double* p, double* out) {
    std::array<bool,72> bins{}; double sum=0;
    for (auto point:points) {
        double angle; const double error=residual(point,m,angle); sum+=error*error;
        bins[std::min(71,static_cast<int>((angle+CV_PI)/(2*CV_PI)*72))]=true;
    }
    const double coverage=std::count(bins.begin(),bins.end(),true)*5.;
    if (points.size()<p[9] || coverage<p[10]) return false;
    int component_points=0,enclosed=0;
    for(int y=4;y<mask.rows-4;y+=8) for(int x=4;x<mask.cols-4;x+=8) {
        if(!component.at<unsigned char>(y,x)) continue;
        ++component_points;
        const double dx=x-m.ellipse.center.x,dy=y-m.ellipse.center.y;
        const double u=(m.ca*dx+m.sa*dy)/m.a,v=(-m.sa*dx+m.ca*dy)/m.b;
        if(std::hypot(u,v)<=1+p[8]/std::min(m.a,m.b)) ++enclosed;
    }
    if(component_points<20 || double(enclosed)/component_points<p[16]) return false;
    int visible=0,supported=0,interior=0,white=0;
    for (int i=0;i<180;++i) {
        const double t=2*CV_PI*i/180, u=m.a*std::cos(t),v=m.b*std::sin(t);
        const int x=std::lround(m.ellipse.center.x+m.ca*u-m.sa*v);
        const int y=std::lround(m.ellipse.center.y+m.sa*u+m.ca*v);
        if (x<4 || y<4 || x>=mask.cols-4 || y>=mask.rows-4) continue;
        ++visible;
        if (distance.at<float>(y,x)<=p[8]+1) ++supported;
    }
    for (int iy=-8;iy<=8;++iy) for (int ix=-8;ix<=8;++ix) {
        if (ix*ix+iy*iy>64) continue;
        const double u=m.a*ix/10, v=m.b*iy/10;
        const int x=std::lround(m.ellipse.center.x+m.ca*u-m.sa*v);
        const int y=std::lround(m.ellipse.center.y+m.sa*u+m.ca*v);
        if (x<0 || y<0 || x>=mask.cols || y>=mask.rows) continue;
        ++interior; if (mask.at<unsigned char>(y,x)) ++white;
    }
    if (visible<30 || interior<20) return false;
    const double support=double(supported)/visible, fraction=double(white)/interior;
    if (support<p[11] || fraction<p[12]) return false;
    out[0]=std::sqrt(sum/points.size()); out[1]=coverage; out[2]=support; out[3]=fraction;
    return true;
}
}

/* ***
函数名字：car26_disc_center
函数作用：在BGR图像中分割白色大圆盘，以确定性RANSAC拟合可见外缘，失败不输出旧圆心。
参数意义与范围：
  bgr：连续width*height*3字节BGR图像，各通道[0,255]。
  width、height：原图宽高，整数[32,8192]。
  p：17个double配置，依次为处理宽[160,1920]、饱和度上限[0,255]、亮度下限[0,255]、奇数形态核[1,15]、连通区域占比(0,1)、最小直径/短边(0,2]、最大直径/长边[0.5,4]、最小轴比[0.3,1]、内点阈值[0.5,10]像素、最少内点[20,1200]、覆盖角[60,300]度、可见支持率[0.3,1]、内部白色比例[0.3,1]、迭代数[20,1000]、明亮分支饱和度上限[0,255]、明亮分支V下限[0,255]、轮廓包围比例[0.7,1]。
  count：配置数量，必须17。
  out：12个double输出，依次为原图中心x/y、椭圆轴径w/h、角度、原图残差像素、覆盖角度、可见支持率、内部白色比例、内点数、分割分支0或1、该分支实际亮度阈值。
返回：1检测成功，0证据不足，-1输入或运行异常。
*** */
extern "C" int car26_disc_center(const unsigned char* bgr,int width,int height,
                                 const double* p,int count,double* out) noexcept {
    if (!bgr || !p || !out || count!=17 || width<32 || height<32 || width>8192 || height>8192) return -1;
    const double lo[]={160,0,0,1,.001,.05,.5,.3,.5,20,60,.3,.3,20,0,0,.7};
    const double hi[]={1920,255,255,15,.9,2,4,1,10,1200,300,1,1,1000,255,255,1};
    for(int i=0;i<17;++i) if(!std::isfinite(p[i]) || p[i]<lo[i] || p[i]>hi[i]) return -1;
    for(int i:{0,1,2,3,9,13,14,15}) if(std::floor(p[i])!=p[i]) return -1;
    if (int(p[3])%2!=1 || p[5]>p[6]) return -1;
    try {
        cv::Mat original(height,width,CV_8UC3,const_cast<unsigned char*>(bgr)),image;
        const double scale=std::min(1.,p[0]/width);
        cv::resize(original,image,cv::Size(std::lround(width*scale),std::lround(height*scale)),0,0,cv::INTER_AREA);
        const double sx=double(width)/image.cols, sy=double(height)/image.rows;
        cv::Mat hsv,mask;
        cv::GaussianBlur(image,image,cv::Size(5,5),0);
        cv::cvtColor(image,hsv,cv::COLOR_BGR2HSV);
        cv::Mat value,binary;
        cv::extractChannel(hsv,value,2);
        const double bright_threshold=std::max(p[15],cv::threshold(value,binary,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU));
        cv::RNG rng(2026); double best=0;
        for (int profile=0;profile<2;++profile) {
        const double threshold=profile==0?p[2]:bright_threshold;
        cv::inRange(hsv,cv::Scalar(0,0,threshold),cv::Scalar(179,profile==0?p[1]:p[14],255),mask);
        cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(int(p[3]),int(p[3]))));
        cv::morphologyEx(mask,mask,cv::MORPH_OPEN,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(int(p[3]),int(p[3]))));
        cv::Mat boundary,distance;
        cv::Canny(mask,boundary,50,150);
        boundary.rowRange(0,4).setTo(0); boundary.rowRange(boundary.rows-4,boundary.rows).setTo(0);
        boundary.colRange(0,4).setTo(0); boundary.colRange(boundary.cols-4,boundary.cols).setTo(0);
        cv::distanceTransform(255-boundary,distance,cv::DIST_L2,3);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(),contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
        std::sort(contours.begin(),contours.end(),larger_contour);
        for (std::size_t ci=0;ci<std::min<std::size_t>(3,contours.size());++ci) {
            if(cv::contourArea(contours[ci])<p[4]*image.total()) continue;
            cv::Mat component=cv::Mat::zeros(mask.size(),CV_8UC1);
            cv::drawContours(component,contours,int(ci),cv::Scalar(255),cv::FILLED);
            std::vector<cv::Point2f> points,valid;
            for(auto point:contours[ci]) if(point.x>3 && point.y>3 && point.x<image.cols-4 && point.y<image.rows-4) valid.push_back(point);
            const std::size_t amount=std::min<std::size_t>(1200,valid.size());
            for(std::size_t i=0;i<amount;++i) points.push_back(valid[i*valid.size()/amount]);
            if(points.size()<p[9]) continue;
            for(int iteration=0;iteration<int(p[13]);++iteration) {
                std::vector<cv::Point2f> sample; std::vector<int> indices;
                while(sample.size()<5) {
                    int index=rng.uniform(0,int(points.size()));
                    if(std::find(indices.begin(),indices.end(),index)==indices.end()) {indices.push_back(index);sample.push_back(points[index]);}
                }
                Model m; cv::RotatedRect ellipse;
                try {ellipse=cv::fitEllipse(sample);} catch(const cv::Exception&) {continue;}
                if(!model(ellipse,image.size(),p,m)) continue;
                auto selected=inliers(points,m,p[8]);
                if(selected.size()<p[9] || selected.size()<=best) continue;
                bool good=true;
                for(int refinement=0;refinement<3;++refinement) {
                    if(selected.size()<p[9]) {good=false;break;}
                    ellipse=cv::fitEllipse(selected);
                    if(!model(ellipse,image.size(),p,m)) {good=false;break;}
                    selected=inliers(points,m,p[8]);
                }
                double q[4];
                if(!good || selected.size()<=best || !quality(m,selected,mask,distance,component,p,q)) continue;
                best=selected.size();
                out[0]=m.ellipse.center.x*sx; out[1]=m.ellipse.center.y*sy;
                out[2]=m.ellipse.size.width*sx; out[3]=m.ellipse.size.height*sy;
                out[4]=m.ellipse.angle; out[5]=q[0]*std::max(sx,sy);
                out[6]=q[1];out[7]=q[2];out[8]=q[3];out[9]=best;
                out[10]=profile;out[11]=threshold;
            }
        }
        }
        return best>0?1:0;
    } catch (...) { return -1; }
}
