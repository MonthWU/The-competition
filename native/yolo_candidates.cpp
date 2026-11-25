// Purpose: select possible YOLO cells by raw logits before sparse probability evaluation.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

/* ***
   函数名字：car26_candidate_cells
   函数作用：按宽松logit阈值筛选候选网格，保持行优先顺序并拒绝含NaN的网格。
   参数意义与范围：
     logits：有效float32三维HWC视图的首元素地址，内存由调用方持有。
     height：网格行数，整数[1,640]。
     width：网格列数，整数[1,640]。
     classes：类别数，整数[1,1024]。
     row_stride：相邻行的float元素跨度，正整数。
     col_stride：相邻列的float元素跨度，正整数。
     class_stride：相邻类别的float元素跨度，正整数。
     cutoff：保守的原始logit下界，有限数值或负无穷，不得为NaN。
     output：可写int32数组，容量至少为height*width。
     capacity：output元素容量，整数且不小于height*width。
   返回范围：非负候选数；-1表示输入非法。
*** */
extern "C" int car26_candidate_cells(const float* logits, int height, int width,
    int classes, std::ptrdiff_t row_stride, std::ptrdiff_t col_stride,
    std::ptrdiff_t class_stride, double cutoff, std::int32_t* output,
    int capacity) noexcept {
    if (!logits || !output || height < 1 || height > 640 || width < 1 || width > 640 ||
        classes < 1 || classes > 1024 || row_stride <= 0 || col_stride <= 0 ||
        class_stride <= 0 || std::isnan(cutoff) || capacity < height*width)
        return -1;
    int count = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const float* cell = logits + y*row_stride + x*col_stride;
            float best = -std::numeric_limits<float>::infinity();
            bool invalid = false;
            for (int c = 0; c < classes; ++c) {
                const float value = cell[c*class_stride];
                if (std::isnan(value)) { invalid = true; break; }
                if (value > best) best = value;
            }
            if (!invalid && static_cast<double>(best) >= cutoff)
                output[count++] = y*width+x;
        }
    }
    return count;
}
