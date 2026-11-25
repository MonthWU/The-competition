# Purpose: compare sparse classification with the original dense math at numerical boundaries and preserve NMS.
import os
import unittest
import numpy as np
from car26.camera.candidates import NativeCandidates, select_classes
from car26.camera.detector import decode_heads


# ***
# 函数名字：dense_reference
# 函数作用：独立保留旧版全量概率与类别选择公式，作为优化一致性参考。
# 参数意义与范围：
#   logits：非空H×W×C浮点数组，允许NaN及无穷值。
#   threshold：概率筛选阈值，有限数值[0,1]。
# ***
def dense_reference(logits, threshold):
    scores=1/(1+np.exp(-np.clip(logits,-80,80)))
    classes=scores.argmax(axis=2)
    yy,xx=np.where(scores.max(axis=2)>=threshold)
    categories=classes[yy,xx]
    return yy,xx,categories,scores[yy,xx,categories]


class CandidateTests(unittest.TestCase):
    # ***
    # 函数名字：assert_selection
    # 函数作用：比较原版与优化版的候选顺序、类别、概率，不使用同一筛选实现生成期望。
    # 参数意义与范围：
    #   self：当前CandidateTests实例。
    #   logits：待验证H×W×C浮点数组。
    #   threshold：有限概率阈值[0,1]。
    #   selector：NativeCandidates实例或None。
    # ***
    def assert_selection(self, logits, threshold, selector=None):
        with np.errstate(all='ignore'):
            expected=dense_reference(logits,threshold)
            actual=select_classes(logits,threshold,selector)
        for before,after in zip(expected,actual):
            np.testing.assert_array_equal(after,before)

    # ***
    # 函数名字：test_random_layouts_and_float_types
    # 函数作用：验证连续HWC、CHW转置和负跨度视图在多浮点类型及阈值下与旧版一致。
    # 参数意义与范围：self：当前CandidateTests实例。
    # ***
    def test_random_layouts_and_float_types(self):
        rng=np.random.default_rng(260624)
        for dtype in (np.float16,np.float32,np.float64):
            values=rng.normal(-3,12,(7,11,9)).astype(dtype)
            layouts=[values,np.ascontiguousarray(values.transpose(2,0,1)).transpose(1,2,0),values[::-1]]
            for array in layouts:
                for threshold in (0.,1e-30,.25,.5,.999999,1.):
                    with self.subTest(dtype=dtype,threshold=threshold,strides=array.strides):
                        self.assert_selection(array,threshold)

    # ***
    # 函数名字：test_rounding_ties_saturation_nan_and_infinity
    # 函数作用：验证饱和概率并列仍选择首类别，NaN整格拒绝，无穷值沿用旧剪裁行为。
    # 参数意义与范围：self：当前CandidateTests实例。
    # ***
    def test_rounding_ties_saturation_nan_and_infinity(self):
        for dtype in (np.float16,np.float32,np.float64):
            values=np.array([[[60,80,70],[float('-inf')]*3,[float('nan'),80,0]],
                             [[float('inf'),80,0],[-90,-100,-80],[-1.0986123,-1.0986122,-30]]],dtype=dtype)
            for threshold in (0.,.25,.5,1.):
                self.assert_selection(values,threshold)
            yy,xx,categories,scores=select_classes(values,.9)
            self.assertEqual(categories[(yy==0)&(xx==0)].tolist(),[0])

    # ***
    # 函数名字：test_exact_probability_threshold_neighbors
    # 函数作用：验证原版实际概率及相邻可表示阈值不会因逆变换舍入丢掉候选。
    # 参数意义与范围：self：当前CandidateTests实例。
    # ***
    def test_exact_probability_threshold_neighbors(self):
        for dtype in (np.float16,np.float32,np.float64):
            for raw in (-80.,-20.,-1.0986123,0.,3.,15.,80.):
                values=np.array([[[raw,raw-1,raw-2]]],dtype=dtype)
                with np.errstate(all='ignore'):
                    probability=dense_reference(values,0)[3][0]
                for threshold in (float(probability),float(np.nextafter(probability,dtype(0))),
                                  float(np.nextafter(probability,dtype(1)))):
                    self.assert_selection(values,threshold)

    # ***
    # 函数名字：test_same_class_nms_and_cross_class_overlap
    # 函数作用：以独立几何期望验证同类重叠框仍被NMS抑制，跨类别框仍保留。
    # 参数意义与范围：self：当前CandidateTests实例。
    # ***
    def test_same_class_nms_and_cross_class_overlap(self):
        outputs=[]
        for side,bin_index in ((4,2),(2,1),(1,1)):
            classes=np.full((1,side,side,2),-30,np.float32)
            boxes=np.full((1,side,side,4,16),-30,np.float32)
            boxes[...,bin_index]=30
            outputs.extend([classes,boxes.reshape(1,side,side,64)])
        outputs[0][0,1,1,0]=6
        outputs[0][0,1,2,0]=5
        outputs[2][0,0,0,1]=4
        result=decode_heads(outputs,['redblock','blackblock'],64,64,.75,.45,input_size=32)
        self.assertEqual([(d.label,d.x1,d.y1,d.x2,d.y2) for d in result],
                         [('redblock',0,0,56,56),('blackblock',0,0,48,48)])

    # ***
    # 函数名字：test_native_matches_dense_reference
    # 函数作用：在板端验证C++跨布局筛选与原版在边界及NaN输入上保持一致。
    # 参数意义与范围：self：当前CandidateTests实例。
    # ***
    @unittest.skipUnless(os.environ.get('CAR26_CANDIDATE_LIBRARY'),'candidate library is built on the board')
    def test_native_matches_dense_reference(self):
        native=NativeCandidates(os.environ['CAR26_CANDIDATE_LIBRARY'])
        values=np.random.default_rng(83).normal(-6,15,(13,17,9)).astype(np.float32)
        values[0,0]=[30,80,60,1,2,3,4,5,6]
        values[0,1,0]=np.nan
        values[0,2]=[-np.inf]*9
        for array in (values,np.ascontiguousarray(values.transpose(2,0,1)).transpose(1,2,0),values[::-1]):
            for threshold in (0.,1e-30,.25,.5,1.):
                self.assert_selection(array,threshold,native)


if __name__=='__main__':
    unittest.main()
