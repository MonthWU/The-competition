import subprocess
import re
import cv2

def get_max_fps(device):
    """使用 v4l2-ctl 查询摄像头支持的最大帧率"""
    try:
        # 调用 v4l2-ctl 列出支持的格式和帧率
        result = subprocess.run(
            ["v4l2-ctl", "--list-formats-ext", "-d", f"/dev/{device}"],
            capture_output=True, text=True, check=True
        )
        output = result.stdout
        
        # 使用正则表达式匹配最大帧率
        fps_matches = re.findall(r"Interval: .*\((\d+\.\d+) fps\)", output)
        if not fps_matches:
            print(f"未能找到 {device} 的帧率信息")
            return None
        
        # 获取最高帧率
        max_fps = max(float(fps) for fps in fps_matches)
        return max_fps

    except subprocess.CalledProcessError:
        print(f"无法访问设备 {device}")
        return None
    
def find_camera(dev_nodes=["video0", "video2"]):
    return sorted(dev_nodes, key=lambda x: get_max_fps(x), reverse=True)

# 查询 video0 和 video2 的最大帧率
fps0 = get_max_fps("video0")
fps2 = get_max_fps("video2")

print(f"video0 最大帧率: {fps0} FPS")
print(f"video2 最大帧率: {fps2} FPS")

print(find_camera())

# # 根据最大帧率分配 cap_obj 和 cap_qrc
# cap_obj, cap_qrc = None, None

# if fps0 and fps0 >= 240:
#     cap_qrc = cv2.VideoCapture(0)
#     cap_obj = cv2.VideoCapture(2) if fps2 and fps2 >= 120 else None
# elif fps2 and fps2 >= 240:
#     cap_qrc = cv2.VideoCapture(2)
#     cap_obj = cv2.VideoCapture(0) if fps0 and fps0 >= 120 else None
# else:
#     if fps0 and fps0 >= 120:
#         cap_obj = cv2.VideoCapture(0)
#     if fps2 and fps2 >= 120:
#         cap_obj = cv2.VideoCapture(2)

# # 检查摄像头是否正确打开
# if cap_obj and cap_obj.isOpened():
#     print("cap_obj 已打开")
# if cap_qrc and cap_qrc.isOpened():
#     print("cap_qrc 已打开")

# # 释放摄像头资源
# if cap_obj:
#     cap_obj.release()
# if cap_qrc:
#     cap_qrc.release()
