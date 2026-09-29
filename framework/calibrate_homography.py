#!/usr/bin/env python3
"""单应性标定工具 —— 图像像素坐标 → 5×5 网格坐标。

背景
----
`obstacle_detector.pixel_to_grid()` 需要单应矩阵 H（像素 → 网格）才能工作。
H 由本工具离线标定生成，默认存到：
    /root/dev_ws/appli/framework/dnn/obstacle_homography.npy

标定原理
--------
现场把障碍物（或任何醒目标记）依次放到**已知网格点**上，从画面读出其像素坐标，
得到 N 组对应点（N ≥ 4，建议 6~8 组），用 cv2.findHomography 解出 H。

坐标约定（务必一致，否则映射会错）
----------------------------------
  · 像素坐标：图像左上角为原点，x 向右、y 向下（单位像素）
  · 网格坐标：(row, col) —— row 向下 0~4，col 向右 0~4（见 map_model.py）
  · H 把 (x_px, y_px) → (row, col)；对应 pixel_to_grid 里 warped[0]=row、warped[1]=col

用法
----
  ① 抓帧 + 叠加像素网格（现场据此读出像素坐标）
     python3 calibrate_homography.py capture [输出jpg]

  ② 求解并保存 H（至少 4 组，格式 row,col:x_px,y_px）
     python3 calibrate_homography.py solve "0,0:320,180" "0,4:1000,190" "4,0:300,600" "4,4:1010,610"

  ③ 验证（用已保存的 H 反算网格坐标）
     python3 calibrate_homography.py verify "2,2:640,360"

  ④ 查看当前标定状态
     python3 calibrate_homography.py status
"""
import os
import subprocess
import sys

import cv2
import numpy as np

H_PATH = "/root/dev_ws/appli/framework/dnn/obstacle_homography.npy"
DEV = "/dev/v4l/by-id/usb-DCXIN_DCXIN_Camera_01.00.000-video-index0"
CAP_W, CAP_H = 1280, 720          # 与 prescan.launch.py 的 SCAN_WIDTH/HEIGHT 保持一致


def _apply_dcxin_brightness(dev):
    """采图前修正 DCXIN 亮度/增益。

    背景：该机固件无真正的 Auto 曝光（只支持 1/3），且 3（光圈优先）在 UVC 上
    空转，画面亮度实际只由 brightness/gain 决定；出厂 50/0 明显偏暗
    （实测均值 68.6）。此处与 prescan.launch.py 的节点参数取一致值。
    """
    for ctrl, val in (("auto_exposure", 3), ("brightness", 128), ("gain", 48)):
        try:
            subprocess.run(["v4l2-ctl", "-d", dev, "--set-ctrl=%s=%s" % (ctrl, val)],
                           timeout=5, capture_output=True)
        except Exception:
            pass


def _parse_pairs(argv):
    """解析 "row,col:x,y" → (image_pts, map_pts)。"""
    image_pts, map_pts = [], []
    for a in argv:
        try:
            gc, px = a.split(":")
            r, c = [float(v) for v in gc.split(",")]
            x, y = [float(v) for v in px.split(",")]
        except Exception:
            print("格式错误: %r（应为 row,col:x,y，例如 \"0,0:320,180\"）" % a)
            sys.exit(1)
        map_pts.append([r, c])        # 注意：row 在前（与 pixel_to_grid 的 warped 顺序一致）
        image_pts.append([x, y])
    return np.array(image_pts, dtype=np.float32), np.array(map_pts, dtype=np.float32)


def cmd_capture(out="/tmp/calib_frame.jpg"):
    _apply_dcxin_brightness(DEV)      # 采图前先提亮（否则画面偏暗）
    cap = cv2.VideoCapture(DEV)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_H)
    if not cap.isOpened():
        print("无法打开相机:", DEV)
        return 1
    frame = None
    for _ in range(15):
        ok, f = cap.read()
        if ok:
            frame = f
    cap.release()
    if frame is None:
        print("抓帧失败")
        return 1
    step = 80
    h, w = frame.shape[:2]
    for x in range(0, w, step):
        cv2.line(frame, (x, 0), (x, h), (0, 140, 255), 1)
        cv2.putText(frame, str(x), (x + 3, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 140, 255), 1)
    for y in range(0, h, step):
        cv2.line(frame, (0, y), (w, y), (0, 140, 255), 1)
        cv2.putText(frame, str(y), (3, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 140, 255), 1)
    cv2.putText(frame, "grid=%dpx  size=%dx%d" % (step, w, h), (w - 300, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 1)
    cv2.imwrite(out, frame)
    gray_mean = float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
    print("已保存: %s（%dx%d，网格 %d px，亮度均值 %.1f）" % (out, w, h, step, gray_mean))
    if gray_mean < 40:
        print("⚠️ 画面偏暗（均值 %.1f）—— 若明显发黑，检查 DCXIN 是否被其它程序占用" % gray_mean)
    return 0


def cmd_solve(pairs):
    if len(pairs) < 4:
        print("至少需要 4 组对应点（当前 %d 组）" % len(pairs))
        return 1
    img_pts, map_pts = _parse_pairs(pairs)
    H, mask = cv2.findHomography(img_pts, map_pts, method=cv2.RANSAC)
    if H is None:
        print("求解失败：请检查点是否共线或输入有误")
        return 1
    # 重投影误差（用 H 把像素点映射回网格，与实际网格点比对）
    proj = cv2.perspectiveTransform(img_pts.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(proj - map_pts, axis=1)
    print("=== 标定结果 ===")
    print("输入点数: %d（RANSAC 内点 %d）" % (len(pairs), int(mask.sum()) if mask is not None else -1))
    print("每点重投影误差（网格单位，越小越好）:")
    for i, (p, e) in enumerate(zip(map_pts, err)):
        print("  #%d 网格(%.1f,%.1f)  误差 %.3f 格" % (i + 1, p[0], p[1], e))
    print("平均误差: %.3f 格  最大: %.3f 格" % (err.mean(), err.max()))
    os.makedirs(os.path.dirname(H_PATH), exist_ok=True)
    np.save(H_PATH, H)
    print("已保存单应矩阵 -> %s" % H_PATH)
    print("提示：误差 >0.5 格时建议重新取点（检查网格坐标/像素坐标是否对应错位）")
    return 0


def cmd_verify(pairs):
    if not os.path.exists(H_PATH):
        print("尚无标定文件:", H_PATH)
        return 1
    H = np.load(H_PATH)
    img_pts, map_pts = _parse_pairs(pairs)
    proj = cv2.perspectiveTransform(img_pts.reshape(-1, 1, 2), H).reshape(-1, 2)
    print("=== 验证（像素 → 网格）===")
    for i, (p, q) in enumerate(zip(img_pts, proj)):
        print("  像素(%.0f,%.0f) -> 预测网格(row=%.2f, col=%.2f)" % (p[0], p[1], q[0], q[1]))
        if i < len(map_pts):
            print("       实际网格(row=%.1f, col=%.1f)  误差 %.3f 格"
                  % (map_pts[i][0], map_pts[i][1],
                     float(np.linalg.norm(q - map_pts[i]))))
    return 0


def cmd_status():
    if os.path.exists(H_PATH):
        H = np.load(H_PATH)
        print("已标定 ✅  %s" % H_PATH)
        print("H =\n%s" % np.array2string(H, precision=6))
    else:
        print("未标定 ❌  缺少 %s" % H_PATH)
        print("→ pixel_to_grid() 目前返回占位 (2,2)，请先执行 capture + solve")
    return 0


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    cmd = sys.argv[1]
    args = sys.argv[2:]
    if cmd == "capture":
        return cmd_capture(args[0] if args else "/tmp/calib_frame.jpg")
    if cmd == "solve":
        if not args:
            print("交互式输入（每行 row,col:x,y，空行结束）：")
            lines = []
            while True:
                try:
                    line = input("> ").strip()
                except EOFError:
                    break
                if not line:
                    break
                lines.append(line)
            args = lines
        return cmd_solve(args)
    if cmd == "verify":
        return cmd_verify(args)
    if cmd == "status":
        return cmd_status()
    print("未知子命令:", cmd)
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())