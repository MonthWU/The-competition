"""mission_dispatcher —— 任务编排节点（骨架）。

职责：一键式串联 预扫描 → 障碍识别 → 道路判定 → 路径规划 → 串口下发 → 拉起原任务。
串口分时：预扫描/规划阶段独占 ttyS1；原任务阶段由 obj_serial 独占。

计划接口：
    run_prescan()          # 调 camera_pan + map_scanner 完成 0/45/90 三帧采集
    detect_obstacles()     # 调 obstacle_detector 识别黑色几何体
    judge_roads()          # 调 road_judge 输出障碍节点集
    plan_path()            # 调 path_planner 生成避障路径
    send_path()            # 串口下发路径指令给下位机
    launch_original()      # subprocess 拉起 run_all.launch.py（原任务）
"""

class MissionDispatcher:
    def __init__(self):
        raise NotImplementedError("TODO: 初始化各子模块（camera_pan/map_scanner/...）")

    def run_prescan(self):
        """阶段0：斜视摄像头 0/45/90 三帧采集。"""
        raise NotImplementedError("TODO: 调 camera_pan.goto(angle) + map_scanner.capture()")

    def detect_obstacles(self):
        """阶段1：识别黑色几何体，输出障碍像素坐标列表。"""
        raise NotImplementedError("TODO: 调 obstacle_detector.detect()")

    def judge_roads(self):
        """阶段2：误差圈 + 候选点判定，输出障碍节点集。"""
        raise NotImplementedError("TODO: 调 road_judge.judge()")

    def plan_path(self):
        """阶段3：BFS 避障路径规划。"""
        raise NotImplementedError("TODO: 调 path_planner.plan()")

    def send_path(self, path):
        """串口下发路径指令（协议待下位机确认）。"""
        raise NotImplementedError("TODO: ttyS1 分时写串口")

    def launch_original(self):
        """阶段4：拉起原任务 launch（原代码零改动）。"""
        raise NotImplementedError("TODO: subprocess 调 run_all.launch.py")


def main():
    disp = MissionDispatcher()
    disp.run_prescan()
    disp.detect_obstacles()
    disp.judge_roads()
    disp.plan_path()


if __name__ == "__main__":
    main()
