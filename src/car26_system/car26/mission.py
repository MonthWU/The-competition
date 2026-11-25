# Purpose: deterministic mission state machine returning effects to the application owner.

from collections import Counter

from .communication.protocol import map_frame, qr_frame


class Mission:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   settings：当前模块配置字典；键、类型及数值范围由Configuration.validate约束。
    #   supports_start：起点标定检查回调；接收起点4或24并返回布尔值。
    # ***
    def __init__(self, settings, supports_start):
        self.settings = settings
        self.supports_start = supports_start
        self.state = {"full": "WAIT_START", "qr": "QR", "object": "OBJECT"}[settings["entry"]]
        self.start_id = None
        self.shots = 0
        self.deadline = None
        self.collect_started = None
        self.votes = Counter()
        self.scores = Counter()
        self.positive_frames = 0
        self.last_sequence = None
        self.last_sensor_time = None
        self.error = None

    # ***
    # 函数名字：command
    # 函数作用：根据当前任务状态处理一条下位机命令。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   command：已解析MCU命令字符串或雷达命令字节；具体取值由所属协议规定。
    #   now：当前单调时间；非负有限秒数，应与对应帧时间戳来自同一时钟。
    # ***
    def command(self, command, now):
        if self.state != "WAIT_START" and command in ("4", "24", "skip"):
            return []
        if self.state == "WAIT_START":
            if command == "skip":
                self.state = "QR"
                return [("camera", "qr")]
            if command not in ("4", "24"):
                return []
            start = int(command)
            if not self.supports_start(start):
                self.fail("MAP_START_NOT_CALIBRATED:" + command)
                return []
            self.start_id = start
            self.state = "WAIT_SHOT"
            self.deadline = now + self.settings["shot_timeout_s"]
            return [("send", b"[ack]")]
        if self.state == "WAIT_SHOT" and command == "shot":
            if now >= self.deadline:
                self.fail("SHOT_COMMAND_TIMEOUT")
                return []
            self.state = "COLLECT"
            self.collect_started = self.last_sensor_time = now
            self.deadline = now + self.settings["collect_timeout_s"]
            self.positive_frames = 0
            self.last_sequence = None
            return [("prescan", None)]
        return []

    # ***
    # 函数名字：angle
    # 函数作用：返回当前预扫描步骤对应的角度。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    @property
    def angle(self):
        return (0, 45, 90)[min(self.shots, 2)]

    # ***
    # 函数名字：observe
    # 函数作用：累计触发后新鲜且独立的观测并按门限完成扫描。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   sequence：观测序号；非负整数，同一数据帧不得重复累计。
    #   captured_at：观测采集单调时间；非负有限秒数，必须晚于本步触发且未过期。
    #   observations：障碍格号到得分的字典；格号属于候选集，得分为非负有限数值。
    #   now：当前单调时间；非负有限秒数，应与对应帧时间戳来自同一时钟。
    # ***
    def observe(self, sequence, captured_at, observations, now):
        if self.state == "COLLECT" and now >= self.deadline:
            return self.tick(now)
        if self.state != "COLLECT" or captured_at < self.collect_started or sequence == self.last_sequence:
            return []
        if now - captured_at > self.settings["sensor_timeout_s"]:
            return []
        self.last_sequence = sequence
        self.last_sensor_time = now
        if observations:
            self.positive_frames += 1
            self.votes.update(observations.keys())
            self.scores.update(observations)
        if self.positive_frames >= self.settings["minimum_frames"]:
            return self._finish_shot(now)
        return []

    # ***
    # 函数名字：tick
    # 函数作用：根据单调时间检查任务超时和扫描结束条件。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   now：当前单调时间；非负有限秒数，应与对应帧时间戳来自同一时钟。
    # ***
    def tick(self, now):
        if self.state == "COLLECT" and now - self.last_sensor_time > self.settings["sensor_timeout_s"]:
            self.fail("PRESCAN_SENSOR_STALE")
        elif self.state == "COLLECT" and now >= self.deadline:
            return self._finish_shot(now)
        elif self.state == "WAIT_SHOT" and now >= self.deadline:
            self.fail("SHOT_COMMAND_TIMEOUT")
        return []

    # ***
    # 函数名字：_finish_shot
    # 函数作用：结束当前扫描并决定下一步握手或地图输出。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   now：当前单调时间；非负有限秒数，应与对应帧时间戳来自同一时钟。
    # ***
    def _finish_shot(self, now):
        self.shots += 1
        actions = [("send", b"[ack]")]
        if self.shots < 3:
            self.state = "WAIT_SHOT"
            self.deadline = now + self.settings["shot_timeout_s"]
            return actions
        ranked = sorted(self.votes, key=lambda i: (self.votes[i], self.scores[i]), reverse=True)
        total = sum(self.votes.values())
        if not ranked or self.votes[ranked[0]] < self.settings["minimum_frames"] or self.votes[ranked[0]] / total < self.settings["minimum_winner_share"]:
            self.fail("OBSTACLE_VOTE_AMBIGUOUS")
            return actions
        actions.append(("send", map_frame([ranked[0]])))
        self.state = "QR"
        actions.append(("camera", "qr"))
        return actions

    # ***
    # 函数名字：qr
    # 函数作用：拒绝含帧尾]的结果，仅对有效二维码生成转发动作。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   text：二维码Unicode原文；只有非空且不包含]时才可进入发送状态。
    # ***
    def qr(self, text):
        if self.state != "QR" or not text or "]" in text:
            return []
        self.state = "QR_SENDING"
        return [("send", qr_frame(text))] * 4 + [("flush", None), ("qr_sent", None)]

    # ***
    # 函数名字：qr_sent
    # 函数作用：在二维码发送完成后关闭旧相机并进入物块阶段。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    def qr_sent(self):
        if self.state != "QR_SENDING":
            raise RuntimeError("UNEXPECTED_QR_COMPLETION")
        self.state = "OBJECT" if self.settings["object_enabled"] else "DONE"
        return [("camera", "object")] if self.state == "OBJECT" else [("camera_stop", None)]

    # ***
    # 函数名字：fail
    # 函数作用：将任务切换为失败状态并记录原因。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   error：失败原因；范围为非空错误标识字符串。
    # ***
    def fail(self, error):
        self.error = error
        self.state = "FAILED"
