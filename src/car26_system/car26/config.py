# Purpose: validate source configuration without opening devices or loading inference models.

import copy
import json
import math
import os
from pathlib import Path

from .communication.protocol import CANDIDATE_IDS
from .camera.modes import object_capture_settings


# ***
# 函数名字：_merge
# 函数作用：递归合并配置字典并保持输入对象不变。
# 参数意义与范围：
#   base：基础配置字典；范围为JSON兼容的嵌套键值。
#   override：覆盖配置字典；范围为JSON兼容的嵌套键值。
# ***
def _merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


class Configuration:
    # ***
    # 函数名字：__init__
    # 函数作用：初始化当前类的依赖、配置和内部状态。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   data：配置字典或协议字节；具体类型与范围按本函数所属类的输入协议确定。
    #   directory：配置相对路径的基准目录；范围为本地可解析的目录路径。
    # ***
    def __init__(self, data, directory):
        self.data = data
        self.directory = Path(directory).resolve()

    # ***
    # 函数名字：path
    # 函数作用：将配置中的相对或环境变量路径解析为绝对路径。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   value：绝对、相对或带环境变量的路径字符串；须可由Path解析。
    # ***
    def path(self, value):
        p = Path(os.path.expandvars(value)).expanduser()
        return p.resolve() if p.is_absolute() else (self.directory / p).resolve()

    # ***
    # 函数名字：mode
    # 函数作用：返回当前配置的work或debug运行模式。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    # ***
    @property
    def mode(self):
        return self.data["mode"]

    # ***
    # 函数名字：validate
    # 函数作用：在不开设备的条件下检查配置结构、参数范围及可选文件路径。
    # 参数意义与范围：
    #   self：当前所属类实例；范围为该类已初始化的对象。
    #   check_files：是否检查依赖路径存在；布尔值True或False。
    # ***
    def validate(self, check_files=False):
        errors = []

        # ***
        # 函数名字：require
        # 函数作用：将未满足的配置约束加入错误列表。
        # 参数意义与范围：
        #   ok：约束是否满足；布尔表达式的结果。
        #   name：失败约束的错误标识；范围为非空字符串。
        # ***
        def require(ok, name):
            if not ok:
                errors.append(name)

        # ***
        # 函数名字：number
        # 函数作用：判断参数是否是指定闭区间内的有限数值。
        # 参数意义与范围：
        #   value：待检查值；可为任意对象，只有非布尔有限数值才可能通过。
        #   low：允许数值的闭区间下界；有限数值且不大于high。
        #   high：允许数值的闭区间上界；有限数值且不小于low。
        # ***
        def number(value, low, high):
            return (not isinstance(value, bool) and isinstance(value, (int, float))
                    and math.isfinite(value) and low <= value <= high)

        try:
            require(self.mode in ("work", "debug"), "INVALID_MODE")
            from .camera.disc import validate_disc_centering
            errors.extend(validate_disc_centering(self,check_files))
            communication = self.data["communication"]
            require(communication["transport"] == "serial", "MCU_CAN_PROTOCOL_NOT_DEFINED")
            require(bool(communication["device"]), "MCU_DEVICE_REQUIRED")
            require(communication["baudrate"] == 115200, "MCU_PROTOCOL_REQUIRES_115200_8N1")
            require(number(communication["write_timeout_s"], 0.01, 30), "INVALID_WRITE_TIMEOUT")
            mission = self.data["mission"]
            require(mission["entry"] in ("full", "qr", "object"), "INVALID_MISSION_ENTRY")
            require(type(mission["object_enabled"]) is bool, "INVALID_OBJECT_ENABLED")
            require(number(mission["shot_timeout_s"], 0.1, 600), "INVALID_SHOT_TIMEOUT")
            require(number(mission["collect_timeout_s"], 0.1, 600), "INVALID_COLLECT_TIMEOUT")
            require(number(mission["sensor_timeout_s"], 0.1, 120), "INVALID_SENSOR_TIMEOUT")
            require(type(mission["minimum_frames"]) is int and mission["minimum_frames"] >= 1,
                    "INVALID_MINIMUM_FRAMES")
            require(number(mission["minimum_winner_share"], 0.500001, 1), "INVALID_WINNER_SHARE")
            source = mission["prescan_source"]
            require(source in ("camera", "radar"), "INVALID_PRESCAN_SOURCE")
            roles = ([] if mission["entry"] == "object" else ["qr"]) + (["object"] if mission["object_enabled"] else [])
            require(mission["entry"] != "object" or mission["object_enabled"], "OBJECT_ENTRY_REQUIRES_OBJECTS")
            if mission["entry"] == "full" and source == "camera":
                roles.append("prescan")
            for role in roles:
                c = self.data["cameras"][role]
                require(bool(c["device"]), "CAMERA_DEVICE_REQUIRED:" + role)
                require(all(type(c[n]) is int and c[n] > 0 for n in ("width", "height")),
                        "INVALID_CAMERA_DIMENSIONS:" + role)
                require(number(c["fps"], 1, 500), "INVALID_CAMERA_FPS:" + role)
                require(type(c.get('buffer_count', 1)) is int and 1 <= c.get('buffer_count', 1) <= 4,
                        'INVALID_CAMERA_BUFFER_COUNT:' + role)
                if role == "object":
                    profiles = c.get("recognition_modes", {})
                    require(isinstance(profiles, dict) and not set(profiles) - {"block", "disc", "target"},
                            "INVALID_OBJECT_RECOGNITION_MODES")
                    for name in ("block", "disc", "target"):
                        profile = profiles.get(name, {})
                        require(isinstance(profile, dict) and not set(profile) - {"width", "height", "fps"},
                                "INVALID_OBJECT_RECOGNITION_PROFILE:" + name)
                        resolved = object_capture_settings(c, name)
                        require(all(type(resolved[k]) is int and 16 <= resolved[k] <= 8192
                                    for k in ("width", "height")), "INVALID_OBJECT_CAPTURE_DIMENSIONS:" + name)
                        require(number(resolved["fps"], 1, 30), "INVALID_OBJECT_CAPTURE_FPS")
                require(c["fourcc"] in ("MJPG", "YUYV"), "INVALID_CAMERA_FOURCC:" + role)
                if role == "qr":
                    qr_roi = c.get("roi")
                    require(qr_roi is None or (isinstance(qr_roi, list) and len(qr_roi) == 4
                            and all(type(v) is int for v in qr_roi)
                            and qr_roi[0] >= 0 and qr_roi[1] >= 0 and qr_roi[2] > 0 and qr_roi[3] > 0
                            and qr_roi[0] + qr_roi[2] <= c["width"]
                            and qr_roi[1] + qr_roi[3] <= c["height"]), "INVALID_QR_ROI")
                if role != "qr":
                    model = self.data["models"][c["model"]]
                    require(number(model["score_threshold"], 0, 1), "INVALID_SCORE_THRESHOLD:" + role)
                    require(number(model["nms_threshold"], 0, 1), "INVALID_NMS_THRESHOLD:" + role)
                    require(type(model.get('overlap_capture', False)) is bool,
                            'INVALID_INFERENCE_OVERLAP:' + role)
                    gains = model["rgb_gains"]
                    require(len(gains) == 3 and all(number(g, 0.001, 10) for g in gains),
                            "INVALID_COLOR_GAINS:" + role)
                    if check_files:
                        for key in ("file", "classes"):
                            require(self.path(model[key]).is_file(), "MODEL_FILE_MISSING:" + model[key])
                        if model.get('candidate_library'):
                            require(self.path(model['candidate_library']).is_file(),
                                    'MODEL_CANDIDATE_LIBRARY_MISSING:'+role)
                        if model.get('class_map') is not None:
                            from .camera.classmap import load_model_labels
                            try:
                                load_model_labels(self,model)
                            except (OSError,ValueError,TypeError) as error:
                                errors.append('MODEL_CLASS_MAP_INVALID:'+str(error))
                if check_files and c["device"]:
                    require(Path(c["device"]).exists(), "CAMERA_DEVICE_MISSING:" + role)
            filters = self.data["camera_processing"]
            focus = self.data.get("target_focus", {})
            require(focus.get('trigger', 'auto') in ('auto', 'command'), 'INVALID_FOCUS_TRIGGER')
            require(number(focus.get('seed_max_age_s', 1.0), .05, 5), 'INVALID_FOCUS_SEED_MAX_AGE')
            require(type(focus.get("enabled", False)) is bool, "INVALID_FOCUS_ENABLED")
            if focus.get("enabled", False):
                require(number(focus["angle_threshold_deg"], 0, 45), "INVALID_FOCUS_ANGLE")
                require(number(focus["fps"], 1, 30), "INVALID_FOCUS_FPS")
                require(number(focus["lost_timeout_s"], .1, 30), "INVALID_FOCUS_LOST_TIMEOUT")
                require(number(focus["maximum_duration_s"], focus["lost_timeout_s"], 600), "INVALID_FOCUS_DURATION")
                require(type(focus["settle_frames"]) is int and 1 <= focus["settle_frames"] <= 30, "INVALID_FOCUS_SETTLE")
                require(all(type(focus[k]) is int and 1 <= focus[k] <= 540 for k in
                    ("search_radius_px", "minimum_radius_px", "maximum_radius_px")), "INVALID_FOCUS_RADIUS")
                require(focus["minimum_radius_px"] <= focus["maximum_radius_px"] <= focus["search_radius_px"], "INVALID_FOCUS_RADIUS_ORDER")
                require(all(number(focus[k], 1, 1000) for k in ("canny_threshold", "circle_threshold")), "INVALID_FOCUS_HOUGH")
                transform = focus["mode_transform"]
                require(len(transform)==4 and all(number(v,-10,10) for v in transform)
                        and transform[0]>0 and transform[1]>0, "INVALID_FOCUS_TRANSFORM")
                require(bool(focus["native_library"]), "FOCUS_LIBRARY_REQUIRED")
                if check_files:
                    require(self.path(focus["native_library"]).is_file(), "FOCUS_LIBRARY_MISSING")
            require(number(filters["minimum_area_px"], 0, 100000000), "INVALID_MINIMUM_AREA")
            require(number(filters["mark23_overlap"], 0, 1), "INVALID_MARK_OVERLAP")
            require(number(filters["mark_block_overlap"], 0, 1), "INVALID_MARK_BLOCK_OVERLAP")
            reference = filters["reference_px"]
            require(reference is None or (len(reference) == 2 and all(number(v, 0, 65535) for v in reference)), "INVALID_TARGET_REFERENCE")
            roi = filters["block_roi"]
            require(roi is None or (len(roi) == 3 and all(number(v, 0, 65535) for v in roi)
                                   and roi[0] <= roi[1]), "INVALID_BLOCK_ROI")
            devices = [self.data["cameras"][role]["device"] for role in roles if self.data["cameras"][role]["device"]]
            require(len(devices) == len(set(devices)), "CAMERA_ROLE_DEVICE_CONFLICT")
            radar = self.data["radar"]
            require(type(radar["enabled"]) is bool, "INVALID_RADAR_ENABLED")
            if radar["enabled"]:
                require(bool(radar["device"]), "RADAR_DEVICE_REQUIRED")
                require(radar["baudrate"] == 460800, "C1_REQUIRES_460800_8N1")
                require(radar["device"] != communication["device"], "RADAR_AND_MCU_DEVICE_CONFLICT")
                require(number(radar["frequency_hz"], 8, 12), "C1_FREQUENCY_OUT_OF_RANGE")
                require(number(radar["stream_timeout_s"], 0.1, 30), "INVALID_RADAR_TIMEOUT")
                require(number(radar["min_range_m"], 0.05, 12)
                        and number(radar["max_range_m"], radar["min_range_m"], 12),
                        "INVALID_RADAR_RANGE")
                require(number(radar["cluster_gap_m"], 0.001, 5), "INVALID_CLUSTER_GAP")
                require(type(radar["minimum_cluster_points"]) is int
                        and radar["minimum_cluster_points"] >= 1, "INVALID_CLUSTER_POINTS")
                require(len(radar["extrinsic"]) == 3
                        and all(number(v, -10000, 10000) for v in radar["extrinsic"]),
                        "INVALID_RADAR_EXTRINSIC")
                require(type(radar["extrinsic_calibrated"]) is bool, "INVALID_CALIBRATION_FLAG")
                require(type(radar["minimum_quality"]) is int and 0 <= radar["minimum_quality"] <= 63,
                        "INVALID_RADAR_QUALITY")
                if check_files and radar["device"]:
                    require(Path(radar["device"]).exists(), "RADAR_DEVICE_MISSING")
            if mission["entry"] == "full" and source == "radar":
                require(radar["enabled"], "RADAR_PRESCAN_REQUIRES_RADAR")
                require(radar["extrinsic_calibrated"], "RADAR_EXTRINSIC_CALIBRATION_REQUIRED")
                require(bool(self.data["radar_mapping"]["targets_by_start"]),
                        "RADAR_MAP_CALIBRATION_REQUIRED")
                mapping = self.data["radar_mapping"]
                require(number(mapping["match_radius_m"], .001, 5), "INVALID_MAP_RADIUS")
                require(number(mapping["minimum_width_m"], 0, 20) and number(mapping["maximum_width_m"], mapping["minimum_width_m"], 20), "INVALID_MAP_OBSTACLE_WIDTH")
                for start, targets in mapping["targets_by_start"].items():
                    require(start in ("4", "24"), "INVALID_MAP_START")
                    require(bool(targets) and len({t["id"] for t in targets}) == len(targets), "INVALID_MAP_TARGETS")
                    require(all(type(t["id"]) is int and t["id"] in CANDIDATE_IDS and number(t["x_m"], -1000, 1000) and number(t["y_m"], -1000, 1000) for t in targets), "INVALID_MAP_COORDINATES")
                    headings = mapping["heading_deg_by_start"].get(start, [])
                    require(len(headings) == 3 and all(number(h, -3600, 3600) for h in headings), "RADAR_SHOT_HEADING_CALIBRATION_REQUIRED:" + start)
            if mission["entry"] == "full" and source == "camera":
                require(bool(self.data["camera_mapping"]["start_ids"]), "CAMERA_ROI_START_REQUIRED")
                require(self.data["camera_mapping"]["start_ids"] == [4], "LEGACY_CAMERA_ROI_ONLY_CALIBRATED_FOR_START_4")
                require(self.data["camera_mapping"]["calibration_size"] == [1280, 720], "LEGACY_CAMERA_ROI_REQUIRES_1280_720")
                require(number(self.data["camera_mapping"]["score_threshold"], 0, 1), "INVALID_MAP_SCORE_THRESHOLD")
                if check_files:
                    require(self.path(self.data["camera_mapping"]["roi_file"]).is_file(),
                            "CAMERA_ROI_FILE_MISSING")
            hmi = self.data["hmi"]
            require(type(hmi["enabled"]) is bool, "INVALID_HMI_ENABLED")
            require(type(hmi["port"]) is int and 1 <= hmi["port"] <= 65535, "INVALID_HMI_PORT")
            require(number(hmi["preview_fps"], 0.1, 60), "INVALID_PREVIEW_FPS")
            require(type(hmi["max_stream_clients"]) is int and hmi["max_stream_clients"] >= 1,
                    "INVALID_STREAM_CLIENT_LIMIT")
            recording = self.data["recording"]
            require(type(recording["enabled"]) is bool and bool(recording["directory"]), "INVALID_RECORDING_CONFIG")
            require(number(recording["maximum_seconds"], 1, 3600), "INVALID_RECORDING_LIMIT")
            if check_files:
                require(Path(communication["device"]).exists(), "MCU_DEVICE_MISSING")
            from .navigation import validate_navigation
            errors.extend(validate_navigation(self))
        except (KeyError, TypeError, ValueError, AttributeError) as error:
            errors.append("INVALID_CONFIG_STRUCTURE:" + str(error))
        return errors


# ***
# 函数名字：load_config
# 函数作用：读取继承配置并应用运行模式与物块相机补光参数，保持原文件不变。
# 参数意义与范围：
#   filename：配置JSON文件路径；范围为可读取且继承链无环的文件。
#   mode：运行模式覆盖值；范围为None、work或debug。
#   lighting：相机补光参数覆盖值；None使用配置默认值，或light_able、light_unable。
# ***
def load_config(filename, mode=None, lighting=None):
    filename = Path(filename).resolve()

    # ***
    # 函数名字：read
    # 函数作用：从串口传输层读取一段有界字节数据。
    # 参数意义与范围：
    #   path：待读取配置文件路径；范围为存在且可解码为UTF-8 JSON的文件。
    #   seen：当前继承链已访问路径集合；范围为Path对象集合，用于拒绝循环继承。
    # ***
    def read(path, seen):
        if path in seen:
            raise ValueError("CONFIG_INHERITANCE_CYCLE")
        data = json.loads(path.read_text(encoding="utf-8"))
        base = data.pop("extends", None)
        return _merge(read((path.parent / base).resolve(), seen | {path}), data) if base else data

    data = read(filename, set())
    if mode is not None:
        data["mode"] = mode
    selected = lighting if lighting is not None else data.get("lighting_mode")
    if selected is not None:
        if selected not in ("light_able", "light_unable"):
            raise ValueError("INVALID_LIGHTING_MODE")
        camera = data["cameras"]["object"]
        profile = camera.get("lighting_profiles", {}).get(selected)
        required = {"auto_exposure", "exposure_time_absolute", "brightness", "gain"}
        if (not isinstance(profile, dict) or set(profile) != required
                or any(type(v) is not int for v in profile.values())
                or profile["auto_exposure"] != 1
                or not 1 <= profile["exposure_time_absolute"] <= 10000
                or not 0 <= profile["brightness"] <= 255
                or not 0 <= profile["gain"] <= 255):
            raise ValueError("INVALID_LIGHTING_PROFILE:" + selected)
        camera["uvc_controls"] = _merge(camera.get("uvc_controls", {}), profile)
        data["lighting_mode"] = selected
    return Configuration(data, filename.parent)
