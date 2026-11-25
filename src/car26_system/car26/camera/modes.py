# Purpose: resolve independent block/target capture profiles for the single object-camera owner.


# ***
# 函数名字：object_capture_settings
# 函数作用：解析物块、圆盘或标靶采集配置，旧文件的圆盘模式回退到标靶采集设置。
# 参数意义与范围：
#   settings：cameras.object配置字典，含公共设备设置及可选recognition_modes。
#   mode：采集档位，只允许block（物块）、disc（圆盘）或target（标靶）。
# ***
def object_capture_settings(settings, mode):
    if mode not in ("block", "disc", "target"):
        raise ValueError("INVALID_OBJECT_RECOGNITION_MODE")
    profiles = settings.get("recognition_modes", {})
    profile = profiles.get(mode, profiles.get('target', {}) if mode == 'disc' else {})
    return dict(settings, width=profile.get("width", settings.get("capture_width", 1920)),
                height=profile.get("height", settings.get("capture_height", 1080)),
                fps=profile.get("fps", settings.get("capture_fps", 30)))


# ***
# 函数名字：capture_signature
# 函数作用：提取尺寸、格式、帧率和缓冲数量，相同配置仅切换算法。
# 参数意义与范围：
#   settings：解析后的采集配置，包含width、height、fps和fourcc，可选buffer_count为1～4。
# ***
def capture_signature(settings):
    return tuple(settings[key] for key in ("width", "height", "fps", "fourcc")) + (settings.get('buffer_count', 1),)
