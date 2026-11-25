# Purpose: validate CLI inputs and apply work/debug output policy to the same business application.
import argparse
import json
import logging

from .config import load_config
from .runtime_output import runtime_output


# ***
# 函数名字：main
# 函数作用：解析启动参数并执行配置检查或启动应用。
# 参数意义与范围：
#   argv：命令行字符串序列；None表示读取当前进程参数，选项范围由ArgumentParser约束。
# ***
def main(argv=None):
    parser = argparse.ArgumentParser(description="CAR26 four-module application")
    parser.add_argument("--config", required=True)
    parser.add_argument("--mode", choices=("work", "debug"))
    parser.add_argument("--lighting", choices=("light_able", "light_unable"),
                        help="select object-camera parameters for lighting on/off; does not control the lamp")
    parser.add_argument("--check", action="store_true", help="validate configuration without opening devices")
    parser.add_argument("--check-files", action="store_true", help="also verify configured paths; implies --check")
    args = parser.parse_args(argv)
    try:
        configuration = load_config(args.config, args.mode, args.lighting)
        errors = configuration.validate(args.check_files)
    except (OSError, ValueError, TypeError, AttributeError) as error:
        errors = ["CONFIG_LOAD_FAILED:" + str(error)]
    if errors:
        print(json.dumps({"status": "CONFIG_INVALID", "errors": errors}, ensure_ascii=False, indent=2))
        return 2
    if args.check or args.check_files:
        print(json.dumps({"status": "CONFIG_VALID", "mode": configuration.mode,
                          "web_hmi_enabled": configuration.mode == 'debug' and configuration.data['hmi']['enabled'],
                          "lighting_mode": configuration.data.get("lighting_mode"),
                          "object_uvc_controls": configuration.data["cameras"]["object"].get("uvc_controls", {})}, indent=2))
        return 0
    with runtime_output(configuration.mode, configuration.directory.parent) as log_path:
        logging.basicConfig(level=logging.DEBUG if configuration.mode == "debug" else logging.INFO,
                            format="%(asctime)s %(levelname)s %(name)s %(message)s", force=True)
        logging.getLogger("car26.app").debug("mode=%s entry=%s config=%s",
            configuration.mode, configuration.data["mission"]["entry"], args.config)
        try:
            from .application import Application
            app = Application(configuration)
            if log_path is not None:
                app.telemetry.update("application", runtime_log=str(log_path))
            return app.run()
        except Exception:
            logging.getLogger("car26.app").exception("application initialization failed")
            return 1
