# Purpose: keep work-mode runtime output in a file while debug uses the terminal, including native writes.
from contextlib import contextmanager, redirect_stdout, redirect_stderr
import os
from pathlib import Path
import sys
import time


# ***
# 函数名字：flush_native_output
# 函数作用：在切换输出描述符前排空Linux原生标准IO缓冲以防日志泄漏到终端。
# 参数意义与范围：
#   无。
# ***
def flush_native_output():
    if os.name == "posix":
        import ctypes
        library = ctypes.CDLL(None)
        flush = library.fflush
        flush.argtypes = [ctypes.c_void_p]
        flush.restype = ctypes.c_int
        flush(None)


# ***
# 函数名字：runtime_output
# 函数作用：将work运行输出留档，并让debug直接显示终端诊断。
# 参数意义与范围：
#   mode：运行模式字符串；仅允许work或debug。
#   project_directory：项目根目录；必须是可解析且允许创建log/runtime的目录路径。
# ***
@contextmanager
def runtime_output(mode, project_directory):
    if mode not in ("work", "debug"):
        raise ValueError("INVALID_MODE")
    if mode == "debug":
        yield None
        return
    directory = Path(project_directory) / "log" / "runtime"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / ("work-" + time.strftime("%Y%m%d-%H%M%S") + "-" + str(os.getpid()) + ".log")
    sys.stdout.flush()
    sys.stderr.flush()
    flush_native_output()
    saved_out, saved_err = os.dup(1), os.dup(2)
    try:
        with path.open("a", encoding="utf-8", buffering=1) as output:
            os.dup2(output.fileno(), 1)
            os.dup2(output.fileno(), 2)
            with redirect_stdout(output), redirect_stderr(output):
                yield path
            output.flush()
    finally:
        flush_native_output()
        os.dup2(saved_out, 1)
        os.dup2(saved_err, 2)
        os.close(saved_out)
        os.close(saved_err)
