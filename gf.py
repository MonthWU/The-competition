import subprocess
import time

def check_qrc_processes():
    """检查是否有以 'qrc' 开头的进程"""
    try:
        # 使用 ps -a 和 grep 查找 qrc* 的进程
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
        
        # 检查输出是否包含 'qrc'
        if 'qrc' not in result.stdout:
            return False  # 没有找到 qrc 相关进程
        return True
    except Exception as e:
        print(f"Error checking processes: {e}")
        return False

def main():
    # 死循环检查 qrc* 进程是否存在
    while True:
        if not check_qrc_processes():
            print("No qrc* processes found. Exiting loop.")
            break
        print("qrc* processes are running. Checking again...")
        
        # 延迟检查，以减少系统资源占用
        # time.sleep(2)

if __name__ == "__main__":
    main()
