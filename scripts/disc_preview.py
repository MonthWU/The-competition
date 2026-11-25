# Purpose: provide white-disc center detection on the object camera without UART, YOLO or QR startup.
import argparse
import json
import logging
import signal
import socket
import threading
import time
from pathlib import Path

from car26.camera.disc import DiscFinder, draw_disc
from car26.camera.module import CameraModule
from car26.camera.modes import object_capture_settings
from car26.config import load_config
from car26.hmi.module import HmiModule
from car26.telemetry import Telemetry


# ***
# 函数名字：main
# 函数作用：检查圆盘检测依赖、处理单张图像或运行有界物块相机圆心预览。
# 参数意义与范围：无；命令行参数由argparse校验，所有默认路径相对当前项目。
# ***
def main():
    import cv2
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description='白色大圆盘外缘圆心测试；不发送UART。')
    parser.add_argument('--config', type=Path, default=root/'config/disc.json', help='圆盘阈值JSON')
    parser.add_argument('--check', action='store_true', help='只检查配置、依赖及相机路径')
    parser.add_argument('--image', type=Path, help='单张图片离线检测，不开设备')
    parser.add_argument('--output', type=Path, help='离线叠加图输出，需配合--image')
    parser.add_argument('--maximum-seconds', type=float, default=3600, help='运行时长，1～3600秒')
    parser.add_argument('--mode', choices=('work','debug'), default='work', help='debug在终端显示诊断')
    args = parser.parse_args()
    if not 1 <= args.maximum_seconds <= 3600:
        parser.error('--maximum-seconds必须为1～3600秒')
    if args.output and not args.image:
        parser.error('--output需要--image')
    finder = DiscFinder(root/'native/lib/libcar26_disc.so', args.config)
    if args.image:
        image = cv2.imread(str(args.image))
        if image is None:
            raise RuntimeError('DISC_IMAGE_UNREADABLE')
        result = finder.find(image)
        if args.output:
            draw_disc(image, result)
            if not cv2.imwrite(str(args.output),image):
                raise RuntimeError('DISC_OUTPUT_FAILED')
        print(json.dumps({'detected':result is not None,'disc':result},ensure_ascii=False))
        return
    config = load_config(root/'config/board-object.json', args.mode)
    settings = object_capture_settings(config.data['cameras']['object'],'block')
    if not Path(settings['device']).exists():
        raise RuntimeError('DISC_CAMERA_NOT_FOUND')
    if not config.data['hmi']['enabled']:
        raise RuntimeError('DISC_HMI_DISABLED')
    if args.check:
        print('DISC_CHECK_OK')
        return
    host,port = config.data['hmi']['host'],config.data['hmi']['port']
    if args.mode == 'debug':
        with socket.socket() as probe:
            if probe.connect_ex(('127.0.0.1' if host=='0.0.0.0' else host,port))==0:
                raise RuntimeError(f'DISC_PORT_BUSY:{port}; stop car26-object.service or existing preview first')
    logs = root/'log/disc'
    logs.mkdir(parents=True,exist_ok=True)
    handlers = [logging.FileHandler(logs/'preview.log',encoding='utf-8')]
    if args.mode=='debug':
        handlers.append(logging.StreamHandler())
    logging.basicConfig(level=logging.INFO,handlers=handlers,format='%(asctime)s %(message)s')
    telemetry = Telemetry()
    camera = CameraModule(config,telemetry)
    camera.role = 'object'
    hmi = HmiModule(config.data['hmi'],telemetry,mode=args.mode)
    stopped = threading.Event()

    # ***
    # 函数名字：request_stop
    # 函数作用：将SIGINT或SIGTERM转换为预览停止请求。
    # 参数意义与范围：number：注册的操作系统信号编号；frame：当前调用栈帧或None，不读取。
    # ***
    def request_stop(number, frame):
        stopped.set()

    for number in (signal.SIGINT,signal.SIGTERM):
        signal.signal(number,request_stop)
    cap = None
    deadline = time.monotonic()+args.maximum_seconds
    sequence,valid = 0,0
    start = time.monotonic()
    try:
        cap = camera._open_capture(settings)
        ok,image = cap.read()
        if not ok:
            raise RuntimeError('DISC_CAMERA_READ_FAILED')
        camera._stream_controls(settings)
        for _ in range(3):
            ok,image = cap.read()
            if not ok:
                raise RuntimeError('DISC_CAMERA_SETTLE_FAILED')
        hmi.start()
        telemetry.update('application',state='disc_preview',mode='white disc center')
        telemetry.update('mission',state='DISC_PREVIEW',uart_enabled=False)
        while not stopped.is_set() and time.monotonic()<deadline:
            timestamp = time.monotonic()
            ok,image = cap.read()
            if not ok:
                raise RuntimeError('DISC_CAMERA_READ_FAILED')
            height,width = image.shape[:2]
            if (width,height)!=(settings['width'],settings['height']):
                raise RuntimeError('DISC_CAMERA_DIMENSIONS_MISMATCH')
            begin = time.monotonic()
            result = finder.find(image)
            processing_ms = (time.monotonic()-begin)*1000
            sequence += 1
            valid += result is not None
            if args.mode == 'debug':
                draw_disc(image,result)
                encoded,jpeg = cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,85])
                if not encoded:
                    raise RuntimeError('DISC_JPEG_FAILED')
                telemetry.image('object',jpeg.tobytes())
            telemetry.update('camera',state='ready',role='object',vision_mode='disc',sequence=sequence,
                             width=width,height=height,capture_width=width,capture_height=height,
                             frame_timestamp=timestamp,processing_ms=round(processing_ms,2),
                             processed_fps=round(sequence/max(time.monotonic()-start,.001),2),
                             disc_detected=result is not None,disc=result,valid_frames=valid,uart_enabled=False)
            if sequence==1:
                logging.info('DISC_PREVIEW_READY port=%s size=%sx%s',port,width,height)
            stopped.wait(max(0,1/config.data['hmi']['preview_fps']-(time.monotonic()-timestamp)))
    except Exception as error:
        telemetry.update('camera',state='error',disc_detected=False,disc=None)
        logging.exception('DISC_PREVIEW_FAILED: %s',error)
        raise
    finally:
        if cap is not None:
            cap.release()
        hmi.close()
        logging.info('DISC_PREVIEW_STOPPED')


if __name__ == '__main__':
    main()
