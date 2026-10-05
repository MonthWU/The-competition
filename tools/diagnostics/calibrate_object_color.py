"""Calibrate from white paper covering the ROI; save and apply without training.

Run while the object camera is publishing. Default ROI is the central 30% of
the frame; ensure the white reference covers that region completely.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import CompressedImage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--roi', type=int, nargs=4, metavar=('X','Y','W','H'))
    parser.add_argument('--environment', default='current lighting')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[2]/'framework/object_camera_color.json')
    args = parser.parse_args()
    samples = []
    observed_roi = None
    rclpy.init()
    node = rclpy.create_node('calibrate_object_color_once')

    def receive(message):
        nonlocal observed_roi
        if len(samples) >= 24:
            return
        image = cv2.imdecode(np.frombuffer(message.data,np.uint8),cv2.IMREAD_COLOR)
        if image is None:
            return
        height,width = image.shape[:2]
        x,y,w,h = args.roi or [int(width*.35),int(height*.35),int(width*.3),int(height*.3)]
        if min(x,y)<0 or min(w,h)<=0 or x+w>width or y+h>height:
            raise ValueError('REFERENCE_ROI_OUT_OF_IMAGE')
        observed_roi = [x,y,w,h]
        samples.append(np.median(image[y:y+h,x:x+w,::-1].reshape(-1,3),axis=0))

    subscription = node.create_subscription(CompressedImage,'/image',receive,qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+12
        while len(samples)<24 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.2)
        if len(samples)<12:
            raise RuntimeError('NO_OBJECT_CAMERA_REFERENCE: start the object camera first')
        rgb = np.median(samples,axis=0)
        if rgb.min()<50 or rgb.max()>248:
            raise ValueError('REFERENCE_TOO_DARK_OR_CLIPPED: use an evenly lit white reference')
        gains = rgb.min()/rgb
        if gains.min()<.5:
            raise ValueError('REFERENCE_TOO_COLORED: white paper must cover the whole ROI')
        client = node.create_client(SetParameters,'/obj_dnn/set_parameters')
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('OBJECT_DETECTOR_NOT_RUNNING: profile has not been changed')
        request = SetParameters.Request()
        request.parameters = [Parameter('color_balance_rgb',value=gains.tolist()).to_parameter_msg()]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=3)
        if not future.done() or not future.result() or not all(item.successful for item in future.result().results):
            raise RuntimeError('COLOR_BALANCE_PARAMETER_REJECTED: profile has not been changed')
        profile = {'camera':'LRCP AR0234','environment':args.environment,'brightness':0,
                   'calibrated_at':datetime.now().astimezone().isoformat(),
                   'method':'neutral white reference; R/G/B order','reference_roi':observed_roi,
                   'reference_rgb':rgb.tolist(),'rgb_gains':gains.tolist(),'sample_count':len(samples)}
        if args.output.exists():
            stamp = datetime.now().strftime('%Y%m%dT%H%M%S%f')
            backup = args.output.with_name(args.output.stem+'.'+stamp+'.backup.json')
            backup.write_bytes(args.output.read_bytes())
        temporary = args.output.with_suffix('.tmp.json')
        temporary.write_text(json.dumps(profile,indent=2),encoding='utf-8')
        temporary.replace(args.output)
        print(json.dumps({'status':'COLOR_BALANCE_APPLIED_AND_SAVED','profile':str(args.output),**profile}),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
