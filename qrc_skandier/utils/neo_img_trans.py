from sensor_msgs.msg import Image

import cv2 as cv
import numpy as np


def cv2ros(Img):
    rimg = cv.imencode(".jpeg", Img)[1].tobytes()
    return rimg


def ros2cv(RImg: Image):
    if RImg is None:
        return None
    np_arr = np.frombuffer(RImg.data, np.uint8)
    cv_image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
    return cv_image
