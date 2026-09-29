"""Map obstacle detections to grid cells for the current school competition.

The fixed ID is a single config value to fill in when the organizer announces
the common obstacle position. Until then, the measured ROI profile is valid
only for start area 1 (grid ID 4), where its photos were captured.
"""

import json
import os

import cv2
import numpy as np

from map_model import OBSTACLE_CANDIDATES_13
from serial_protocol import grid_cell


class ObstacleLocator:
    def __init__(self, config_path, start_id):
        with open(config_path, encoding="utf-8") as stream:
            config = json.load(stream)
        self.expected_count = int(config["expected_obstacle_count"])
        self.minimum_frames = int(config["minimum_detection_frames"])
        self.minimum_share = float(config["minimum_winner_share"])
        if self.expected_count != 1 or self.minimum_frames < 1:
            raise ValueError("INVALID_SCHOOL_OBSTACLE_PROFILE")
        if not 0.5 < self.minimum_share <= 1.0:
            raise ValueError("INVALID_MINIMUM_WINNER_SHARE")

        fixed_id = config.get("fixed_obstacle_id")
        self.fixed_cell = None
        self.polygons = {}
        if fixed_id is not None:
            self.fixed_cell = grid_cell(int(fixed_id))
            if self.fixed_cell not in OBSTACLE_CANDIDATES_13:
                raise ValueError("FIXED_OBSTACLE_ID_NOT_CANDIDATE")
            self.source = "fixed_obstacle_id"
            return

        roi_start_id = config.get("roi_start_id")
        if roi_start_id is None or int(roi_start_id) != start_id:
            raise ValueError("ROI_PROFILE_MISSING_FOR_START_%s" % start_id)
        roi_path = os.path.join(os.path.dirname(config_path), config["roi_file"])
        with open(roi_path, encoding="utf-8") as stream:
            roi_data = json.load(stream)
        for angle in (0, 45, 90):
            rows = roi_data.get("%ddeg" % angle)
            if not rows:
                raise ValueError("ROI_ANGLE_MISSING_%s" % angle)
            polygons = []
            for row in rows:
                label = int(row["label"])
                if not 1 <= label <= len(OBSTACLE_CANDIDATES_13):
                    raise ValueError("ROI_LABEL_INVALID_%s" % label)
                polygon = np.asarray(row["pts"], dtype=np.float32)
                if polygon.shape != (4, 2) or not cv2.isContourConvex(polygon):
                    raise ValueError("ROI_POLYGON_INVALID_%s" % label)
                polygons.append((OBSTACLE_CANDIDATES_13[label - 1], polygon))
            self.polygons[angle] = polygons
        self.source = "roi_start_%s" % start_id

    def locate(self, angle, x, y):
        """Return one candidate cell, or None for a point outside calibrated ROIs."""
        if self.fixed_cell is not None:
            return self.fixed_cell
        hits = [
            cell for cell, polygon in self.polygons[angle]
            if cv2.pointPolygonTest(polygon, (float(x), float(y)), False) >= 0
        ]
        if len(hits) > 1:
            raise ValueError("ROI_OVERLAP_AT_DETECTION")
        return hits[0] if hits else None
