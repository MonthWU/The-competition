"""Check area boundaries, invalid boxes, and class-independent behavior."""

import pytest
from ai_msgs.msg import Roi, Target

from obj_detect.target_area_filter import (
    filter_targets_by_area,
    filter_targets_by_y_center,
)


def target(name, width, height):
    item = Target(type=name)
    roi = Roi()
    roi.rect.width = width
    roi.rect.height = height
    item.rois = [roi]
    return item


def located_target(name, x_offset, y_offset, width, height):
    item = Target(type=name)
    roi = Roi()
    roi.rect.x_offset = x_offset
    roi.rect.y_offset = y_offset
    roi.rect.width = width
    roi.rect.height = height
    item.rois = [roi]
    return item


def test_area_boundary_applies_to_blocks_and_markers():
    targets = [
        target("black1", 1, 1999),
        target("red1", 40, 50),
        target("targetOne", 50, 40),
        target("blue2", 41, 50),
    ]
    assert filter_targets_by_area(targets, 2000) == targets[1:]


def test_zero_disables_size_gate_but_rejects_invalid_boxes():
    tiny = target("black1", 1, 1)
    assert filter_targets_by_area(
        [tiny, target("red1", 0, 100), Target(type="targetTwo")], 0
    ) == [tiny]


def test_empty_input_remains_empty():
    assert filter_targets_by_area([], 2000) == []


def test_negative_threshold_is_rejected():
    with pytest.raises(ValueError):
        filter_targets_by_area([], -1)


def test_bottom_half_centers_are_dropped():
    targets = [
        located_target("black1", 100, 260, 120, 100),  # center y=310 -> 车体区
        located_target("red1", 0, 190, 100, 100),      # center y=240 -> 压线保留
        located_target("targetOne", 0, 20, 184, 172),  # center y=106 -> 正常
    ]
    assert filter_targets_by_y_center(targets, 240) == targets[1:]


def test_y_gate_passes_through_valid_and_skips_empty_rois():
    valid = located_target("green1", 0, 0, 50, 50)
    assert filter_targets_by_y_center([valid, Target(type="targetTwo")], 25) == [valid]


def test_y_ratio_one_keeps_full_frame():
    bottom = located_target("black1", 0, 380, 100, 100)  # center y=430 <= 480
    assert filter_targets_by_y_center([bottom], 480) == [bottom]


def test_negative_max_center_y_is_rejected():
    with pytest.raises(ValueError):
        filter_targets_by_y_center([], -1)
