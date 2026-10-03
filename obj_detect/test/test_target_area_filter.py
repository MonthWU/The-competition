"""Check area boundaries, invalid boxes, and class-independent behavior."""

import pytest
from ai_msgs.msg import Roi, Target

from obj_detect.target_area_filter import filter_targets_by_area


def target(name, width, height):
    item = Target(type=name)
    roi = Roi()
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
