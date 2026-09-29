"""Validate the four-part school competition QR task code."""

import re


_FORMAT = re.compile(r"([1-6]{3})\+([1-3]{3})\+([1-6]{3})\+([1-3]{3})\Z")


def is_valid_task_code(value):
    if not isinstance(value, str):
        return False
    match = _FORMAT.fullmatch(value)
    if match is None:
        return False
    first_colors, first_places, second_colors, second_places = match.groups()
    return (
        len(set(first_colors)) == 3
        and len(set(second_colors)) == 3
        and set(first_colors) == set(second_colors)
        and set(first_places) == {"1", "2", "3"}
        and set(second_places) == {"1", "2", "3"}
    )
