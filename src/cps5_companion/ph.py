"""Named photo task helpers for the AUV control script."""

from pai import snap


def a_jpg() -> str:
    return snap(prefix="a")


def b_jpg() -> str:
    return snap(prefix="b")


def c_jpg() -> str:
    return snap(prefix="c")
