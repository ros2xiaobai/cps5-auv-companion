"""Small timing helpers used by the AUV control script."""

import time


def sleep_ms(milliseconds: int | float) -> None:
    """Sleep for the requested number of milliseconds."""
    time.sleep(float(milliseconds) / 1000.0)
