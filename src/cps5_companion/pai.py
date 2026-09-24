"""Camera snapshot helper.

The main control script calls ``snap()`` when a photo task is triggered.
This implementation captures one frame from the default local camera and
saves it under the ``photos`` directory.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import cv2


def snap(prefix: str = "snap", camera_index: int = 0, output_dir: str = "photos") -> str:
    """Capture a single frame and return the absolute saved image path."""
    photo_dir = Path(output_dir)
    photo_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    image_path = photo_dir / f"{prefix}_{timestamp}.jpg"

    camera = cv2.VideoCapture(camera_index)
    try:
        if not camera.isOpened():
            raise RuntimeError(f"Unable to open camera index {camera_index}")

        ok, frame = camera.read()
        if not ok or frame is None:
            raise RuntimeError("Unable to read a frame from the camera")

        if not cv2.imwrite(str(image_path), frame):
            raise RuntimeError(f"Unable to save image to {image_path}")
    finally:
        camera.release()

    return str(image_path.resolve())
