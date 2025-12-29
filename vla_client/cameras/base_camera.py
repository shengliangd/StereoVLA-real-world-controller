from abc import ABC, abstractmethod
import numpy as np
import threading
from typing import List


class BaseCamera(ABC):
    def __init__(self, fps=30):
        self._fps = fps
        self._frames = None

        def update_frames():
            import time
            while True:
                start_time = time.time()
                self._frames = self._get_frames()
                elapsed = time.time() - start_time
                sleep_time = max(0, 1/self._fps - elapsed)
                time.sleep(sleep_time)
        self._thread = threading.Thread(target=update_frames, daemon=True)
        self._thread.start()

    @abstractmethod
    def _get_frames(self) -> List[np.ndarray]:
        pass

    def get_frames(self) -> List[np.ndarray]:
        while self._frames is None:
            import time
            time.sleep(1/self._fps)
        return self._frames

    @staticmethod
    @abstractmethod
    def list_cameras(self) -> List[str]:
        pass

    @abstractmethod
    def get_num_frames(self) -> int:
        pass
