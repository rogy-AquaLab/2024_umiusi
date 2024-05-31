from contextlib import contextmanager
from threading import Lock

from serial import Serial


class MutexSerial:
    def __init__(self, *args, **kwargs) -> None:
        self._lock = Lock()
        self._ser = Serial(*args, **kwargs)

    @contextmanager
    def lock(self):
        with self._lock:
            yield self._ser
