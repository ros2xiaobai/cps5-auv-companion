"""DVL TCP transport and CRLF-delimited JSON stream parsing."""

from __future__ import annotations

import json
import socket
from typing import Any

from .config import DvlConfig


class DvlStreamParser:
    """Recover complete JSON reports from an arbitrarily chunked TCP stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[dict[str, Any]]:
        self._buffer.extend(data)
        reports: list[dict[str, Any]] = []

        while True:
            delimiter_index = self._buffer.find(b"\r\n")
            if delimiter_index < 0:
                break

            frame = bytes(self._buffer[:delimiter_index])
            del self._buffer[: delimiter_index + 2]
            if not frame.strip():
                continue

            try:
                decoded = json.loads(frame.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if isinstance(decoded, dict):
                reports.append(decoded)

        return reports


class DvlClient:
    """Single-owner DVL client used by the monitor application."""

    def __init__(self, config: DvlConfig) -> None:
        self.config = config
        self.parser = DvlStreamParser()
        self._socket: socket.socket | None = None

    @property
    def connected(self) -> bool:
        return self._socket is not None

    def connect(self) -> None:
        if self._socket is not None:
            return
        connection = socket.create_connection(
            (self.config.host, self.config.port),
            timeout=self.config.connect_timeout_s,
        )
        connection.settimeout(self.config.receive_timeout_s)
        self._socket = connection

    def poll(self) -> list[dict[str, Any]]:
        if self._socket is None:
            return []
        try:
            data = self._socket.recv(4096)
        except TimeoutError:
            return []
        if not data:
            self.close()
            raise ConnectionError("DVL closed the TCP connection")
        return self.parser.feed(data)

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None
