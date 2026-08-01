"""Line framing over a serial port.

Split out from bridge.py so the node depends on a tiny interface
(`write_line` / `read_available_lines` / `close`) rather than on pyserial
directly - tests substitute a fake and never open a port.
"""

from typing import List


class SerialLineTransport:
    """NDJSON over pyserial: non-blocking reads, partial lines buffered until
    their newline arrives (a USB hot-plug routinely cuts a line in half)."""

    def __init__(self, port: str, baud: int):
        import serial  # imported here so the codec/tests never need pyserial

        self._ser = serial.Serial(port=port, baudrate=baud, timeout=0)
        self._buf = ""

    def write_line(self, line: str) -> None:
        self._ser.write((line + "\n").encode("utf-8"))
        self._ser.flush()

    def read_available_lines(self) -> List[str]:
        """Returns every complete line currently buffered; never blocks."""
        waiting = self._ser.in_waiting
        if waiting:
            self._buf += self._ser.read(waiting).decode("utf-8", errors="replace")

        if "\n" not in self._buf:
            return []

        *complete, self._buf = self._buf.split("\n")
        return complete

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:  # noqa: BLE001 - closing a dead port must not raise
            pass
