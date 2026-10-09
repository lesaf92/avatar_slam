"""Water Linked Modem-M64: serial protocol and link adapter (task T-C6-01).

Protocol (https://docs.waterlinked.com/modem-m64/modem-m64-protocol/, checked 2026-10-07):
UART 115200 8-N-1; messages ``w`` + direction (``c`` command, ``r`` response) + command letter +
``,``-separated fields + ``*`` + checksum + ``\\n``. The checksum is CRC-8 (polynomial 0x07,
initial value 0, no reflection: CRC-8/SMBUS) over everything before ``*``, written as two
lower-case hex digits; these parameters are the only ones that reproduce all six examples of
the protocol page (``tests/test_m64.py``). A packet carries **8 bytes** of payload, which may be
binary; an all-zero payload is reserved (sync packets, filtered by the receiver). Two modems
talk in a pair, roles ``a`` and ``b`` on one of channels 1-7.

The link adapter carries Avatar wire packets (at most 64 B on the M64, ``docs/hardware.md``)
over those 8-byte frames: byte 0 of a frame is a header (packet id 3 bits, last-fragment flag,
fragment index 4 bits), bytes 1-7 data. The data of a packet start with its length (one byte),
so the last fragment can be zero-padded; a 64 B packet takes 10 frames. A packet whose frames
do not all arrive, in order, is dropped. The first frame of a packet holds its non-zero length
and every later frame a non-zero header, so no frame is ever all zeros.
"""

from __future__ import annotations

from collections.abc import Callable

PAYLOAD_B = 8
FRAME_DATA_B = PAYLOAD_B - 1
MAX_FRAGMENTS = 16
MAX_PACKET_B = MAX_FRAGMENTS * FRAME_DATA_B - 1  # 111 B; Avatar sends at most 64 B on the M64


def crc8(data: bytes) -> int:
    """CRC-8/SMBUS (poly 0x07, init 0, no reflection, no final XOR)."""
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def command(name: str, *fields: str | bytes) -> bytes:
    """One command line, e.g. ``command("s", "a", "3")`` -> ``b"wcs,a,3*xx\\n"``."""
    body = (
        b"wc"
        + name.encode()
        + b"".join(b"," + (f if isinstance(f, bytes) else f.encode()) for f in fields)
    )
    return body + b"*%02x\n" % crc8(body)


def queue_frame(frame: bytes) -> bytes:
    """``wcq`` command that queues one 8-byte frame for transmission."""
    if len(frame) != PAYLOAD_B or not any(frame):
        raise ValueError("a frame has 8 bytes, not all zero")
    return command("q", str(PAYLOAD_B), frame)


def fragment(packet: bytes, packet_id: int) -> list[bytes]:
    """The 8-byte frames of one wire packet (``packet_id`` modulo 8)."""
    if not 0 < len(packet) <= MAX_PACKET_B:
        raise ValueError(
            f"packet of {len(packet)} B: 1-{MAX_PACKET_B} B fit {MAX_FRAGMENTS} frames"
        )
    data = bytes([len(packet)]) + packet
    chunks = [data[i : i + FRAME_DATA_B] for i in range(0, len(data), FRAME_DATA_B)]
    frames = []
    for i, chunk in enumerate(chunks):
        header = (packet_id % 8) << 5 | (i == len(chunks) - 1) << 4 | i
        frames.append(bytes([header]) + chunk.ljust(FRAME_DATA_B, b"\0"))
    return frames


class Reassembler:
    """Frames -> wire packets; an incomplete packet is dropped (``dropped`` counts them)."""

    def __init__(self) -> None:
        self._parts: list[bytes] | None = None
        self._id = -1
        self.dropped = 0

    def push(self, frame: bytes) -> bytes | None:
        """Add one received frame; returns a packet when it completes one."""
        if len(frame) != PAYLOAD_B or not any(frame):
            return None  # not a data frame (sync packets are all zeros)
        header, data = frame[0], frame[1:]
        pid, last, index = header >> 5, header >> 4 & 1, header & 0x0F
        if index == 0:
            if self._parts is not None:
                self.dropped += 1  # the previous packet lost its tail
            self._parts, self._id = [data], pid
        elif self._parts is None or pid != self._id or index != len(self._parts):
            if self._parts is not None:
                self.dropped += 1  # a fragment of this packet was lost
            self._parts = None
            return None
        else:
            self._parts.append(data)
        if not last:
            return None
        blob, self._parts = b"".join(self._parts), None
        size = blob[0]
        if size == 0 or size > len(blob) - 1:
            self.dropped += 1
            return None
        return blob[1 : 1 + size]


class ResponseParser:
    """Splits the modem's output into ``(command letter, fields)``, checking every checksum.

    A received packet (``wrp,<size>,<payload>``) is read by length: its binary payload may
    contain ``,``, ``*`` or a line end. Lines with a wrong or missing checksum are dropped and
    counted in ``bad``.
    """

    def __init__(self) -> None:
        self._buf = b""
        self.bad = 0

    def feed(self, data: bytes) -> list[tuple[str, list[bytes]]]:
        self._buf += data
        out = []
        while True:
            start = self._buf.find(b"w")
            if start < 0:
                self._buf = b""
                return out
            buf = self._buf = self._buf[start:]
            if buf.startswith(b"wrp,"):
                comma = buf.find(b",", 4)
                if comma < 0 and len(buf) < 8:
                    return out  # the size field is not complete yet
                if comma < 0 or not buf[4:comma].isdigit():
                    self._buf, self.bad = buf[1:], self.bad + 1
                    continue
                star = comma + 1 + int(buf[4:comma])  # the payload is read by length
                tail = buf[star : star + 5]  # "*xx\n" or "*xx\r\n"
                cr = tail[3:4] == b"\r"
                if len(tail) < 4 + cr:
                    return out
                self._buf = buf[star + 4 + cr :]
                if tail[:3] != b"*%02x" % crc8(buf[:star]) or tail[3 + cr : 4 + cr] != b"\n":
                    self.bad += 1
                    continue
                out.append(("p", [buf[4:comma], buf[comma + 1 : star]]))
                continue
            nl = buf.find(b"\n")
            if nl < 0:
                return out
            line, self._buf = buf[:nl].rstrip(b"\r"), buf[nl + 1 :]
            star = line.rfind(b"*")
            if star < 0 or len(line) < 3 or line[1:2] != b"r":
                self.bad += 1
                continue
            body = line[:star]
            if line[star + 1 :] != b"%02x" % crc8(body):
                self.bad += 1
                continue
            fields = body[3:].split(b",")[1:] if len(body) > 3 else []
            out.append((body[2:3].decode(), fields))


class M64Link:
    """Wire packets over one Modem-M64 (byte-stream in and out; no serial-port code here).

    ``write`` receives the bytes for the modem's UART. ``send`` queues one packet as frames;
    ``receive`` takes the bytes read from the UART and returns the packets they complete, plus
    any other responses (``responses``, e.g. ``("q", [b"n"])`` when the modem refused a frame).
    """

    def __init__(self, write: Callable[[bytes], None]) -> None:
        self._write = write
        self._next_id = 0
        self.parser = ResponseParser()
        self.reassembler = Reassembler()
        self.responses: list[tuple[str, list[bytes]]] = []

    def send(self, packet: bytes) -> int:
        """Queue ``packet``; returns the number of frames (8 B each on the air)."""
        frames = fragment(packet, self._next_id)
        self._next_id = (self._next_id + 1) % 8
        for f in frames:
            self._write(queue_frame(f))
        return len(frames)

    def receive(self, data: bytes) -> list[bytes]:
        packets = []
        for name, fields in self.parser.feed(data):
            if name == "p" and len(fields) == 2:
                p = self.reassembler.push(fields[1])
                if p is not None:
                    packets.append(p)
            else:
                self.responses.append((name, fields))
        return packets


def frames_on_air(packet_bytes: int) -> int:
    """Frames (8 B each) that a wire packet of ``packet_bytes`` takes on the M64."""
    return -(-(packet_bytes + 1) // FRAME_DATA_B)


__all__ = [
    "PAYLOAD_B",
    "M64Link",
    "Reassembler",
    "ResponseParser",
    "command",
    "crc8",
    "fragment",
    "frames_on_air",
    "queue_frame",
]
