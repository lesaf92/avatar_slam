"""Modem-M64 protocol and link adapter (T-C6-01)."""

import numpy as np
import pytest

from avatar.comm import codec
from avatar.comm.m64 import (
    M64Link,
    ResponseParser,
    command,
    crc8,
    fragment,
    frames_on_air,
)

# Verbatim examples of https://docs.waterlinked.com/modem-m64/modem-m64-protocol/ (2026-10-07)
DOC_RESPONSES = [
    b"wrv,1,0,1*44",
    b"wrn,8*ba",
    b"wrs,a*01",
    b"wrq,a*d7",
    b"wrp,8,HelloTop*bb",
    b"wrp,8,HelloSea*58",
]


@pytest.mark.parametrize("line", DOC_RESPONSES)
def test_crc8_reproduces_the_protocol_examples(line):
    body, check = line.rsplit(b"*", 1)
    assert b"%02x" % crc8(body) == check


def test_parser_reads_the_examples_and_binary_payloads():
    p = ResponseParser()
    out = p.feed(b"\r\n".join(DOC_RESPONSES) + b"\r\n")
    assert [n for n, _ in out] == ["v", "n", "s", "q", "p", "p"]
    assert out[1][1] == [b"8"] and out[4][1] == [b"8", b"HelloTop"]
    # a binary payload with a line end and a "*" inside, split across reads, and a corrupted line
    payload = b"a*\nb\r\x00,z"
    line = b"wrp,8," + payload
    stream = line + b"*%02x\n" % crc8(line) + b"wrq,a*00\n"
    got = p.feed(stream[:9]) + p.feed(stream[9:])
    assert got == [("p", [b"8", payload])] and p.bad == 1


def test_commands_carry_their_checksum():
    assert command("s", "a", "3") == b"wcs,a,3*%02x\n" % crc8(b"wcs,a,3")
    assert command("v") == b"wcv*%02x\n" % crc8(b"wcv")


def test_fragments_never_send_the_reserved_all_zero_payload():
    for n in (1, 6, 7, 64):
        frames = fragment(bytes(n), packet_id=0)  # an all-zero packet
        assert len(frames) == frames_on_air(n) and all(any(f) and len(f) == 8 for f in frames)
    assert frames_on_air(64) == 10


def _modem_pair(rng, loss):
    """Two links whose queued frames reach the other side as ``wrp`` lines, some lost."""
    inbox = {0: [], 1: []}
    links = {}

    def make(me):
        def write(cmd):
            assert cmd.startswith(b"wcq,8,") and cmd.endswith(b"\n")
            frame = cmd[6:14]
            assert cmd[14:] == b"*%02x\n" % crc8(cmd[:14])
            if rng.random() >= loss:
                line = b"wrp,8," + frame
                inbox[1 - me].append(line + b"*%02x\r\n" % crc8(line))

        return M64Link(write)

    links[0], links[1] = make(0), make(1)
    return links, inbox


def test_loopback_delivers_wire_packets_and_drops_incomplete_ones():
    rng = np.random.default_rng(3)
    for loss in (0.0, 0.1):
        links, inbox = _modem_pair(rng, loss)
        sent = []
        for i in range(200):
            rec = codec.LandmarkRecord(i + 1, (1.0, -2.0, 3.0), 0.1, 0.2, (0.5, 0.5, 2.0), 3, 1, 4)
            msg = codec.LandmarkDigest(2, i, 1000 * i, 3, 0, (rec,) * (1 + i % 3))
            pkt = codec.encode(msg)
            assert len(pkt) <= 64
            sent.append(pkt)
            links[0].send(pkt)
        stream = b"".join(inbox[1])
        got = []
        for i in range(0, len(stream), 37):  # arbitrary read sizes
            got += links[1].receive(stream[i : i + 37])
        assert all(codec.decode(p).seq >= 0 for p in got)  # intact: decode checks the CRC-16
        assert set(got) <= set(sent)
        if loss == 0.0:
            assert got == sent and links[1].reassembler.dropped == 0
        else:
            assert 0 < len(got) < len(sent) and links[1].reassembler.dropped > 0
