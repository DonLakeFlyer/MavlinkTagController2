"""UdpReceiver: dedicated receive thread feeding a bounded packet ring."""

import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from udp_receiver import PacketRing, UdpReceiver  # noqa: E402


def test_ring_rejects_non_positive_capacity():
    for bad in (0, -1):
        try:
            PacketRing(max_packets=bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f'PacketRing({bad}) must raise ValueError')


def test_ring_is_fifo():
    ring = PacketRing(max_packets=4)
    for i in range(3):
        assert ring.push(bytes([i]))
    assert [ring.pop(timeout=0.01) for _ in range(3)] == [b'\x00', b'\x01', b'\x02']
    assert ring.pop(timeout=0.01) is None


def test_ring_overflow_drops_newest_and_counts():
    ring = PacketRing(max_packets=2)
    assert ring.push(b'a')
    assert ring.push(b'b')
    assert not ring.push(b'c')
    assert ring.dropped == 1
    assert ring.pop(timeout=0.01) == b'a'
    assert ring.pop(timeout=0.01) == b'b'
    assert ring.pop(timeout=0.01) is None


def test_ring_pop_wakes_on_push():
    ring = PacketRing(max_packets=2)
    got = []

    def consumer():
        got.append(ring.pop(timeout=2.0))

    t = threading.Thread(target=consumer)
    t.start()
    time.sleep(0.05)
    ring.push(b'x')
    t.join(timeout=2.0)
    assert got == [b'x']


def test_receiver_thread_delivers_all_datagrams():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(('127.0.0.1', 0))
    port = rx.getsockname()[1]
    ring = PacketRing(max_packets=16)
    receiver = UdpReceiver(rx, ring)
    receiver.start()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
            for i in range(5):
                tx.sendto(bytes([i]) * 16, ('127.0.0.1', port))
        # Ordering is the socket's property, not the receiver's; FIFO of the
        # ring itself is pinned by test_ring_is_fifo.
        got = {ring.pop(timeout=1.0) for _ in range(5)}
        assert got == {bytes([i]) * 16 for i in range(5)}
    finally:
        receiver.stop()
        rx.close()
    assert not receiver.is_alive()
