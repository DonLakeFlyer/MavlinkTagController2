"""Dedicated UDP receive thread so processing stalls never block the socket."""

import collections
import socket
import threading


class PacketRing:
    """Bounded FIFO of raw datagrams. Overflow drops the newest and is counted."""

    def __init__(self, max_packets):
        self._max = int(max_packets)
        if self._max <= 0:
            raise ValueError(f'max_packets must be > 0, got {max_packets}')
        self._q = collections.deque()
        self._cv = threading.Condition()
        self._dropped = 0

    @property
    def dropped(self):
        with self._cv:
            return self._dropped

    def push(self, packet):
        with self._cv:
            if len(self._q) >= self._max:
                self._dropped += 1
                return False
            self._q.append(packet)
            self._cv.notify()
            return True

    def pop(self, timeout):
        with self._cv:
            if not self._q and not self._cv.wait_for(lambda: bool(self._q), timeout):
                return None
            return self._q.popleft()

    def __len__(self):
        with self._cv:
            return len(self._q)


class UdpReceiver(threading.Thread):
    def __init__(self, sock, ring, max_datagram=65536):
        super().__init__(name='udp-rx', daemon=True)
        self._sock = sock
        self._ring = ring
        self._max = max_datagram
        self._stop_event = threading.Event()

    def run(self):
        self._sock.settimeout(0.5)
        while not self._stop_event.is_set():
            try:
                data = self._sock.recv(self._max)
            except socket.timeout:
                continue
            except OSError:
                break
            self._ring.push(data)

    def stop(self, join_timeout=2.0):
        self._stop_event.set()
        self.join(timeout=join_timeout)
