"""Record one tag's decimated IQ stream to disk until terminated.

Launched by the controller for START_IQ_RECORDING (antenna pattern calibration
sweeps, issue #183). No detection is run.

Output in --out-dir:
  iq_tag<T>.c64        complex64 samples (interleaved little-endian float32 I/Q),
                       every datagram's payload appended in arrival order
  iq_tag<T>_index.csv  one row per datagram: sample_index, timestamp_ns, n_samples
                       (timestamp is the controller clock at the datagram's first
                       sample; a jump larger than n_samples / fs marks a gap)
  iq_tag<T>.json       sample rate, tag id and the file layout above
"""

import argparse
import json
import os
import signal
import socket
import struct
import sys

TIMESTAMP_BYTES = 8
SAMPLE_BYTES = 8


def decode_timestamp_ns(packet):
    """Decimator header: uint32 seconds + uint32 nanoseconds, little-endian."""
    sec, nsec = struct.unpack_from('<II', packet)
    return sec * 1_000_000_000 + nsec


class IqRecorder:
    def __init__(self, out_dir, tag_id, fs):
        stem = os.path.join(out_dir, f'iq_tag{tag_id}')
        with open(f'{stem}.json', 'w') as meta:
            json.dump({'tag_id': tag_id,
                       'fs': fs,
                       'samples': f'iq_tag{tag_id}.c64',
                       'sample_format': 'complex64 little-endian',
                       'index': f'iq_tag{tag_id}_index.csv'}, meta, indent=2)
        # Samples flushed before their index row: the controller stops this process with SIGKILL
        self._samples = open(f'{stem}.c64', 'wb')
        self._index = open(f'{stem}_index.csv', 'w', buffering=1)
        self._index.write('sample_index,timestamp_ns,n_samples\n')
        self.sample_count = 0
        self.datagram_count = 0

    def write(self, packet):
        """Append one decimator datagram. Returns False if it was malformed."""
        payload_bytes = len(packet) - TIMESTAMP_BYTES
        if payload_bytes <= 0 or payload_bytes % SAMPLE_BYTES:
            return False
        n_samples = payload_bytes // SAMPLE_BYTES
        timestamp_ns = decode_timestamp_ns(packet)
        self._samples.write(packet[TIMESTAMP_BYTES:])
        self._samples.flush()  # retries short writes; raises OSError rather than drop bytes
        self._index.write(f'{self.sample_count},{timestamp_ns},{n_samples}\n')
        self.sample_count += n_samples
        self.datagram_count += 1
        return True

    def close(self):
        self._samples.close()
        self._index.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', type=int, required=True, help='UDP port of the decimated IQ stream')
    ap.add_argument('--tag-id', type=int, required=True)
    ap.add_argument('--fs', type=int, required=True, help='Stream sample rate (recorded in the json only)')
    ap.add_argument('--out-dir', type=str, required=True)
    args = ap.parse_args()

    stop = False

    def _stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(0.5)

    recorder = IqRecorder(args.out_dir, args.tag_id, args.fs)
    print(f'Recording tag {args.tag_id} from UDP :{args.port} to {args.out_dir}', flush=True)

    while not stop:
        try:
            packet = sock.recv(65536)
            if not recorder.write(packet):
                print(f'Warning: dropped malformed {len(packet)} byte datagram', file=sys.stderr, flush=True)
        except socket.timeout:
            pass

    recorder.close()
    print(f'Recorded {recorder.sample_count} samples in {recorder.datagram_count} datagrams', flush=True)


if __name__ == '__main__':
    main()
