#!/usr/bin/env python3
"""USB-CDC round-trip latency benchmark for the comms-bench bridge.

The bridge Pico echoes every byte it receives back over USB-CDC. This
script sends N timestamped 32-byte frames, waits for each echo (one in
flight at a time = pure RTT, not throughput), and reports a latency
histogram.

Flash `bridge_echo.uf2` onto a Pico before running this.
"""
from __future__ import annotations

import argparse
import statistics
import struct
import sys
import time


FRAME_LEN = 32
MAGIC = b"\xa5\x5a\xa5\x5a"


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no XOR-out.

    Matches the C reference on the Pico side bit-for-bit.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def make_frame(seq: int, t_send_ns: int) -> bytes:
    """Build a 32-byte benchmark frame (see README for layout)."""
    frame = bytearray(FRAME_LEN)
    frame[0:4] = MAGIC
    struct.pack_into("<I", frame, 4, seq & 0xFFFFFFFF)
    struct.pack_into("<Q", frame, 8, t_send_ns & 0xFFFFFFFFFFFFFFFF)
    # frame[16:30] left as zero — payload pad
    crc = crc16_ccitt(bytes(frame[0:30]))
    struct.pack_into("<H", frame, 30, crc)
    return bytes(frame)


def parse_frame(buf: bytes) -> tuple[int, int] | None:
    if len(buf) != FRAME_LEN:
        return None
    if buf[0:4] != MAGIC:
        return None
    expected = struct.unpack_from("<H", buf, 30)[0]
    if crc16_ccitt(buf[0:30]) != expected:
        return None
    seq = struct.unpack_from("<I", buf, 4)[0]
    t_send = struct.unpack_from("<Q", buf, 8)[0]
    return seq, t_send


def read_one_frame(ser: "serial.Serial") -> bytes | None:
    """Read one 32-byte frame, resynchronising on MAGIC if needed.

    Returns the raw 32 bytes (including magic) or None on timeout.
    """
    # Slide a 4-byte window byte-by-byte until it matches MAGIC.
    sync = bytearray(4)
    for i in range(4):
        b = ser.read(1)
        if not b:
            return None
        sync[i] = b[0]
    while bytes(sync) != MAGIC:
        b = ser.read(1)
        if not b:
            return None
        sync[0] = sync[1]
        sync[1] = sync[2]
        sync[2] = sync[3]
        sync[3] = b[0]
    rest = ser.read(FRAME_LEN - 4)
    if len(rest) != FRAME_LEN - 4:
        return None
    return bytes(sync) + bytes(rest)


def percentile(sorted_values: list[float], p: float) -> float:
    """Nearest-rank percentile on an already-sorted list."""
    n = len(sorted_values)
    idx = min(int(p / 100.0 * n), n - 1)
    return sorted_values[idx]


def print_histogram(values: list[float], buckets: int = 20, bar_width: int = 40):
    if not values:
        return
    lo, hi = min(values), max(values)
    if hi <= lo:
        hi = lo + 1e-6
    step = (hi - lo) / buckets
    counts = [0] * buckets
    for v in values:
        i = min(int((v - lo) / step), buckets - 1)
        counts[i] += 1
    peak = max(counts)
    print(f"\n  Histogram ({len(values)} samples, linear scale):")
    for i, c in enumerate(counts):
        e_lo = lo + i * step
        e_hi = e_lo + step
        bar = "#" * int(bar_width * c / peak) if peak else ""
        print(f"    {e_lo:8.1f}–{e_hi:8.1f} us | {bar:<{bar_width}} {c}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True,
                    help="serial port (e.g. COM7, /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=921600,
                    help="USB-CDC ignores baud, but pyserial wants a value")
    ap.add_argument("-n", "--count", type=int, default=1000,
                    help="number of frames to time (default 1000)")
    ap.add_argument("--gap-us", type=int, default=0,
                    help="sleep between frames in µs (default 0 = back-to-back)")
    ap.add_argument("--timeout", type=float, default=0.1,
                    help="per-frame receive timeout in seconds (default 0.1)")
    ap.add_argument("--warmup", type=int, default=20,
                    help="warmup frames to discard at start (default 20)")
    ap.add_argument("--csv", type=str, default=None,
                    help="if given, write per-frame RTT in µs to this CSV file")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed — run: pip install pyserial")

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)  # let the device settle

    print(f"Sending {args.count} frames (+{args.warmup} warmup) to {args.port} ...")

    rtts_us: list[float] = []
    drops = 0
    seq_mismatch = 0
    crc_bad = 0
    payload_corrupt = 0

    total = args.count + args.warmup
    for seq in range(total):
        t_send_ns = time.perf_counter_ns()
        frame = make_frame(seq, t_send_ns)
        ser.write(frame)

        echo = read_one_frame(ser)
        t_recv_ns = time.perf_counter_ns()

        if echo is None:
            drops += 1
        else:
            parsed = parse_frame(echo)
            if parsed is None:
                crc_bad += 1
            else:
                rx_seq, rx_t_send = parsed
                if rx_seq != seq:
                    seq_mismatch += 1
                elif rx_t_send != t_send_ns:
                    # Echo bytes weren't bit-identical to what we sent.
                    payload_corrupt += 1
                else:
                    rtt_us = (t_recv_ns - t_send_ns) / 1000.0
                    if seq >= args.warmup:
                        rtts_us.append(rtt_us)

        if args.gap_us > 0:
            time.sleep(args.gap_us / 1e6)

    ser.close()

    n_ok = len(rtts_us)
    print()
    print(f"  successful round-trips : {n_ok} / {args.count}")
    print(f"  timeouts               : {drops}")
    print(f"  CRC failures           : {crc_bad}")
    print(f"  sequence mismatches    : {seq_mismatch}")
    print(f"  payload corruption     : {payload_corrupt}")

    if n_ok == 0:
        print("\nNo successful round-trips. Check:")
        print("  * bridge_echo.uf2 is flashed and the LED is blinking")
        print("  * --port matches the Pico's serial device")
        print("  * no other program has the port open")
        sys.exit(1)

    rtts_us.sort()
    print()
    print("  RTT (microseconds):")
    print(f"    min     : {rtts_us[0]:8.1f}")
    print(f"    median  : {statistics.median(rtts_us):8.1f}")
    print(f"    mean    : {statistics.mean(rtts_us):8.1f}")
    print(f"    p90     : {percentile(rtts_us, 90):8.1f}")
    print(f"    p99     : {percentile(rtts_us, 99):8.1f}")
    print(f"    p99.9   : {percentile(rtts_us, 99.9):8.1f}")
    print(f"    max     : {rtts_us[-1]:8.1f}")
    print(f"    stdev   : {statistics.pstdev(rtts_us):8.1f}")

    print_histogram(rtts_us)

    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as f:
            f.write("rtt_us\n")
            for r in rtts_us:
                f.write(f"{r:.3f}\n")
        print(f"\n  Wrote {args.csv}")


if __name__ == "__main__":
    main()
