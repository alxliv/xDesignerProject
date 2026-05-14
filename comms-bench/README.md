# comms-bench — Phase 1: USB-CDC echo baseline

A minimum-viable round-trip latency benchmark between a laptop (Python)
and a Raspberry Pi Pico over USB-CDC. The Pico just echoes every byte
it receives. **No radio yet** — that's Phase 2.

The point of Phase 1 is to measure the USB + OS-scheduling latency
floor *first*, in isolation. Once we know it, we'll know how much of
the eventual nRF24-loop latency comes from the radio versus the USB
side, and we won't waste a day blaming the radio for jitter the laptop
introduced.

## What "minimum viable" means here

- One frame in flight at a time (no pipelining → pure RTT, not throughput).
- 32-byte fixed frames (matches the nRF24L01 max payload, so the
  Phase 2 frame can be the same shape).
- Magic prefix + sequence number + send timestamp + CRC-16/CCITT.
- 1000 frames by default; first 20 discarded as warmup.
- Reports min / median / mean / P90 / P99 / P99.9 / max in microseconds,
  plus an inline histogram and an optional per-frame CSV dump.

## Files

```
comms-bench/
├── README.md
├── bench.py                            ← laptop side
└── firmware/
    └── bridge_echo/
        ├── CMakeLists.txt
        └── bridge_echo.c               ← USB-CDC echo, no radio
```

## Hardware

- One Pico (any of Pico, Pico W, Pico 2, Pico 2W), USB-C/Micro-USB
  cable, your laptop.

## Frame layout

Same shape used in Phase 2, so the Pico-side parser doesn't have to
change. All multi-byte fields little-endian.

```
offset  size  field
 0      4     MAGIC = 0xA5 0x5A 0xA5 0x5A
 4      4     sequence number (uint32)
 8      8     t_send_ns at the laptop (uint64, perf_counter_ns)
16     14     payload pad — zero in Phase 1, control/telemetry in Phase 2
30      2     CRC-16/CCITT over bytes [0..29]  (poly 0x1021, init 0xFFFF)
```

The echo bridge does **not** parse the frame in Phase 1; it just echoes
bytes. The bench script validates the magic and CRC on the way back.

**CRC reference vector** (for whoever writes the C parser in Phase 2):
`crc16_ccitt(bytes(range(30)))` = `0x3554`. If your C implementation
returns anything else, your CRC is wrong before any bytes hit the wire.

## Building the firmware

You need the pico-sdk installed and `PICO_SDK_PATH` set in your env.
The SDK ships `pico_sdk_import.cmake` at `$PICO_SDK_PATH/external/` —
copy it next to `CMakeLists.txt` first.

```bash
cd firmware/bridge_echo
cp "$PICO_SDK_PATH/external/pico_sdk_import.cmake" .
mkdir build && cd build

# For RP2040 (Pico, Pico W):
cmake -DPICO_BOARD=pico ..
# For RP2350 (Pico 2, Pico 2W):
# cmake -DPICO_BOARD=pico2 ..

cmake --build . -j
```

Output: `build/bridge_echo.uf2`. Hold BOOTSEL while plugging the Pico
into USB to expose the mass-storage volume, drag the `.uf2` onto it.
The board reboots; the on-board LED blinks at 1 Hz so you can confirm
the firmware is running.

## Running the benchmark

```bash
pip install pyserial
python bench.py --port COM7              # Windows
python bench.py --port /dev/ttyACM0      # Linux/macOS
```

Useful flags:

| flag | what |
|------|------|
| `-n 5000` | number of frames (default 1000) |
| `--gap-us 100` | pause between sends in µs (default 0 = back-to-back) |
| `--timeout 0.05` | per-frame receive timeout, seconds (default 0.1) |
| `--warmup 50` | warm-up frames to discard (default 20) |
| `--csv out.csv` | dump per-frame RTT values for later analysis |

## What good numbers look like

Rough expectations (vary by OS, USB controller, and how busy the laptop is):

| platform | median RTT | P99 RTT | drop rate |
|----------|-----------:|--------:|----------:|
| Linux, idle | 0.5–2 ms | < 10 ms | 0% |
| macOS | 1–3 ms | < 20 ms | 0% |
| Windows, idle | 1–5 ms | < 30 ms | 0% |
| Windows, busy | 1–5 ms | 30–100 ms | < 0.1% |

If your **median** is above ~10 ms or you see drops in Phase 1, the
problem is in USB or laptop scheduling — fix that here, before adding
the radio. Otherwise we can't tell radio jitter from USB jitter.

## Phase 2 (next, not in this directory yet)

- Add a second Pico on the "rover" side with an nRF24L01+PA+LNA.
- Bridge Pico: USB-CDC ↔ nRF24 forwarder.
- Rover Pico: nRF24 echo (receives a packet, sends it back as ACK
  payload via Enhanced ShockBurst).
- Same `bench.py`, same frame layout, two new firmware projects.
- The latency we measure then minus the Phase 1 number = radio cost.
