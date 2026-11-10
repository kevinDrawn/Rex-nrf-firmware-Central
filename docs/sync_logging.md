# External sync input

`include/sync_pin.h` defines P0.05 as an active-high input with an internal
pull-down. `main()` calls `sync_pin_init()` before reception and sampling.
The nRF21540 DK overlay releases P0.05 from UART0 RTS; UART0 uses TX/RX only.

Each 1 kHz EXO sample reads the physical level: `sync_state=0` for Low and
`sync_state=1` for High. There is no interrupt, event latch, or debounce.
Pulses entirely between samples are not recorded. UART queue drops can
still lose entire batches.

## UART format

Offsets exclude the four-byte header. Existing fields retain their encoding.

| Payload offset | Field | Encoding |
| --- | --- | --- |
| 0-119 | Six EXO blocks | 10 int16 samples per block, little endian |
| 120-359 | Four EMG blocks | 20 signed 24-bit samples per block, big endian |
| 360-369 | `sync_state` | 10 uint8 samples: 0=Low, 1=High |

Header: `AA 55 72 01` (370-byte payload, little-endian length).
Total frame size: 374 bytes, every 10 ms. State samples align by index with
EXO samples. Update the PC parser to accept this length and save a
`sync_state` column. The PC receiver is not included in this repository.

## Hardware check

Using a common ground and a GPIO-compatible signal, verify idle Low,
held High, and return to Low. Check the UART length and PC log state column.
