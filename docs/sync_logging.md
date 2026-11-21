# External sync input

`include/sync_pin.h` defines P0.03 as an active-high input with an internal
pull-down. `main()` calls `sync_pin_init()` before reception and sampling.
P0.03 is the external sync input. The nRF21540 DK overlay releases P0.05
from UART0 RTS; UART0 uses TX/RX only.

Each 1 kHz EXO sample reads the physical level: `sync_state=0` for Low and
`sync_state=1` for High. There is no interrupt, event latch, or debounce.
Pulses entirely between samples are not recorded. UART queue drops can
still lose entire batches.

## UART format

Offsets start at the payload. Each channel is sampled at 1 kHz and packed
as a separate block of 10 samples. The signed blocks use little-endian int16
values scaled by 10000.

| Payload offset | Field | Source or encoding |
| --- | --- | --- |
| 0-19 | `qL` | `mc.q_L` |
| 20-39 | `qR` | `mc.q_R` |
| 40-59 | `qMirror` | `mc.q_mirror` |
| 60-79 | `torque_axis` | `mc.torque_axis` |
| 80-99 | `torque_can_L` | `mc.torque_can_left` |
| 100-119 | `torque_can_R` | `mc.torque_can_right` |
| 120-139 | `q_nominal` | `mc.nominal_q` |
| 140-159 | `q_compensation` | `mc.torque_admittance` |
| 160-169 | `sync_state` | 10 uint8 samples, 0=Low and 1=High |

The header is `AA 55 AA 00`, with a 170-byte payload length in little-endian
order. The 174-byte frame is sent every 10 ms. All blocks align by sample
index. The PC parser must accept the 170-byte payload and decode this layout.

## Hardware check

Using a common ground and a GPIO-compatible signal, verify idle Low,
held High, and return to Low. Check the UART length and PC log state column.
