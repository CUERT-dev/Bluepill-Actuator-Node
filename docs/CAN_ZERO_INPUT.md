# Received joystick values remain zero

The user reports moving the joystick while BLUE CAN throttle and steering stay
zero, and confirmed that decode keeps increasing. This observation points to
decoded zero demands. It does not establish which upstream stage produces them,
and the current flags/raw packet have not been supplied. Further changes to
actuator permissions cannot reconstruct a missing joystick demand.

The local Blackpill source initializes every sample as neutral/invalid. Failed
ADC reads or invalid calibration return that sample for transmission. Successful
acquisition sets VALID and normalizes throttle/steering independently of ARM and
DEADMAN. Its existing debug USART2 (PA2 TX, 115200 8N1) reports raw ADC values,
acquisition result, CMD and the last transmitted UART packet. The current ESP
CAN bridge source was not found in the available project tree; historical relay
code is not evidence of the flashed bridge's behavior.

## v5 diagnostic update

Capture retains the last eight CAN payload bytes, ID, DLC, IDE/RTR and ISR
timestamp before decoding or queue admission. These diagnostics survive motor
shutdown and also show rejected packets. They never authorize motion. Status
and the automatic one-second report now include:

```text
CAN RX have=1 id=105 dlc=8 ide=0 rtr=0 data=... rx_ms=... last_decoded=... decoded_ms=...
```

`last_decoded` identifies the last successfully decoded sample, independently of
the latest raw packet. Compare timestamps to determine freshness:

| Label | Meaning |
|---|---|
| NO_VALID_SAMPLE | No packet has successfully decoded since boot |
| STOP | Last decoded packet has VALID clear and zero demands |
| NEUTRAL | Last decoded packet has VALID set and zero demands |
| DEMAND | Last decoded packet contains a nonzero demand |

Revision-10 payload bytes are sequence [0:1], throttle [2:3], signed steering
[4:5], flags [6], CRC8 [7], with little-endian 16-bit fields. For example,
`0100580238FF0762` is sequence 1, throttle 600, steering -200, flags 7.
Nonzero axis bytes with no valid decode require examining DLC/format/CRC;
successfully decoded zero axis bytes require checking upstream acquisition and
forwarding. A stopped output does not zero the captured/decoded input report.

This is an input diagnostic update, not a claimed repair of the observed
physical input failure. Direct actuation and v4 transient resumption remain.
No upstream files were edited and no board was flashed or physically tested.

## Verification and artifacts

142/142 host tests passed in both Debug and Release. Capture tests cover raw
evidence before decoding and preservation through shutdown. Formatting tests
cover all four labels, raw bytes, rejected-frame distinction and truncation.
CanBenchRelease, SteeringBenchRelease and CanBench built; linked firmware,
preset/cache/manifest/hash, enabled syntax, negative configuration, peripheral
ownership and CubeMX hook checks passed. Both active HEX files passed record
length, checksum and EOF validation.

```text
build/CanBenchRelease/Bluepill-Actuator-Node.hex
SHA256 b76c84faba48495a10493f4b011da53ff9396fa545c1256070b5dd9b92649b39
Flash 25856 bytes; linker-reserved RAM 13280 bytes

build/SteeringBenchRelease/Bluepill-Actuator-Node.hex
SHA256 9a4800b16c2002e4590739bdeaeaee0a1fc0e4e483402a54c74aaecd201f2414
Flash 25840 bytes; linker-reserved RAM 13280 bytes
```
