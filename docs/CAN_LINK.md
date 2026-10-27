# Bluepill CAN reception test

**CanBenchRelease now enables full-range actuation** using the existing
commissioning profile with **direct commands and no operator arming sequence**.
See [the current behavior and HEX](DIRECT_COMMANDS.md).
It retains the same CAN timing/decoder; it is no longer a receive-only image.

Use **CanBench**, which enables real 500 kbit/s CAN on PB8 RX / PB9 TX and
inhibits motor outputs. Debug disables CAN. Normal Release now enables CAN
and both motor outputs using the commissioning configuration.

```powershell
cmake --preset CanBench
cmake --build --preset CanBench
```

Flash `build/CanBench/Bluepill-Actuator-Node.hex`. Connect the Bluepill FTDI to
PA9 TX and GND; open **115200 baud, 8N1, no flow control**. Optional FTDI TX to
PA10 supports `status` / `recover` followed by newline.

Firmware `BLUE FW actuation-diag-v2` prints a CAN report and an `ACT` output
report every second. See [ACTUATION_DIAGNOSIS.md](ACTUATION_DIAGNOSIS.md) for
supervisor states, timer/GPIO fields and the task-ordering correction. Expect
`configured=1 ready=1`, then rising `rx` and `decode` counters when the ESP
forwards real valid commands. `seq`, `thr`, `str` and `flags` report the most
recent decoded command. `motor=0` is expected in this CAN-only test image.

See the complete paired-node build, wiring, validation and troubleshooting
procedure in [the ESP CAN link guide](../../ESP32-Sensing-Node/docs/CAN_LINK.md).
The ESP is configured for GPIO19 TX / GPIO18 RX, 500 kbit/s, ID 0x105, DLC 8.

The updated CanBench firmware checks and 133 host tests passed locally in both
Debug and Release host configurations. Verify rising counters on the flashed
board; CAN-only firmware intentionally cannot enable either motor.
