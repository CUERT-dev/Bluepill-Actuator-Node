# Bluepill CAN reception test

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

Firmware `BLUE FW can-link-v1` prints a CAN report every second. Expect
`configured=1 ready=1`, then rising `rx` and `decode` counters when the ESP
forwards real valid commands. `seq`, `thr`, `str` and `flags` report the most
recent decoded command. `motor=0` is expected in this CAN-only test image.

See the complete paired-node build, wiring, validation and troubleshooting
procedure in [the ESP CAN link guide](../../ESP32-Sensing-Node/docs/CAN_LINK.md).
The ESP is configured for GPIO19 TX / GPIO18 RX, 500 kbit/s, ID 0x105, DLC 8.

Bluepill CanBench firmware checks and 131 host tests passed locally. Physical
reception is still unverified until the flashed board reports rising counters.
