# Volvo Power Steering Controller
Volvo power steering P1 CAN controller for managing the pump's speed over CAN. This works with a Haltech ECU that is configured to output duty cycle to IO Box A, such as with a generic output. In this way, the duty cycle can be adjusted in a Haltech table, mapping it to speed or other channels.

## Controller Status
The controller will send status back on CAN 2 (same as Haltech). This can be received by another device (e.g. Motec, AiM) and used for alerts or for display.

Big endian byte order.
| CAN ID     | Rate (Hz) | Type   | Offset(len) | Variable          | Unit | Factor | Mapping                         |
|------------|-----------|--------|-------------|-------------------|------|--------|---------------------------------|
| 0x100D0001 | 5         | ubyte  | 0(1)        | Controller status | 0-4  | 1      | 1 = online                      |
|            |           |        |             |                   |      |        | 2 = pump & ECU offline (unused) |
|            |           |        |             |                   |      |        | 3 = pump offline                |
|            |           |        |             |                   |      |        | 4 = ECU offline                 |
|            |           | ushort | 1(2)        | Pump Percent      | %    | 10     |                                 |
|            |           | ushort | 3(2)        | Pump value        |      | 1      | 0-6000                          |

Status 2 is no longer emitted. When the ECU goes offline the controller stops the pump
keep alive on purpose, so the pump falls silent shortly afterwards as a consequence of
that rather than as a second fault. Reporting 4 keeps a plain ECU dropout from pointing
at the wrong box. The code is kept in the table because the value is part of the protocol.

Pump Percent and Pump value both read 0 while the controller is not commanding the pump,
rather than holding the last live values.

# Hardware
Targets ESP32 with dual CAN bus.
- https://www.autosportlabs.com/product/esp32-can-x2-dual-can-bus-automotive-grade-development-board/

# References: 
- https://github.com/rusefi/rusefi/blob/master/firmware/controllers/lua/examples/Volvo-electric-power-steering-pump.txt
- https://www.maxxecu.com/webhelp/can_peripheral_control_volvo_powersteering.html
- https://github.com/NMSTEC/Volvo_EPS_FREE/blob/main/src/main.cpp


# Building
The sketch in `ps-controller/` builds two ways, from the same files. Nothing is
duplicated: the PlatformIO project points `src_dir` at the sketch folder.

## Arduino IDE
Select **AutosportLabs ESP-CAN-X2** (board support comes from Espressif's
`package_esp32_index.json`) and build `ps-controller/ps-controller.ino`.

## PlatformIO
```
pio run              # build
pio run -t upload    # build and flash
pio device monitor   # serial at 115200
```
Or use the PlatformIO VS Code extension, which picks up `platformio.ini`.

This uses the [pioarduino](https://github.com/pioarduino/platform-espressif32)
platform fork rather than the registry `espressif32`. Espressif no longer maintains
official PlatformIO support, so the registry platform is pinned to arduino-esp32
2.0.17, which predates the `aslcanx2` board variant and is a different ESP-IDF
generation (4.4 rather than 5.x). That matters here because the firmware leans on the
TWAI driver, so building against the same 3.x core the Arduino IDE flashes keeps the
two toolchains producing equivalent firmware.

Set `PS_DEBUG_SERIAL` to 1 at the top of the sketch for per-frame serial tracing.
Failures, init progress and CAN 1 bus-off transitions always print.

`ps-controller/ps_logic.h` holds the pure controller logic (duty cycle conversion,
frame packing, online timeouts) with no Arduino or ESP-IDF dependencies, so the same
code the firmware runs can be compiled and tested on a host.

# Tests
Host-side unit tests for `ps_logic.h` live in `test/`. They need nothing but a C++17
compiler, and they do not touch the Arduino build.

```
make -C test          # build and run
make -C test clean
```

On Windows, run them from WSL:

```
wsl make -C test
```
