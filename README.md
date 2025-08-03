# Volvo Power Steering Controller
Volvo power steering P1 CAN controller for managing the pump's speed over CAN. This works with a Haltech ECU that is configured to output duty cycle to IO Box A, such as with a generic output. In this way, the duty cycle can be adjusted in a Haltech table, mapping it to speed or other channels.

## Controller Status
The controller will send status back on CAN 2 (same as Haltech). This can be received by another device (e.g. Motec, AiM) and used for alerts or for display.

Big endian byte order.
| CAN ID     | Rate (Hz) | Type   | Offset(len) | Variable          | Unit | Factor | Mapping                         |
|------------|-----------|--------|-------------|-------------------|------|--------|---------------------------------|
| 0x100D0001 | 10        | ubyte  | 0(1)        | Controller status | 0-4  | 1      | 1 = online                      |
|            |           |        |             |                   |      |        | 2 = pump & ECU offline          |
|            |           |        |             |                   |      |        | 3 = pump offline                |
|            |           |        |             |                   |      |        | 4 = ECU offline                 |
|            |           | ushort | 1(2)        | Pump Percent      | %    | 10     |                                 |
|            |           | ushort | 3(2)        | Pump value        |      | 1      | 0-6000                          |

# Hardware
Targets ESP32 with dual CAN bus.
- https://www.autosportlabs.com/product/esp32-can-x2-dual-can-bus-automotive-grade-development-board/

# References: 
- https://github.com/rusefi/rusefi/blob/master/firmware/controllers/lua/examples/Volvo-electric-power-steering-pump.txt
- https://www.maxxecu.com/webhelp/can_peripheral_control_volvo_powersteering.html
- https://github.com/NMSTEC/Volvo_EPS_FREE/blob/main/src/main.cpp

