# innova-fancoil

ESPHome firmware for reading an **Innova AirLeaf SL** fancoil (tested on an SL600, control board
N540022C) over its RS485 port with a [LilyGo T-CAN485](https://github.com/Xinyuan-LilyGO/T-CAN485)
ESP32 board, plus a read-only **Modbus discovery tool** for devices with an unknown register map.

Innova does not publish the Modbus register map for this board. Everything below was found by
probing and is **unofficial and incomplete**.

**Status:** the fancoil answers Modbus RTU (settings below) and a first register dump is done. The registers are not mapped yet, so the firmware has no fancoil sensors yet, only the
discovery tool.

> Not affiliated with or endorsed by Innova. The fancoil has mains voltage inside: switch it off at
> the breaker before opening it. Use at your own risk.

## Findings

### Connection

| | |
|---|---|
| Port | 4-pin green terminal **"+ A B −"** on the control board (unused from the factory) |
| Mode, jumper on **RTU** | **Modbus RTU, 9600 baud, 8N1** (recommended; ESPHome `modbus_controller` speaks RTU) |
| Mode, jumper on **ASCII** (factory) | **Modbus ASCII, 9600 baud, 7E1** |
| Slave address | **1** |
| Functions | 03 and 04 both answer, with identical values |
| Polarity | The fancoil's **B** idles high (+2.3 V to −), **A** low (+0.9 V). Innova uses the "B = D+" convention, so wire fancoil **B → transceiver A** and fancoil **A → transceiver B** for a Maxim-style transceiver like the one on the T-CAN485 |
| GND | Fancoil **−** to the LilyGo's GND |
| + | Supply voltage, not measured yet; not connected (the LilyGo runs on USB-C) |

The board's **ASCII / RTU jumper** (3-pin, next to the DIP switches) selects the Modbus framing. It
comes in the **ASCII** position, where the board ignores all RTU frames, so an RTU-only scan finds
nothing. Moving the cap to the two pins on the "RTU" side (fancoil switched off at the breaker)
gives Modbus RTU at 9600 8N1, which ESPHome's `modbus_controller` speaks. Register values are the
same in both modes. The transceiver on the fancoil board is an ST485.

![Wiring diagram on the cover](docs/img/wiring-label.jpg)

| Board | RS485 terminal | ASCII/RTU jumper and DIP switches |
|---|---|---|
| ![Board](docs/img/board-overview.jpg) | ![RS485 terminal](docs/img/rs485-terminal.jpg) | ![Jumper and DIP switches](docs/img/dip-jumper.jpg) |

DIP switches as found: A off, B off, C on, D off (meaning unknown, see the installation manual's
"Set-Up of Auxiliary Dip-Switch Functions"). The 4-pin **ZOT** header next to the pads `+5V SW GND
RST` is the STM8 programming (SWIM) port, not a bus.

### Registers (first dump, unconfirmed)

Dump of registers 0–99 with FC03 and FC04 (identical results, same over ASCII and RTU). Registers **0–19, 21 and 22** answer;
20 and 23–99 return exception 2 (illegal data address). At the time of the dump, the panel showed
setpoint 18.0 °C and heating was on.

| Reg | Raw | ×0.1 | Guess |
|---:|---:|---:|---|
| 0 | 199 | 19.9 | **room temperature** (panel showed 19.9 °C) |
| 1 | 204 | 20.4 | ? (another air temperature?) |
| 2 | −510 | −51.0 | missing probe (H4, the 4-pipe water probe, is not fitted) |
| 3 | 13, later 9 | | ? (changes) |
| 4 | 11 | | ? |
| 5 | 398 | 39.8 | water temperature? |
| 6 | 408 | 40.8 | water temperature? |
| 7 | −1020 | | missing-probe marker? |
| 8 | 180 | 18.0 | **setpoint** (matches the panel) |
| 9 | 0 | | |
| 10 | 3374 | | ? |
| 11 | 3333 | | ? |
| 12 | 8238 (0x202E) | | bit field? |
| 13 | 419 | 41.9 | water temperature? |
| 14 | 414 | 41.4 | water temperature? |
| 15 | 0 | | |
| 16 | 1023 (0x3FF) | | 10-bit ADC at full scale: open probe input (H4)? |
| 17 | 0 | | |
| 18 | 199 | 19.9 | room temperature again (copy, or filtered/display value?) |
| 19 | 0 | | |
| 21 | 0 | | |
| 22 | 0 | | |

Encodings from Innova's WiFi module API (the [`innova-controls`](https://pypi.org/project/innova-controls/)
Python package, which talks HTTP to the WiFi module, not Modbus). These show what the board knows
and how it encodes values:

| Field | Meaning | Encoding |
|---|---|---|
| `ta` | room temperature | ×0.1 °C |
| `sp` | setpoint | ×0.1 °C, steps of 0.5, 16–31 °C |
| `tw` | water temperature | ×0.1 °C |
| `fn` | fan function | 1 = auto, 2 = night, 3 = min, 4 = max |
| mode | operating mode | 3 = heating, 5 = cooling |
| `kl` | keypad lock | 0/1 |

Contributions welcome: if you have the official register map, or mapped more registers, open an
issue or PR.

## Hardware

- **LilyGo T-CAN485** (ESP32 + RS485 transceiver; power input 5–12 V per LilyGo, or USB-C).
- Short twisted pair from the fancoil's "+ A B −" terminal: **B → A**, **A → B**, **− → GND** (see
  Polarity above). Two devices and a short cable, so no extra termination.
- The board also has **10V-DC +/− pads**, a possible supply for the LilyGo. Measure them under load
  before using them.

## Setup

```bash
cp secrets.yaml.example secrets.yaml   # fill in Wi-Fi, API key, OTA and hotspot passwords
docker run --rm -v "$PWD":/config esphome/esphome compile innova-fancoil.yaml
docker run --rm -v "$PWD":/config --device /dev/ttyACM0 esphome/esphome run innova-fancoil.yaml   # first flash over USB
docker run --rm -v "$PWD":/config esphome/esphome run innova-fancoil.yaml --device <ip>          # later: OTA
```

Then adopt the device in Home Assistant (ESPHome integration, API key from `secrets.yaml`). Its web
UI with the live log is at `http://innova-fancoil.local`.

## Discovery tool

`modbus_discovery.h` is a small non-blocking Modbus master for finding an unknown device. It speaks
**RTU and ASCII** and **only sends read functions 01–04, never writes**. Results go to the ESPHome
log; the **Discovery Status** sensor shows progress. All settings are Home Assistant config
entities and survive a reboot.

| Entity | Use |
|---|---|
| Modbus Baud / Modbus Frame / Modbus Mode | Serial settings (e.g. 9600, 7E1, ASCII), applied right away |
| Modbus Slave Address | Slave to dump/watch; a successful scan fills it in |
| Register Start / Register Count | Range for scan (start only), dump and watch; wire addresses, 0-based (40001 = 0) |
| Dump Functions | 03 + 04 (default), 01 + 02, single ones, or all four |
| Scan Last Address | Scan addresses 1..N (247 = all; 32 is much faster) |
| Reply Timeout | Wait per request beyond the frame time (150 ms) |
| **Scan Addresses** | FC03 to each address at the current settings |
| **Scan All Serial Settings** | Same, through 2400–115200 baud × RTU 8N1/8E1/8O1 and ASCII 7E1/8N1/7N2/7O1 (49 combinations, likely ones first); stops at the first that answers and fills in the settings. About 5 min with 32 addresses, 40 min with 247 |
| **Dump Registers** | Reads the range in blocks of 16 (failed blocks retried per register), logs every readable value with signed and ×0.1 columns |
| **Watch Registers** | Repeats the dump every 2 s and logs only `CHANGE` lines |
| Stop Discovery | Stops any of the above |

A slave counts as found on a normal reply *and* on an exception reply. `invalid reply` warnings mean
something answers but the serial settings are off. While idle, the tool logs any unsolicited bus
traffic, which would mean another master is on the bus.

**Mapping registers:** turn on **Watch Registers**, change one thing on the fancoil's panel
(setpoint, fan speed, mode), and see which `CHANGE` lines appear.

## Roadmap

- [ ] Map registers 0–22 with Watch Registers (setpoint, fan function, mode, on/off, fan speed,
      valve, alarms); try other ranges (100, 1000, …) and FC01/02.
- [ ] Measure the "+" terminal and the 10V-DC pads; maybe power the LilyGo from the fancoil.
- [ ] Replace the discovery tool with ESPHome's `modbus_controller` (RTU, 9600 8N1, slave 1).
- [ ] Read-only sensors in Home Assistant: room and water temperature, setpoint, fan state. Then
      check whether the fancoil's minimum-water-temperature cutoff stops the fan when a heat pump
      runs a low supply temperature (24–30 °C).
- [ ] Optional: a DS18B20 on the water inlet pipe (free pins IO25/32/33, 4.7 kΩ pull-up to 3.3 V)
      as an independent water-temperature reference.
- [ ] Later, maybe: control (e.g. low fan speed while the heat pump runs). Only after the
      register map is known.

## Rules

- **Read-only first:** only functions 01–04 until the register map is known. No writes (05/06/15/16)
  to unknown registers.
- The fancoil must keep working on its own; its own thermostat stays the fallback. Any control
  logic runs on the ESP32, not in Home Assistant.

## License

MIT, see [LICENSE](LICENSE).
