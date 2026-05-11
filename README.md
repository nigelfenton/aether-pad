# aether_pad

A standalone hardware control surface for [AetherSDR](https://github.com/ten9876/AetherSDR), built on an Arduino Giga R1 WiFi + Display Shield. One self-contained box gives you VFO tuning, mode/volume/band quick-control via a touchscreen, and a CW iambic keyer — all talking to AetherSDR over its TCI WebSocket.

Ops by **G0JKN / W3**.

## Features

- **Auto-discovery** of AetherSDR on the LAN (UDP probe → unicast reply on port 40002)
- **TCI WebSocket** client — hand-rolled framing, no external libraries beyond stock Arduino WiFi / Display
- **Optical encoder** for VFO tuning with software divider and configurable Hz/click
- **8-button touchscreen** (2 × 4 grid) for VOL / BAND / MODE / VFO ± controls
- **CW iambic keyer** (Mode A or B) — paddles on a 3.5 mm jack, drives the radio's key line through a **GPIO output (D6)** for sub-microsecond, jitter-free timing.  Optional TCI echo (`keyer:0,true|false;`) can be enabled at compile time for AetherSDR-side UI awareness.
- Live status display on the 800 × 480 panel: frequency, mode, volume, band, encoder diagnostics
- Falls back to a hardcoded host IP if discovery times out — works against vanilla AetherSDR (≥ v0.9.5.1) once the responder patch lands

## Hardware

| Part | Notes |
|---|---|
| Arduino Giga R1 WiFi | STM32H747, dual-core M7+M4, USB-C |
| Arduino Giga Display Shield | 800 × 480 panel, GT911 capacitive touch |
| Optical rotary encoder | **LPD3806-600BM-G5-24C** (600 PPR, NPN open-collector, 5–24 V) — runs off Giga 5 V. Autonics E50S-360B (360 PPR, 12–24 V) is also supported as a compile-time alternate via `#define ENCODER_AUTONICS_E50S` near the top of the sketch. |
| 2 × **10 kΩ** resistors | Pull-ups for encoder A (D2) and B (D3) to **3.3 V — never 5 V**. **Required**, not optional — the Giga's internal `INPUT_PULLUP` is too weak to firmly pull these lines HIGH between transistor pulses. |
| Optional: 2 × 10 kΩ resistors | Paddle pull-ups for D4/D5 to 3.3 V if you see noise on long paddle cables. The internal `INPUT_PULLUP` is fine for short paddle leads since a paddle is a passive switch (no leakage current). |
| Optional: 2 × 100 pF caps | D2 / D3 to GND, additional EMI filtering |
| 3.5 mm stereo TRS jack | CW iambic paddle input (or straight key — tip + sleeve only) |
| Encoder supply | 5 V from the Giga **5V** pin if using LPD3806; external 13.8 V (shack rail) if using Autonics E50S — its LED will not light below 12 V |

## Wiring

```
                  Giga R1 WiFi (with Display Shield)
                  ┌─────────────────────────────────┐
                  │                                 │
       Encoder    │                                 │
       ┌────┐     │                                 │
       │ +V ├──── 5 V  (LPD3806 — Giga 5V pin)      │
       │    │     or 13.8 V (Autonics E50S only)    │
       │ 0V ├──── GND ────────── share with Giga    │
       │ A  ├──── D2  (10 kΩ pull-up to 3.3 V)      │
       │ B  ├──── D3  (10 kΩ pull-up to 3.3 V)      │
       └────┘     │                                 │
                  │   pull-ups go to 3.3 V, NEVER   │
                  │   5 V — Giga inputs aren't 5 V  │
                  │   tolerant                      │
                  │                                 │
       Paddle     │                                 │
       (3.5 mm    │                                 │
        TRS)      │                                 │
       ┌────┐     │                                 │
       │ tip├──── D4   (INPUT_PULLUP, dit)          │
       │ rng├──── D5   (INPUT_PULLUP, dah)          │
       │ slv├──── GND                               │
       └────┘     │                                 │
                  │   (straight-key mode discussed, │
                  │   not yet in firmware — for now │
                  │   the iambic state machine will │
                  │   try to interpret a held key)  │
                  │                                 │
       Radio key  │                                 │
                  ├──── D6 → NPN or opto buffer     │
                  │     → radio CW key jack         │
                  │     (see "CW key output" below) │
                  └─────────────────────────────────┘
```

Touch transform `tx = raw_y, ty = 479 - raw_x` is verified for landscape mounting (long axis horizontal, USB-C on the right). If you mount differently, run `tm 0..3` over serial to flip.

## Network

- **WiFi SSID/pass** is hardcoded near the top of `aether_pad.ino` (`tinkerbell` by default — change to your shack network).
- **Discovery** broadcasts `AETHERPAD?` on the local /24 subnet, UDP port 40002. AetherSDR ≥ v0.9.5.1 (with the [TciServer responder patch](https://github.com/nigelfenton/AetherSDR)) replies unicast `AETHERSDR ip=<addr> tci=<port>`.
- **Fallback** after 5 s without a reply: `10.0.0.107:40001`. Override at runtime with `ip <addr>` over serial.

## Install — build and flash the firmware

You'll need an Arduino Giga R1 WiFi, a USB-C cable, and the Display
Shield mounted. Everything below runs in the **Arduino IDE** — no
command-line compiler to wrestle with.

### 1. Install Arduino IDE

Download **Arduino IDE 2.x** from <https://www.arduino.cc/en/software>
and run the installer (Windows / macOS / Linux all supported).

### 2. Add Giga R1 board support

In the IDE:

1. **Tools → Board → Boards Manager…**
2. Search for **`Mbed OS Giga`**
3. Install **Arduino Mbed OS Giga Boards** (latest version)

The board package pulls in the Display Shield libraries automatically:

- `Arduino_H7_Video`
- `Arduino_GigaDisplay_GFX`
- `Arduino_GigaDisplayTouch`

If any are missing, install them via **Sketch → Include Library → Manage Libraries…**

### 3. Download the firmware

Either clone the repo:

```sh
git clone https://github.com/nigelfenton/aether-pad.git
```

…or click the green **Code → Download ZIP** button at the top of this
page and unzip it.

### 4. Set your WiFi credentials

Open `aether_pad.ino` in the IDE and edit the two lines near the top:

```cpp
#define WIFI_SSID  "your-network"
#define WIFI_PASS  "your-password"
```

The Giga's WiFi radio is **2.4 GHz only** — point it at your 2.4 GHz SSID,
not a 5 GHz one.

### 5. Plug in the Giga R1

Connect the Giga to your computer with the USB-C cable. After a few
seconds it should appear as a new serial port.

### 6. Pick the board and port

- **Tools → Board → Arduino Mbed OS Giga Boards → Arduino Giga R1**
- **Tools → Port → *<the new port that just appeared>***
  - Windows: typically `COM4` / `COM5` / etc.
  - macOS: `/dev/cu.usbmodem…`
  - Linux: `/dev/ttyACM0`

### 7. Upload

Click the **→** Upload button (top-left of the IDE window). The first
flash takes 1–2 minutes — subsequent ones are quicker. Watch for
**"Done uploading."** at the bottom of the IDE.

### 8. Confirm it booted

Open **Tools → Serial Monitor** and set the baud rate to **115200**
(dropdown bottom-right of the Monitor window). You should see something
like:

```
aether_pad — booting
Build: <date> <time>
WiFi: ........
WiFi connected, IP = 10.0.0.220
Discovery probe → 10.0.0.255
Discovery reply: AETHERSDR ip=10.0.0.107 tci=40001
AetherSDR @ 10.0.0.107:40001
TCI: connecting to 10.0.0.107:40001
TCI: WebSocket handshake OK
```

The touchscreen will light up showing frequency / mode / volume cards
once TCI is connected.

### Troubleshooting

- **No port shows under Tools → Port.** Try a different USB-C cable —
  some are charge-only and carry no data. Unplug, replug, restart the
  IDE if needed.
- **"Board not in selected port" / upload fails.** Double-press the
  Giga's small reset button to force USB bootloader mode, then re-select
  the new port (it will change) and Upload again.
- **WiFi never connects.** Double-check SSID/password spelling. Confirm
  you're on the 2.4 GHz SSID, not 5 GHz.
- **TCI never connects.** In AetherSDR, open **Tools → TCI** and make
  sure the TCI server is enabled on port **40001**. The Giga and the
  AetherSDR host need to be on the same LAN.

## Operating

### Encoder
Spin the knob to tune the VFO. Default step is **100 Hz/click** (fine SSB tuning). Change with `step N` over serial — common values: `step 10`, `step 100`, `step 1000`, `step 10000`. The same step value is shared with the on-screen VFO± buttons.

### Touch buttons (2 × 4 grid)

| Top row | Bottom row |
|---|---|
| `VOL+`  (+5 %) | `VOL−`  (−5 %) |
| `BAND+` (next ham band)¹ | `BAND−` (previous)¹ |
| `MODE+` (next mode in `am sam dsb lsb usb cw digl digu fm dfm spec rtty`) | `MODE−` |
| `VFO+`  (step by current `step`) | `VFO−` (step by current `step`) |

**Long press** (~600 ms) on `VFO+` or `VFO−` cycles the step size:
**100 Hz → 1 kHz → 10 kHz → 100 kHz → wrap.** The current value is shown in the **STEP** card, and the encoder uses the same value, so spinning the knob and tapping the VFO buttons always agree.

¹ *Status note (2026-05-04):* AetherSDR's TCI server doesn't currently implement `band_up`/`band_down` commands, so the BAND± buttons do nothing. Tracked as a known issue.

### CW iambic keyer
Squeeze paddles, send dits/dahs.  Default 25 WPM, Iambic Mode B.  Adjust with `wpm N` and `iambic A|B` over serial.  The Giga handles all element timing in software and drives the radio's key line directly via **D6** — TCI is bypassed entirely for the timing-critical wire path because WebSocket-over-WiFi latency and jitter made dit/dah length audibly unstable even at modest speeds.

> **Optional TCI echo:** if you want AetherSDR to also see the key events for sidetone / UI / metering purposes, define `KEYER_TCI_ECHO 1` at the top of the sketch (or set `-DKEYER_TCI_ECHO=1` at compile time).  The hardware key line is still authoritative for actual radio keying — TCI just gets informational copies.

### CW key output (D6 — default since v0.2)

D6 goes HIGH on key-down and LOW on key-up.  Wire a small buffer between D6 and the radio's CW key jack — never connect the Giga GPIO directly to a key line, since some radios bias the line to +12 V or higher.  Two interface options depending on how isolated you want the Giga from the radio:

Two hardware options depending on how isolated you want the Giga from the radio:

#### Option 1 — NPN buffer (simple, ~£0.10)

Fine for modern transceivers with safe key-line voltages (typically 3–15 V open-circuit, low current). Shares ground with the radio.

```
                                        ┌──────── Radio KEY tip
   Giga D6 ──[ 1 kΩ ]── B (NPN)         │
                        │                C
                        │           ┌────┴────┐
                        E ── GND ───┤   NPN   │
                        (Giga)      │  BC547  │
                                    │ 2N3904  │
                                    │ 2N2222  │
                                    └─────────┘
                                         E ──── GND (shared with radio)
```

- **D6 HIGH** → transistor ON → key line pulled to ground = **key down**
- **D6 LOW**  → transistor OFF → key line floats = **key up**
- Optional 10 kΩ from base to emitter keeps the transistor firmly OFF if D6 ever goes high-impedance.

#### Option 2 — Opto-isolator (proper galvanic isolation, ~£0.50)

Recommended if you're keying a high-power amplifier directly, a tube rig where the key line could float at +30 V or more, or you've ever seen ground-loop or RF-induced weirdness in the shack. The Giga and radio share **no** ground.

```
                                ┌────────── Radio KEY tip
   Giga D6 ──[ 330 Ω ]── A      │
                         │      C
                         │ ┌────┴────┐
                         K │  4N35   │
                         │ │  PC817  │
                         │ │  4N25   │
                         │ └────┬────┘
                         GND    E ──── Radio GND (NOT Giga GND)
                        (Giga)
```

#### Firmware

The keyer's wire path lives in a single `keyerOutput()` function in `aether_pad.ino`.  Default behaviour (since v0.2) is GPIO-only:

```cpp
void keyerOutput(bool down) {
    if (kr.keyDown == down) return;
    kr.keyDown = down;
    digitalWrite(KEY_PIN_OUT, down ? HIGH : LOW);
#if KEYER_TCI_ECHO
    wsSendText(down ? "keyer:0,true;" : "keyer:0,false;");
#endif
    if (down) kr.txCount++;
    uiNeedsRedraw = true;
}
```

The state machine is fully decoupled from the wire — to add MIDI keying, a second hardware output, or anything else, edit only this function.

## Serial commands

All commands at **115200 8-N-1**.

| Command | Effect |
|---|---|
| `?` | Print full state (WiFi/TCI/freq/mode/vol/band/encoder/keyer) |
| `ip <a.b.c.d>` | Override AetherSDR IP and force reconnect |
| `freq <Hz>` | Tune VFO directly |
| `step <Hz>` | Set the shared step size (10 / 100 / 1000 / 10000 typical). Used by both the encoder and the on-screen VFO± buttons. |
| `vol <0..100>` | Set volume directly (AetherSDR audioGain percent — *not* dB) |
| `div <N>` | Set encoder divider (raw quadrature counts per emitted step). Defaults: **40** for LPD3806, **24** for Autonics E50S. |
| `tm 0..3` | Touch transform: 0 = default (verified), 1 = 180°, 2 = raw, 3 = flip-both |
| `wpm <5..100>` | CW keyer speed |
| `iambic A\|B` | Iambic mode (A = current-element-only / no memory; B = opposite-paddle memory, default) |

## Architecture

```
┌─────────────┐  WiFi (UDP 40002)  ┌──────────┐
│  aether_pad ├───── discovery ───►│          │
│   (Giga)    │◄──── reply ────────┤          │
│             │                    │ AetherSDR│
│             │   WS TCI :40001    │          │
│             ├────────────────────►          │
│             │◄────── echoes ─────┤          │
└─┬─┬─┬─┬─────┘                    └──────────┘
  │ │ │ └── touchscreen UI (display + 8 buttons)
  │ │ └──── optical encoder (D2/D3, INT-driven, quadrature)
  │ └────── CW paddle (D4/D5, polled, iambic state machine)
  └──────── serial console (115200, debug + control)
```

The TCI client is hand-rolled WebSocket framing over `WiFiClient`, same minimal approach as [ShackController](https://github.com/nigelfenton/shackcontroller). Outbound frames: `0x81 [len] [payload]` (FIN + text, unmasked — AetherSDR is permissive). Inbound parser handles both masked and unmasked text frames; close frame triggers reconnect. No external WebSocket library needed.

## File map

```
aether_pad/
├── aether_pad.ino       ← all firmware, single sketch
├── README.md            ← this file
├── CASE_DIMENSIONS.md   ← physical case build notes
├── CASE_DIMENSIONS.pdf  ← rendered version of the above
├── md_to_pdf.py         ← helper to regenerate the PDF
├── Aether pad.step      ← Fusion-exported 3D model of the front panel
├── Aether pad.stl       ← latest STL for the slicer
└── lpd3806_panel.dxf    ← 2D sketch of the encoder cutouts (boss + 30 mm PCD)
```

## Future work

- Memory keyer (preprogrammed CQ / 73 / name+QTH macros via `cw_msg:` or `cw_macros:`)
- On-device WPM adjustment via touch (long-press an existing button to enter "CW mode")
- Web UI on port 80 (HTML scaffold is dormant in the source — about 50 lines from working)
- Migrate keyer tick to a hardware timer ISR if loop jitter affects element edges

## Licence

Same as the rest of `shack-experiments` — pick whatever G0JKN's repo licence is.
