# aether_pad

A standalone hardware control surface for [AetherSDR](https://github.com/ten9876/AetherSDR), built on an Arduino Giga R1 WiFi + Display Shield. One self-contained box gives you VFO tuning, mode/volume/band quick-control via a touchscreen, and a CW iambic keyer — all talking to AetherSDR over its TCI WebSocket.

Ops by **G0JKN / W3**.

## Features

- **Auto-discovery** of AetherSDR on the LAN (UDP probe → unicast reply on port 40002)
- **TCI WebSocket** client — hand-rolled framing, no external libraries beyond stock Arduino WiFi / Display
- **Optical encoder** for VFO tuning with software divider and configurable Hz/click
- **8-button touchscreen** (2 × 4 grid) for VOL / BAND / MODE / VFO ± controls
- **CW iambic keyer** (Mode A or B) — paddles wired to a 3.5 mm jack, drives the radio via TCI `keyer:` commands
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
       (optional) │                                 │
                  ├──── D6 → buffer (NPN or opto)   │
                  │     → radio key jack            │
                  │     (see "CW key output" below) │
                  └─────────────────────────────────┘
```

Touch transform `tx = raw_y, ty = 479 - raw_x` is verified for landscape mounting (long axis horizontal, USB-C on the right). If you mount differently, run `tm 0..3` over serial to flip.

## Network

- **WiFi SSID/pass** is hardcoded near the top of `aether_pad.ino` (`tinkerbell` by default — change to your shack network).
- **Discovery** broadcasts `AETHERPAD?` on the local /24 subnet, UDP port 40002. AetherSDR ≥ v0.9.5.1 (with the [TciServer responder patch](https://github.com/nigelfenton/AetherSDR)) replies unicast `AETHERSDR ip=<addr> tci=<port>`.
- **Fallback** after 5 s without a reply: `10.0.0.107:40001`. Override at runtime with `ip <addr>` over serial.

## Building

Arduino IDE 2.x with the **Arduino Mbed OS Giga Boards** package installed. Required libraries (all from the IDE library manager or auto-installed by the board package):

- `Arduino_H7_Video`
- `Arduino_GigaDisplay_GFX`
- `Arduino_GigaDisplayTouch`

Open `aether_pad/aether_pad.ino`, select the **Arduino Giga R1** board, plug the USB-C, hit **Upload**.

On boot, serial console at 115200 will show:

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
Squeeze paddles, send dits/dahs. Default 25 WPM, Iambic Mode B. Adjust with `wpm N` and `iambic A|B` over serial. The Giga handles all element timing and emits `keyer:0,true;` / `keyer:0,false;` to AetherSDR.

> **Status note:** AetherSDR's TCI CW path was reportedly flaky as of 2026-05-03. The keyer state machine itself is correct — if TCI keying doesn't drive the radio, you can also drive a real radio's key input directly from a GPIO pin (see below).

### CW key output — driving a real radio (optional)

If you want the keyer to drive a physical radio's key input directly — instead of, or *in addition to*, the TCI command going to AetherSDR — wire a buffer from a free GPIO (default suggested: **D6**) into your radio's key jack. The keyer's element timing is the same either way; only the wire path changes.

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

#### Firmware change (~5 lines)

The single `keyerOutput()` function in `aether_pad.ino` controls the wire path — touching that one function lets you add or swap the output without changing any of the keyer state machine.

Add a pin constant near the other hardware pins:

```cpp
const int KEY_OUT_PIN = 6;
```

In `setup()`, configure the pin as an output:

```cpp
pinMode(KEY_OUT_PIN, OUTPUT);
digitalWrite(KEY_OUT_PIN, LOW);
```

In `keyerOutput()`, add the GPIO drive line alongside (or in place of) the TCI send:

```cpp
void keyerOutput(bool down) {
    if (kr.keyDown == down) return;
    kr.keyDown = down;
    digitalWrite(KEY_OUT_PIN, down ? HIGH : LOW);              // hardware key out
    wsSendText(down ? "keyer:0,true;" : "keyer:0,false;");     // also via TCI (optional)
    if (down) kr.txCount++;
    uiNeedsRedraw = true;
}
```

Keep both paths active (TCI for AetherSDR, GPIO for the radio) or comment out the `wsSendText` line if you only want the hardware key out.

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
