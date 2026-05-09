# aether_pad — case dimensions reference

Single-source dimensional data for designing the Fusion 360 case.
**Verify critical dimensions with calipers before committing extrusions** —
some values below are from datasheets/typicals and may differ slightly from
the actual hardware on your bench.

## 1. Arduino Giga R1 WiFi (base board)

| Property | Value |
|---|---|
| PCB outline | 101.52 × 53.3 mm |
| PCB thickness | 1.6 mm |
| Form factor | Arduino Mega-compatible footprint |
| Mounting holes | 4 × M3 (Mega-compatible) — **NOT used in this build** (Giga hangs from Shield via stacked headers) |
| USB-C connector | top edge (long side), centred near corner |
| Power jack | top edge, barrel 2.1mm centre-positive (5.5mm OD), accepts 7-24 V |
| Headers | 4 stacks (digital, analog, power, SDA/SCL) — 8.5 mm tall |
| Stack height to shield top of headers | ~14 mm above PCB |

**Verify:** mounting hole XY positions relative to PCB corner (0,0).

## 2. Giga Display Shield (mates on top of Giga)

| Property | Value |
|---|---|
| Shield PCB outline | **106 × 80 mm** (measured 2026-05-03) |
| Display module (LCD + cover glass, visible extent) | **100 × 58 mm** (measured 2026-05-03) |
| Active pixel area | 84 × 50.5 mm (800 × 480 px, 5:3 aspect, inside the display module) |
| Active display offset from PCB **left edge** | **6.3 mm** (measured) |
| Active display offset from PCB **right edge** | 15.7 mm (computed: 106 − 6.3 − 84) |
| Active display offset from PCB **top edge** | **16 mm** (measured 2026-05-03) |
| Active display offset from PCB **bottom edge** | **13 mm** (measured 2026-05-03) |
| Active display height (computed) | 51 mm (80 − 16 − 13, matches 50.5 mm spec) |
| Vertical bias of active within PCB | ~1.5 mm **toward the bottom** |
| Mounting holes | 4 corners, M3, **5 × 5 mm inset** from each PCB edge (measured) |
| **Mounting hole pattern (centre-to-centre)** | **96 × 70 mm** — these are the only case mounting bosses needed |
| **Front-panel window cutout** | **100 × 58 mm** to expose the full display module |
| USB-C connector (on shield) | edge OPPOSITE the 6.3 mm-margin side (right edge in default orientation) |
| Total stack height (Giga + headers + shield) | ~22 mm from Giga PCB bottom to shield top surface |

**IMPORTANT for assembly:** mount the shield so its **6.3 mm-margin edge faces the encoder**. The opposite edge has 15.7 mm of dead PCB beyond the active area, and orienting that side toward the encoder would cost ~9.4 mm of clearance. The flip-mode rotation preserves this — when the top rotates 180°, the 6.3 mm edge still faces the (now opposite-side) knob.

## 3. Autonics E50S8G5-360B-G24N optical encoder

| Property | Value |
|---|---|
| Body diameter | 50 mm |
| Body length (behind panel) | **38.6 mm** (measured 2026-05-03) |
| Total length incl. shaft | ~52 mm |
| Shaft diameter | 8 mm |
| Shaft length | ~13 mm beyond body |
| Mounting | 3 × M3 screws on a **40 mm PCD** (20 mm radius), on the front face — measured |
| Panel mount hole | 32 mm centre clearance (to clear shaft + boss) |
| Wiring | 5-wire pigtail: A, B, Z, +V (12-24V), 0V |
| Operating voltage | 12-24 V DC (uses 13.8 V from shack rail) |
| PPR | 360 |

**For the recessed-knob design:**
- Panel cutout: 32 mm hole for shaft + boss, 3 × M3 tapped or clearance holes on **40 mm PCD**
- Behind-panel clearance needed: **38.6 mm minimum depth** (encoder body length), 50+ mm diameter
- Knob hollow cavity: 52 mm ID, 38.6+ mm deep so it clears the encoder body
- Knob outer dimensions: ~55-60 mm OD typical, 10-15 mm tall above panel
- Knob coupling: 8 mm bore, M3 grub screw onto shaft flat

## 4. CW paddle jack — 3.5 mm TRS, panel mount

| Property | Value |
|---|---|
| Panel hole | 6 mm |
| Body length behind panel | ~15-20 mm (varies by part) |
| Mounting | nut + lock washer on threaded barrel |
| Wiring | tip = dit (D4), ring = dah (D5), sleeve = GND |

## 5. Power entry — **Anderson PowerPole, daisy-chainable**

12 VDC in via Anderson PowerPoles. **Two pairs** (red+black, red+black) on the
rear panel — one pair is the feed in, the second pair allows daisy-chaining the
next shack module on the same supply. Dimensions confirmed against the parts on
hand.

| Property | Value |
|---|---|
| Housing size (single PP15/30/45) | 7.87 × 11.94 × 16.26 mm |
| Red+black pair (clipped together) | 15.75 × 11.94 × 16.26 mm |
| **Cutout per pair** | **16.3 × 18.4 mm** (snug fit on housing, ~2.7 mm clearance for retainer wall) |
| **Retainer sleeve** | **24.6 mm long, 3 mm wall thickness** around the opening — 3D-printed, extends into the case interior |
| Total clearance behind panel | ~25 mm for housing + wire bend |
| Wiring | Wire both pairs in parallel internally (same +/− nodes) so daisy-chain just passes through |

Encoder needs 12-24 V (uses 13.8 V from shack rail); Giga accepts 7-24 V. Single 12 V supply works for both.

**Cutout layout — TBD, two options:**
- **(a)** Two separate 16.3 × 18.4 cutouts with 5-10 mm gap between → flexible, two independent retainers
- **(b)** One combined 16.3 × ~37.3 mm cutout → single retainer block, more compact

Place on rear panel, central horizontally, ~10 mm up from the case base for cable clearance.

## 6. Combined stack — case envelope (symmetric flip-mode, locked)

Sloped desktop unit, screen on the right of the front panel, encoder + recessed
knob on the left, no side cutouts (no USB-C side access — open the case for
reflashing using a USB-C extension cable).

```
       ┌────────────────────────────────────────┐
      ╱                                         ╲      ← front panel plane
     ╱   ⬡        ┌──────────────────┐         ╲    (sloped 10-15° back)
    ╱   knob      │   Display 84×50  │   15mm  ╲
   ╱   recess     │   active area    │   margin ╲
  ╱   (left)      └──────────────────┘           ╲
 └────────────────────────────────────────────────┘   ← base
        │←──────── ~175 mm total width ────────→│
```

Symmetric design — knob and screen window mirror about the case centreline.
Hardware (Giga + Shield + encoder) lives in the **removable top half**, so
rotating the top 180° physically moves them to the opposite side; the base
stays put with rear connectors fixed in position.

**Front panel layout (default orientation, encoder on left):**

```
 ┌────────────────────────────────────────────────────────┐
 │                                                        │
 │  ┌──────┐                          ┌─────────────────┐ │
 │  │      │                          │                 │ │
 │  │ knob │←────── 20 mm gap ────→│  display window │ │
 │  │ Ø60  │                          │   100 × 58 mm   │ │
 │  │      │                          │                 │ │
 │  └──────┘                          └─────────────────┘ │
 │                                                        │
 └────────────────────────────────────────────────────────┘
   │←15→│←──60──→│←─── 20 ───→│←────── 100 ──────→│←15→│
   │←──── 60 ────→│             │←──── 100 ────→│
   │  knob centre at -60       │ window centre at +40
   │←──────────── 210 mm internal ────────────→│
```

| Property | Value |
|---|---|
| Margin: knob outer edge to left wall | 15 mm |
| Margin: window outer edge to right wall | 15 mm |
| Gap between knob and window | **20 mm** (gives 26.7 mm encoder-PCB clearance) |
| Knob centre offset from case centreline | 60 mm (left in default) |
| Window centre offset from case centreline | 40 mm (right in default) |
| Top + bottom margins to window | **17 mm** each (gives ~5 mm PCB clearance) |

**Internal envelope (locked):**

| Dimension | Value | Driver |
|---|---|---|
| Width  | **210 mm** | symmetric about centreline, 60+30+15 each side, 20 gap, screen window 100 |
| Height (perp to slope) | **92 mm** | window 58 + 17 top + 17 bottom margins (PCB ~5 mm clearance) |
| **Slope angle from vertical** | **15°** | nearly upright, compact, comfortable bench viewing |
| **Total interior depth (front to back at base)** | **68.8 mm** | wedge 23.8 + box 45 (box bumped to clear deeper-than-spec encoder body) |
| **Total interior height (vertical)** | **88.9 mm** | = 92 × cos(15°) |

**Side profile (right trapezoid, looking from the right):**

```
       A─────────────────────────────B    A = (23.8, 88.9) — wedge top
      ╱│                             │    B = (68.8, 88.9) — back top
     ╱ │                             │    C = (68.8, 0)    — back bottom
    ╱  │        case                 │    D = (0, 0)       — front bottom
   ╱   │        interior             │
  ╱    │                             │    Slope D→A: 92 mm @ 15° from vertical
 ╱     │                             │    Top   A→B: 45 mm flat (box top)
╱      │                             │    Back  B→C: 88.9 mm vertical
└──────┴─────────────────────────────┘    Base  C→D: 68.8 mm horizontal
       ←── 23.8 ──→←──── 45 ────→
        wedge       box
```

External envelope with 3 mm walls: **216 W × 75 D × 95 H mm**.

**Component placement inside (verified, no 3D conflicts):**

| Component | Front-face / mount position | Extent into case |
|---|---|---|
| Display Shield + Giga stack | parallel to slope, mounted to fascia | 22 mm perpendicular to slope |
| Encoder body | slope midpoint (X=11.9, Y=44.4), Z=−60 from centreline | 38.6 mm along slope normal → back face at (X=49.2, Y=34.4); back-top corner at (X=55.7, Y=58.6) |
| PowerPole sleeves (×2) | back wall, centred Z, centred Y | 24.6 mm into case (X from 44.2 to 68.8) |
| Paddle jack | back wall, centred Z | ~15-20 mm sleeve |
| Flip switch | back wall, near corner | tiny |

The encoder body and PowerPole sleeves overlap in side view (both at similar X), but they're separated in the case-width (Z) direction by ~26 mm — encoder at Z=−60, PowerPoles centred at Z=0. No 3D conflict.

**Encoder back-top to back wall: 13.1 mm clearance** — enough for the 5-wire pigtail strain relief and a 90° cable bend toward the base's central connector.

**Box top (the 45 mm flat region between wedge apex and back wall)** is useful internal volume for routing the cable between the rotating fascia and the base's rear connectors.

**PCB position note:** because the active display is biased ~1.5 mm toward the bottom of the PCB, when you align the display module with the case-centred window, the PCB itself sits ~1.5 mm UPWARD of the case vertical centre. Standoff heights need to account for this if you're putting the PCB on a centred mount.

**After 180° top rotation:** knob ends up on the right at +60, window on the left at −40, all margins preserved. Identical clearances in both orientations.

### Front panel artwork (default orientation, encoder on left)

Front panel size: **210 × 92 mm** (in the plane of the slope). Origin at lower-left, X right, Y up.

**Visible features (cuts through the front face):**

| Feature | Position | Size / type |
|---|---|---|
| Screen window cutout | centre (145, 46) | 100 × 58 mm rectangle, 2-3 mm corner radius |
| Knob shaft clearance | centre (45, 46) | Ø32 mm round cutout |
| Encoder mount #1 (12 o'clock) | (45.0, 66.0) | Ø3.5 mm M3 clearance |
| Encoder mount #2 (4 o'clock) | (62.3, 36.0) | Ø3.5 mm M3 clearance |
| Encoder mount #3 (8 o'clock) | (27.7, 36.0) | Ø3.5 mm M3 clearance |

The 3 encoder mounting holes are 120° apart on a **Ø40 mm PCD** (20 mm radius), measured 2026-05-03. The orientation shown puts one hole at 12 o'clock — rotate the trio if the encoder pigtail needs clearance elsewhere.

**Internal-only features (back side of fascia, no through-holes):**

| Feature | Position | Type |
|---|---|---|
| Shield mounting standoff | (101.7, 82.5) | M3 self-tap or threaded insert |
| Shield mounting standoff | (197.7, 82.5) | (4 corners of shield PCB) |
| Shield mounting standoff | (101.7, 12.5) | |
| Shield mounting standoff | (197.7, 12.5) | |

Standoff heights set the gap between fascia inner surface and PCB top — typically 5 mm to clear the LCD module thickness.

**No screws on the front face** — the fascia attaches via an inner skirt + side screws (see §7).

## 7. Two-half case construction (the rotating-top design)

The case is split horizontally:

- **BASE** = standard, fixed orientation. Rear panel with connectors. Sits on bench. Side walls have screw-engagement holes for the fascia skirt.
- **TOP (fascia)** = removable, contains all active hardware (Giga + Display Shield + encoder + their wiring). Has an **inner skirt** on the underside that drops into the base interior; held captive by **side screws** through the base side walls into the skirt. No screws visible on the front face.

**To flip for friend-mode:** power off, undo side screws, lift top off, unplug internal cable, rotate top 180°, reseat, replug, rescrew, set the rear flip switch, power up. ~60 seconds.

**Mechanical requirements (preserves flip-mode rotational symmetry):**
- Inner skirt outline: **rectangular, centred on the case**, slightly smaller than the base interior (~2 mm under-size all round for a snug locate fit). Skirt depth into base ~10-15 mm.
- Side screws on **mirrored positions left and right** at the same height — typically 2 per side at top and bottom of the engagement region.
- Internal cable connector (5 wires: +12V, GND, paddle dit, paddle dah, flip switch) located on the **case centreline** so both orientations reach it.
- Rear connectors stay on the base, position-independent of top rotation.
- Top must be wide and deep enough internally to hold the hardware in either orientation without collision (drives the 210 × 92 × 53.8 mm interior dimensions in §6).

## 8. Firmware changes needed for flip-mode

Add to `aether_pad.ino`:

```cpp
const int FLIP_PIN = 6;            // rear switch → D6, INPUT_PULLUP
bool flipped = false;              // set in setup() from FLIP_PIN

// In setup(), after pinMode/INPUT_PULLUP on FLIP_PIN:
flipped = (digitalRead(FLIP_PIN) == LOW);
tft.setRotation(flipped ? 3 : 1);
touchMode = flipped ? 1 : 0;       // mode 1 = 180° from default

// In encISR, conditionally invert direction:
encRaw += (flipped ? -1 : +1) * QUAD[(encLast << 2) | now];
```

That's it — ~5 lines plus a pinMode in setup.

## 9. USB-C access — **decided: no external cutout**

No external USB-C access on the case. For reflashing, open the case (or use a
USB-C extension cable threaded through one of the rear cutouts temporarily).
This keeps the side panels clean, simplifies symmetry for flip-mode, and avoids
needing a panel-mount USB-C extension.

## 10. Rear panel — connector layout summary

All external connectors live on the rear panel. Suggested layout (left to right
when viewed from the back):

```
 ┌──────────────────────────────────────────────────┐
 │                                                  │
 │  [PP×4 cutout]    [3.5mm jack]    [flip switch]  │
 │   32×17 mm         ⌀6 mm           if symmetric  │
 │                                                  │
 │  [vent slots]                                    │
 │                                                  │
 └──────────────────────────────────────────────────┘
```

| Connector | Position | Cutout |
|---|---|---|
| PowerPole pair × 2 | rear panel, **two separate cutouts** with ~5-10 mm gap between | 16.3 × 18.4 mm each (24.6 mm sleeves into case) |
| 3.5 mm CW paddle TRS | rear panel, central | ⌀6 mm circular |
| Flip-mode switch | rear panel, near corner | small SPST toggle, ~6 mm hole |
| Ventilation slots | rear panel, lower portion | array of slots, ~10% open area |

## 10. Ventilation

The Giga R1 runs warm (STM32H747 dual-core), especially with WiFi active. Recommend:

- Slot pattern on bottom panel under the Giga
- Slot pattern on rear panel (or wherever the convection exit is on a sloped enclosure)
- 3-5 mm wall-to-PCB-bottom standoff so air can flow under

Total open area ~10% of the relevant face is plenty for passive cooling.

---

## TODO before final case design

- [x] ~~Caliper-verify Giga mounting hole XY pattern~~ → not needed; Giga hangs from Shield via headers. Optional printed support clips between Giga and Shield to take bending load off the header pins.
- [ ] Caliper-verify shield active-display XY relative to mounting holes
- [ ] Caliper-verify shield mic position (if grille wanted on top of case)
- [x] ~~Decide power entry style~~ → Anderson PowerPole, 16.3 × 18.4 mm per pair, 24.6 mm sleeve, 3 mm wall
- [x] ~~Decide USB-C strategy~~ → no external access, open case for reflash
- [x] ~~Decide screen-edge to side-of-case margin~~ → 15 mm right side
- [x] ~~Decide encoder offset~~ → 60 mm from case centreline (default = left), vertically centred
- [x] ~~Decide PowerPole cutout layout~~ → two separate 16.3 × 18.4 cutouts, ~5-10 mm gap between
- [x] ~~Confirm knob OD~~ → 60 mm assumed (verify when knob is sourced/printed)
- [x] ~~Commit to flip-mode~~ → YES, via removable+rotatable top half (see §7)
- [x] ~~Verify shield active display offset within PCB~~ → 6.3 mm from left, 13 mm from top
- [x] ~~Verify display module size~~ → 100 × 58 mm (front-panel window)
- [x] ~~Lock case slope angle~~ → **15° from vertical** (interior 68.8 mm deep, 88.9 mm tall, wedge front + 45 mm box back)
- [x] ~~Caliper-verify encoder body depth and mounting PCD~~ → 38.6 mm deep (was 30 in datasheet typical), 40 mm PCD (was 30) — measured 2026-05-03
- [x] ~~Source or design the recessed knob~~ → 3D-print in 2 parts with pause-print at the steel-ball layer to embed weight (Nigel's design). Confirm final OD when designed — affects case width if >60 mm.
- [ ] Source PowerPole panel-mount retainer (commercial or 3D-printed)
- [ ] Add flip-pin handling to firmware (~5 line change, waits until physical switch is wired)
- [ ] Wire the flip-mode switch on rear panel (D6, INPUT_PULLUP)
