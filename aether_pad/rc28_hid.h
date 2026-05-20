/*
 * rc28_hid.h
 *
 * USB HID device that impersonates the Icom RC-28 tuning controller for the
 * purpose of hardware-in-the-loop testing of AetherSDR's IcomRC28Parser
 * (see ten9876/AetherSDR PR #2870).
 *
 * Identifies as:
 *   VID  = 0x0C26 (Icom)
 *   PID  = 0x001E (RC-28)
 *   bcdDevice = 0x0100
 *
 * Report layout (32 bytes data + 1 byte Report ID = 33 bytes on wire on
 * Windows/Linux; 32 bytes after hidapi strip on macOS), per the corrected
 * layout in PR ten9876/AetherSDR#2870:
 *
 *   offset (0-indexed, after Report ID)   meaning
 *   0                                      seq counter (0x01 or 0x02)
 *   1                                      unused (0x00)
 *   2                                      direction (0x00 idle, 0x01 CW, 0x02 CCW)
 *   3                                      unused (0x00)
 *   4                                      buttons (idle=0x07, F1=0x05, F2=0x03, TX=0x06)
 *   5..31                                  unused (0x00)
 *
 * The RC-28 sends TWO reports per detent — seq 0x01 then seq 0x02. The
 * parser only acts on seq=0x01 (per the PR fix), so we mirror that
 * behaviour: every helper that "sends a detent" emits both reports back
 * to back so the parser sees a real detent on the wire.
 *
 * Build context:
 *   - Arduino mbed_giga 4.5.0 (STM32H747)
 *   - Library: USBHID (built into the core, not Adafruit_TinyUSB)
 *   - The mbed PluggableUSBDevice infrastructure handles enumeration; we
 *     subclass arduino::USBHID and override report_desc() to declare our
 *     own vendor-defined 32-byte input report.
 *
 * Ethical posture (matches design doc §13 for ShackLog and the broader
 * project pattern): we ARE impersonating Icom's VID/PID, but only on
 * Nigel's bench, only for in-house testing of AetherSDR's parser. If
 * this firmware ever ships publicly, switch to a non-Icom VID/PID and
 * add a matching parser entry in AetherSDR's HidDeviceParser table.
 */

#pragma once

#include <Arduino.h>
#include "PluggableUSBHID.h"

namespace AetherPad {

// Button enum values as they appear in report data[4] on the wire.
enum class Rc28Button : uint8_t {
    Idle = 0x07,
    F1   = 0x05,
    F2   = 0x03,
    Tx   = 0x06
};

// Direction enum as it appears in report data[2].
enum class Rc28Dir : uint8_t {
    None = 0x00,
    Cw   = 0x01,
    Ccw  = 0x02
};

class Rc28Hid : public arduino::USBHID {
public:
    Rc28Hid();

    // High-level helpers — pre-pack the report and send via send_nb().

    // Send a single detent: emits the seq=0x01 then seq=0x02 pair.
    void sendDetent(Rc28Dir dir, Rc28Button btn = Rc28Button::Idle);

    // Send a button event (press then release on next call).
    // Sends a single report with seq=0x01, dir=None.
    void sendButton(Rc28Button btn);

    // Send a raw idle report (seq=0x01, all data zero except buttons=0x07).
    // Useful as a "device is alive" pulse.
    void sendIdle();

    // Special: drive the exact byte sequence that exposes the macOS
    // edge-case bug aethersdr-agent flagged on PR #2870 (seq counter
    // appearing as buf[0] on hidapi-strip platforms, colliding with
    // the report-ID heuristic). Emits one CW detent worth of reports
    // but with a deliberately marked status so test logs can identify
    // it in the audit trail.
    void sendMacOsEdgeCase();

protected:
    // Override the default 27-byte mbed descriptor with our vendor-
    // defined 32-byte input report.
    const uint8_t* report_desc() override;
    uint16_t       report_desc_length() override;

private:
    // Build one report into `out` (must be at least 33 bytes). Sets
    // out[0] = Report ID (1), then 32 bytes of RC-28 payload starting
    // at out[1] which corresponds to "data[0]" in the parser.
    static void buildReport(uint8_t* out,
                            uint8_t seq,
                            Rc28Dir dir,
                            Rc28Button btn);

    // Send a fully-built 33-byte report (Report ID + 32 payload).
    bool sendBuilt(const uint8_t* data33);
};

} // namespace AetherPad
