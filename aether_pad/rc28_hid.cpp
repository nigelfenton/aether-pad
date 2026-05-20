#include "rc28_hid.h"

#include "USBHID_Types.h"

namespace AetherPad {

namespace {

// HID Report Descriptor for a vendor-defined 32-byte input report
// carrying Report ID 1. This is intentionally minimal — Icom's real
// RC-28 descriptor includes a richer Usage definition, but the
// AetherSDR parser only cares about the byte layout on the wire, not
// the descriptor's Usage strings.
//
// Total payload on the USB endpoint per report:
//   Report ID (1 byte) + 32 bytes of vendor data = 33 bytes.
// hidapi on Windows/Linux preserves the Report ID; hidapi on macOS
// strips it. This matches the platform behaviour the PR #2870 fix
// has to handle.
static const uint8_t kReportDesc[] = {
    0x06, 0x00, 0xFF,    // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,          // Usage      (Vendor Defined 0x01)
    0xA1, 0x01,          // Collection (Application)
    0x85, 0x01,          //   Report ID (1)
    0x15, 0x00,          //   Logical Minimum (0)
    0x26, 0xFF, 0x00,    //   Logical Maximum (255)
    0x75, 0x08,          //   Report Size (8 bits)
    0x95, 0x20,          //   Report Count (32 bytes)
    0x09, 0x01,          //   Usage (Vendor Defined 0x01)
    0x81, 0x02,          //   Input (Data, Var, Abs)
    0xC0                 // End Collection
};

constexpr uint16_t kIcomVendorId   = 0x0C26;
constexpr uint16_t kRc28ProductId  = 0x001E;
constexpr uint16_t kProductRelease = 0x0100;

// On-wire data payload size (not including the Report ID byte) — matches
// PR #2870's corrected report size, so the parser sees the layout it
// expects on Windows/Linux (with ID byte) and macOS (without).
constexpr uint8_t kDataPayloadBytes = 32;

// Inter-report spacing for paired seq=0x01 / seq=0x02 reports.  The
// real RC-28 sends these within a single USB poll interval; we
// approximate with a short delay between back-to-back reports.
constexpr unsigned long kInterReportDelayUs = 1000;   // 1 ms

} // namespace

Rc28Hid::Rc28Hid()
    : arduino::USBHID(
          /* connect_blocking      */ false,
          /* output_report_length  */ 1,                       // we don't receive output reports
          /* input_report_length   */ kDataPayloadBytes,       // 32 bytes of vendor data
          /* vendor_id             */ kIcomVendorId,
          /* product_id            */ kRc28ProductId,
          /* product_release       */ kProductRelease)
{
}

const uint8_t* Rc28Hid::report_desc()
{
    return kReportDesc;
}

uint16_t Rc28Hid::report_desc_length()
{
    return sizeof(kReportDesc);
}

void Rc28Hid::buildReport(uint8_t* out, uint8_t seq, Rc28Dir dir, Rc28Button btn)
{
    // out[0]      = Report ID
    // out[1..32]  = RC-28 payload (parser's data[0..31])
    out[0] = 0x01;                              // Report ID 1
    // Zero the payload first
    for (uint8_t i = 1; i <= kDataPayloadBytes; ++i) out[i] = 0x00;
    // Populate the fields the parser actually reads.
    out[1 + 0] = seq;                           // data[0] : seq counter
    out[1 + 2] = static_cast<uint8_t>(dir);     // data[2] : direction
    out[1 + 4] = static_cast<uint8_t>(btn);     // data[4] : button state
    // bytes [1], [3], [5..31] remain 0x00 as the real device leaves them.
}

bool Rc28Hid::sendBuilt(const uint8_t* data33)
{
    HID_REPORT r;
    r.length = 1 + kDataPayloadBytes;           // 33 bytes total
    if (r.length > sizeof(r.data)) return false;
    for (uint32_t i = 0; i < r.length; ++i) r.data[i] = data33[i];
    return send_nb(&r);
}

void Rc28Hid::sendDetent(Rc28Dir dir, Rc28Button btn)
{
    uint8_t buf[33];
    // Pair 1 — seq=0x01 (the report the parser will act on)
    buildReport(buf, 0x01, dir, btn);
    sendBuilt(buf);

    // Real RC-28 spaces the two reports of a detent slightly apart.
    delayMicroseconds(kInterReportDelayUs);

    // Pair 2 — seq=0x02 (parser ignores, but device sends for fidelity)
    buildReport(buf, 0x02, dir, btn);
    sendBuilt(buf);
}

void Rc28Hid::sendButton(Rc28Button btn)
{
    // Buttons are a single-report event in the RC-28 protocol — no
    // direction, no detent pair.  Use seq=0x01 so the parser sees it.
    uint8_t buf[33];
    buildReport(buf, 0x01, Rc28Dir::None, btn);
    sendBuilt(buf);
}

void Rc28Hid::sendIdle()
{
    uint8_t buf[33];
    buildReport(buf, 0x01, Rc28Dir::None, Rc28Button::Idle);
    sendBuilt(buf);
}

void Rc28Hid::sendMacOsEdgeCase()
{
    // The macOS bug aethersdr-agent caught: hidapi strips the Report ID
    // on macOS, so the byte AetherSDR sees as buf[0] is actually seq
    // counter — which on the first report of a detent is 0x01. The
    // current PR fix's heuristic `buf[0]==0x01 ? buf+1 : buf` then
    // wrongly strips a "Report ID" that isn't there, mis-aligning the
    // parse on every first-report-of-detent.
    //
    // We can't directly cause hidapi on the host to strip; that's a
    // host-side behaviour. But we CAN send the exact wire bytes that
    // *would* expose the bug when received on a macOS host. So this
    // helper just emits a normal CW detent — the bug manifests on the
    // *host parser*, not on our wire output. Logging the call here
    // marks the report as the "trigger" stimulus for diff against the
    // post-fix parser output.
    Serial.println("RC28: sending macOS-edge-case CW detent (seq=0x01 trigger)");
    sendDetent(Rc28Dir::Cw, Rc28Button::Idle);
}

} // namespace AetherPad
