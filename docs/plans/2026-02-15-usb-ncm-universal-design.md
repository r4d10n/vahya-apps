# USB CDC NCM Universal Transport Design

**Date:** 2026-02-15
**Status:** Approved
**Scope:** Replace vendor-specific USB (class 0xFF) with CDC NCM for cross-platform Android + iOS support

---

## 1. Problem Statement

The Vahya transceiver currently uses a vendor-specific USB device class (0xFF) with 4 bulk endpoints for IQ streaming. This works well on Android via the USB Host API but is fundamentally incompatible with iOS, which has no user-space API for accessing vendor-specific USB endpoints without Apple MFi licensing.

**Goals:**
- Support both Android and iOS phones over USB without MFi or root
- Maintain current throughput (128 Mbps / 4.0 MSps dual-band)
- Improve stability and phone compatibility
- Use a single USB device identity across all platforms

**Non-goals:**
- USB 3.0 SuperSpeed (current hardware is USB 2.0 HS via USB3343 ULPI)
- Wireless connectivity (WiFi/BLE)
- Generic network adapter functionality (this is a radio transport, not internet sharing)

---

## 2. Solution: CDC NCM as Universal Transport

Present the Vahya device as a standard USB CDC NCM Ethernet adapter. IQ data and control commands are carried as UDP packets inside Ethernet frames, wrapped in NCM Transfer Blocks (NTBs).

**Why CDC NCM:**

| Property | Vendor-Specific (current) | CDC NCM (proposed) |
|----------|---------------------------|---------------------|
| Android support | USB Host API (works) | USB Host API + userspace NTB parsing (works) |
| iOS support | NO (needs MFi) | YES (native, driverless, iOS 16+) |
| Linux/Mac/Windows | Needs libusb + custom app | Native driver, zero-config |
| USB class | 0xFF (unknown device) | 0x02/0x0D (Ethernet adapter) |
| Stability | Vendor-dependent handling | OS-level power/suspend/error management |
| Protocol overhead | Zero | ~0.6% at 16KB NTBs |

**Why not other USB classes:**
- CDC ACM (serial): iOS has no serial API without MFi. Throughput limited to 1-3 MB/s.
- USB Audio (UAC1): Bandwidth capped at ~1 MB/s. Wrong abstraction for IQ data.
- USB HID: Interrupt transfers, ~800 KB/s max. Not suitable.
- CDC ECM: Less efficient than NCM (no frame aggregation). iOS prefers NCM.
- RNDIS: Being deprecated from Linux kernel. Microsoft proprietary.

---

## 3. USB Descriptor Layout

```
Device Descriptor:
  bcdUSB:             0x0200
  bDeviceClass:       0x02   (Communications)
  bDeviceSubClass:    0x0D   (NCM)
  bDeviceProtocol:    0x00
  idVendor:           0x1209 (pid.codes)
  idProduct:          0x0001
  iManufacturer:      "Vahya"
  iProduct:           "Vahya Radio"
  bMaxPacketSize0:    64

Configuration Descriptor:
  bNumInterfaces:     2
  bmAttributes:       0x80   (bus-powered)
  bMaxPower:          250    (500 mA)

  Interface 0: CDC NCM Control
    bInterfaceClass:    0x02 (Communications)
    bInterfaceSubClass: 0x0D (NCM)
    bInterfaceProtocol: 0x00
    Functional Descriptors:
      CDC Header           (bcdCDC = 0x0120)
      CDC Union            (bControlInterface=0, bSubordinateInterface=1)
      CDC Ethernet Networking:
        iMACAddress:             <string descriptor, globally-administered>
        bmEthernetStatistics:    0x00000000
        wMaxSegmentSize:         1514
        wNumberMCFilters:        0
        bNumberPowerFilters:     0
      CDC NCM:
        bcdNcmVersion:           0x0100
        bmNetworkCapabilities:   0x00
    Endpoint:
      EP1 IN  (Interrupt, wMaxPacketSize=16, bInterval=32ms)

  Interface 1: CDC NCM Data
    Alternate Setting 0: (no endpoints, idle per NCM spec)
    Alternate Setting 1: (active)
      EP2 IN  (Bulk, wMaxPacketSize=512)
      EP3 OUT (Bulk, wMaxPacketSize=512)
```

**MAC address strategy:** Use a globally-administered MAC (bit 1 of first octet = 0). This causes the Linux `usbnet` driver to name the interface `eth0` instead of `usb0`, providing compatibility with Android versions prior to 15 where the `EthernetTracker` regex only matches `eth\d`. The MAC should be derived from a device serial number for uniqueness.

---

## 4. Network Topology

```
Vahya Device                              Phone
192.168.73.1                              192.168.73.2
(DHCP server)                             (DHCP client)

  ┌─────────────┐     USB Cable      ┌─────────────────┐
  │  ECP5 FPGA  │◄──────────────────►│  Android / iOS  │
  │  CDC NCM    │     CDC NCM        │  (USB host)     │
  │  device     │                    │                 │
  └─────────────┘                    └─────────────────┘

UDP Port Map:
  5550  ◄─► Control (SPI read/write, status, LED, reset)
  5551   ─► RX IQ 900 MHz  (device → phone, ~16 MB/s)
  5552   ─► RX IQ 2.4 GHz  (device → phone, ~16 MB/s)
  5553  ◄─  TX IQ 900 MHz  (phone → device)
  5554  ◄─  TX IQ 2.4 GHz  (phone → device)

IP Configuration:
  Subnet:   192.168.73.0/30 (point-to-point, 2 usable addresses)
  Gateway:  none (link-local only)
  DNS:      none
```

---

## 5. NCM Transfer Block (NTB16) Format

All data uses NTB16 framing (simpler than NTB32, universally supported).

### 5.1 Device → Host (RX IQ Data)

```
Offset  Size  Field                Value
──────  ────  ───────────────────  ───────────────────────────
NTH16 Header (12 bytes):
0x0000  4     dwSignature          0x484D434E ("NCMH")
0x0004  2     wHeaderLength        0x000C (12)
0x0006  2     wSequence            <incrementing 0-65535>
0x0008  2     wBlockLength         <total NTB size>
0x000A  2     wNdpIndex            <offset to NDP16>

Datagram 1 — RX 900 MHz IQ (Ethernet + IPv4 + UDP + payload):
0x000C  6     Dst MAC              <phone MAC or FF:FF:FF:FF:FF:FF>
0x0012  6     Src MAC              <device MAC>
0x0018  2     EtherType            0x0800 (IPv4)
0x001A  20    IPv4 Header          src=192.168.73.1, dst=.2, proto=UDP
0x002E  8     UDP Header           src=5551, dst=5551, len=payload+8
0x0036  N     IQ Payload           int16_t LE: I0,Q0,I1,Q1,...

Datagram 2 — RX 2.4 GHz IQ (same structure):
          6     Dst MAC
          6     Src MAC
          2     EtherType            0x0800
          20    IPv4 Header          src=192.168.73.1, dst=.2, proto=UDP
          8     UDP Header           src=5552, dst=5552
          N     IQ Payload           int16_t LE: I0,Q0,I1,Q1,...

NDP16 Datagram Pointer Table:
          4     dwSignature          0x00304D43 ("NCM0")
          2     wLength              24 (header + 2 entries + terminator)
          2     wNextNdpIndex        0x0000
          2     wDatagram0Index      0x000C
          2     wDatagram0Length     <datagram 1 size>
          2     wDatagram1Index      <datagram 2 offset>
          2     wDatagram1Length     <datagram 2 size>
          2     wTerminatorIndex     0x0000
          2     wTerminatorLength    0x0000
```

### 5.2 Host → Device (TX IQ + Control)

Same NTB16 structure. The FPGA's NTB deframer inspects the UDP destination port to route data:

| Dst Port | Action |
|----------|--------|
| 5550 | Control command → SPI/LED/reset handler |
| 5553 | TX IQ 900 MHz → TX FIFO bridge |
| 5554 | TX IQ 2.4 GHz → TX FIFO bridge |

### 5.3 NTB Sizing

```
Max NTB size:           16384 bytes (matches current USB transfer size)
NTH16 overhead:         12 bytes
NDP16 overhead (2 dg):  24 bytes
Eth+IP+UDP per dg:      42 bytes (14+20+8)
Total framing:          12 + 24 + 42*2 = 120 bytes
Max IQ payload:         16384 - 120 = 16264 bytes
  Per band:             ~8100 bytes = ~2025 IQ samples
Efficiency:             99.3%
```

---

## 6. Control Protocol (Replacing EP0 Vendor Requests)

Control commands move from EP0 vendor requests to UDP packets on port 5550:

```
Control Packet Format:
┌────────┬──────────┬───────────────────┐
│ cmd    │ addr     │ data              │
│ 1 byte │ 2 bytes  │ 0-N bytes         │
└────────┴──────────┴───────────────────┘

Commands (host → device):
  0x01  SPI_WRITE      addr=register, data=1 byte value
  0x02  SPI_READ       addr=register, data=none (response follows)
  0x03  GET_STATUS     addr=0, data=none
  0x04  SET_LED        addr=LED bits [2:0], data=none
  0x05  RESET_AT86     addr=0, data=none
  0x06  GET_RX_DEBUG   addr=0, data=none

Responses (device → host):
  0x82  SPI_READ_RESP  addr=register, data=1 byte value
  0x83  STATUS_RESP    addr=0, data=6 bytes (version, spi_busy, led, fifo_status)
  0x86  RX_DEBUG_RESP  addr=0, data=1 byte (rxclk, sync, fifo flags)
```

Control responses are included as an additional datagram in the next outbound NTB, on UDP port 5550.

---

## 7. FPGA Gateware Architecture

### 7.1 Modified Block Diagram

```
RadioTop
├── [UNCHANGED] VahyaPlatform (PLL, pin definitions)
├── [UNCHANGED] AT86 SPI Master
├── [UNCHANGED] AT86 Startup FSM
├── [UNCHANGED] AT86 LVDS RX (x2 bands)
├── [UNCHANGED] AT86 LVDS TX
├── [UNCHANGED] AsyncFIFO CDC Bridges (RX and TX, x2 bands)
│
├── [REPLACED] USB Device
│   ├── NCM Descriptors (replaces vendor-specific descriptors)
│   ├── NCM Control Handler
│   │   ├── GET_NTB_PARAMETERS → returns max NTB size, alignment
│   │   ├── SET_INTERFACE(1, alt=1) → activates data endpoints
│   │   └── Interrupt EP1 IN → NetworkConnection + SpeedChange notifications
│   ├── EP2 IN (Bulk) → NTB frames to host
│   └── EP3 OUT (Bulk) → NTB frames from host
│
├── [NEW] NTB Framer
│   ├── Input: IQ samples from RX FIFO bridges (existing)
│   ├── Constructs NTH16 header (sequence counter)
│   ├── Constructs Ethernet + IPv4 + UDP headers
│   │   (mostly ROM constants, only lengths and checksum vary)
│   ├── Packs IQ samples as UDP payload
│   ├── Appends NDP16 pointer table
│   ├── Dual-band: interleaves two datagrams per NTB
│   ├── Includes control responses as third datagram when pending
│   └── Output: complete NTB → EP2 IN stream interface
│
├── [NEW] NTB Deframer
│   ├── Input: NTB bytes from EP3 OUT stream interface
│   ├── Validates NTH16 signature
│   ├── Parses NDP16 → extracts datagram offsets/lengths
│   ├── For each datagram: strips Eth(14)+IP(20)+UDP(8) = 42 bytes
│   ├── Routes by UDP dst port:
│   │   ├── 5550 → Control command FIFO
│   │   ├── 5553 → TX 900 MHz FIFO bridge
│   │   └── 5554 → TX 2.4 GHz FIFO bridge
│   └── Drops malformed/unknown packets silently
│
├── [NEW] Network Responders
│   ├── ARP Responder
│   │   ├── Matches ARP requests for 192.168.73.1
│   │   └── Replies with device MAC
│   ├── DHCP Server (minimal)
│   │   ├── Listens on UDP port 67
│   │   ├── DISCOVER → OFFER (192.168.73.2, mask /30, lease 86400s)
│   │   └── REQUEST → ACK
│   └── ICMP Echo (optional, for diagnostics)
│       └── Replies to ping requests
│
└── [MODIFIED] Control Handler
    ├── Input: control command FIFO (from NTB deframer)
    ├── Dispatches SPI read/write, LED, reset commands
    ├── Queues responses for NTB framer
    └── Reuses existing SPI master and LED/reset logic
```

### 7.2 Resource Estimate (ECP5-25F: 24K LUTs)

| Module | Est. LUTs | Notes |
|--------|-----------|-------|
| Existing design | ~8000 | USB+SPI+LVDS+FIFOs+PLL |
| NCM descriptors (delta from vendor-specific) | +200 | More descriptor data, same LUNA framework |
| NTB Framer | +400 | Header ROM, MUX, sequence counter, length calc |
| NTB Deframer | +300 | Offset parser, header skip, port demux |
| ARP Responder | +150 | Pattern match + reply builder |
| DHCP Server | +300 | 4-state FSM, option builder |
| NCM Control Handler | +100 | GET_NTB_PARAMETERS, notifications |
| Control-over-UDP handler | +100 | Port from existing vendor_requests.py |
| **Total** | **~9550** | **~40% of ECP5-25F** |

Headroom: ~14K LUTs available for future features.

### 7.3 Clock Domains (unchanged)

| Domain | Frequency | Used By |
|--------|-----------|---------|
| fast | 258 MHz | High-speed internal logic |
| sync | 129 MHz | SPI master, startup FSM, control handler |
| usb | 60 MHz | All USB logic, NTB framer/deframer, network responders |
| at86_rx | 64 MHz | LVDS RX DDR (from AT86RF215 RXCLK) |
| at86_tx | ~64.6 MHz | LVDS TX DDR (sync/2) |

All new NCM modules run in the `usb` domain (60 MHz). The existing AsyncFIFO bridges handle CDC between `at86_rx`/`at86_tx` and `usb` domains — no changes needed.

---

## 8. Android App Changes

### 8.1 New: NcmTransport

Replaces `UsbTransport`. Implements the same `ITransport` interface.

```cpp
class NcmTransport : public ITransport {
    // USB setup
    void open(int fd);          // libusb_wrap_sys_device()
    void close();

    // Claims Interface 1 (NCM Data), selects Alt Setting 1
    // Reads GET_NTB_PARAMETERS from device
    void setupNcm();

    // NTB RX: parse NTH16 → NDP16 → strip Eth+IP+UDP → IQ data
    // Routes by UDP port to appropriate ring buffer
    void rxCallback(libusb_transfer* xfer);

    // NTB TX: wrap IQ data in UDP+IP+Eth → NTB16
    void txCallback(libusb_transfer* xfer);

    // Control: SPI read/write via UDP port 5550 datagrams
    uint8_t spiRead(uint16_t addr);
    void spiWrite(uint16_t addr, uint8_t data);

    // Same ring buffer interface as UsbTransport
    RingBuffer<int16_t> rxRing09_, rxRing24_;
    RingBuffer<int16_t> txRing09_, txRing24_;
};
```

**Estimated size:** ~500 lines C++.

### 8.2 NTB Parsing (core of NcmTransport)

```cpp
// Simplified NTB16 RX parsing
void NcmTransport::parseNtb(const uint8_t* data, int len) {
    // Validate NTH16
    if (read_le32(data) != 0x484D434E) return;  // "NCMH"
    uint16_t blockLen = read_le16(data + 8);
    uint16_t ndpIndex = read_le16(data + 10);

    // Parse NDP16
    const uint8_t* ndp = data + ndpIndex;
    if (read_le32(ndp) != 0x00304D43) return;  // "NCM0"
    uint16_t ndpLen = read_le16(ndp + 4);

    // Iterate datagram pointers
    for (int i = 8; i < ndpLen; i += 4) {
        uint16_t dgIdx = read_le16(ndp + i);
        uint16_t dgLen = read_le16(ndp + i + 2);
        if (dgIdx == 0 && dgLen == 0) break;  // terminator

        // Skip Ethernet(14) + IPv4(20) = 34 bytes to UDP header
        const uint8_t* udp = data + dgIdx + 34;
        uint16_t dstPort = read_be16(udp + 2);
        const uint8_t* payload = udp + 8;
        int payloadLen = dgLen - 42;

        switch (dstPort) {
            case 5551: rxRing09_.write((int16_t*)payload, payloadLen/2); break;
            case 5552: rxRing24_.write((int16_t*)payload, payloadLen/2); break;
            case 5550: handleControlResponse(payload, payloadLen); break;
        }
    }
}
```

### 8.3 Unchanged Components

The following VahyaDroid components require **zero changes:**
- `RadioEngine` (DSP orchestration)
- `FftEngine` (waterfall FFT)
- `Ddc` / `Duc` (digital down/up converter)
- `Demod` / `Modulator` (AM/FM/SSB/CW)
- `CwKeyer` (iambic keyer)
- `AudioEngine` (Oboe)
- All Kotlin UI code (RadioScreen, VFO, waterfall, PTT)
- `RadioViewModel`

### 8.4 USB Device Filter Update

```xml
<!-- res/xml/usb_device_filter.xml -->
<resources>
    <usb-device vendor-id="4617" product-id="1"
        class="2" subclass="13" protocol="0" />
</resources>
```

---

## 9. iOS App Architecture

### 9.1 Overview

```
VahyaRadio.app (iOS, Swift + C++)
├── Network/
│   ├── RadioConnection.swift      — NWConnection lifecycle, .wiredEthernet
│   ├── UdpTransport.swift         — UDP socket wrapper for all 5 ports
│   └── ConnectionMonitor.swift    — NWPathMonitor for Ethernet detection
├── Native/ (C++ via Swift-C++ interop)
│   ├── radio_engine.cpp           — REUSED from VahyaDroid
│   ├── fft_engine.cpp             — REUSED
│   ├── ddc.cpp, duc.cpp           — REUSED
│   ├── demod.cpp, modulator.cpp   — REUSED
│   ├── cw_keyer.cpp               — REUSED
│   └── ios_audio_bridge.cpp       — NEW: AVAudioEngine adapter
├── UI/ (SwiftUI)
│   ├── RadioScreen.swift          — Main radio interface
│   ├── WaterfallView.swift        — Metal-based waterfall display
│   ├── VfoView.swift              — Frequency tuning
│   └── ControlsView.swift        — Mode, band, PTT, CW paddles
└── App/
    └── VahyaRadioApp.swift        — Entry point
```

### 9.2 Network Layer

```swift
// RadioConnection.swift
class RadioConnection: ObservableObject {
    private var rxConn09: NWConnection?
    private var rxConn24: NWConnection?
    private var txConn09: NWConnection?
    private var txConn24: NWConnection?
    private var ctrlConn: NWConnection?

    func connect() {
        let params = NWParameters.udp
        params.requiredInterfaceType = .wiredEthernet

        // RX streams (listen)
        let listener09 = try NWListener(using: params, on: 5551)
        let listener24 = try NWListener(using: params, on: 5552)

        // TX streams (send to device)
        txConn09 = NWConnection(host: "192.168.73.1", port: 5553, using: params)
        txConn24 = NWConnection(host: "192.168.73.1", port: 5554, using: params)

        // Control
        ctrlConn = NWConnection(host: "192.168.73.1", port: 5550, using: params)
    }
}
```

### 9.3 No NTB Parsing Needed

iOS handles all NCM/Ethernet/IP framing in the kernel. The iOS app receives raw UDP payloads = IQ samples directly. This makes the iOS transport code dramatically simpler than Android (~200 lines vs ~500 lines).

---

## 10. Throughput Analysis

```
USB 2.0 HS signaling rate:           480 Mbps
USB 2.0 HS practical bulk max:       ~424 Mbps (~53 MB/s)

NCM overhead per 16KB NTB (2 datagrams):
  NTH16:              12 bytes
  2x Ethernet header: 28 bytes (14 * 2)
  2x IPv4 header:     40 bytes (20 * 2)
  2x UDP header:      16 bytes (8 * 2)
  NDP16 (2 entries):  24 bytes
  Total framing:      120 bytes
  Efficiency:         (16384 - 120) / 16384 = 99.3%

Effective payload throughput:
  53 MB/s * 0.993 = 52.6 MB/s = 421 Mbps

Per-band capacity (2 bands):
  ~26 MB/s = 6.5 MSps (at 4 bytes per IQ sample)

Current requirement:
  4.0 MSps * 4 bytes * 2 bands = 32 MB/s

Utilization: 32 / 52.6 = 60.8%
Headroom: ~20 MB/s (39.2%) available
```

---

## 11. Compatibility Matrix

| Platform | Mechanism | NTB Handling | Tested |
|----------|-----------|-------------|--------|
| Android 15+ | USB Host API (userspace) | App parses NTB in C++ | Pending |
| Android 13-14 | USB Host API (userspace) | Same — bypasses broken kernel | Pending |
| Android 9-12 | USB Host API (userspace) | Same | Pending |
| iOS 16+ (USB-C) | Native NCM driver | Kernel handles NTB | Pending |
| iOS 17+ (iPhone 15+) | Native NCM driver | Kernel handles NTB | Pending |
| Linux desktop | Native cdc_ncm driver | Kernel handles NTB | Pending |
| macOS | Native NCM driver | Kernel handles NTB | Pending |
| Windows 10+ | Native NCM driver | OS handles NTB | Pending |

**MAC address note:** Using a globally-administered MAC ensures Linux names the interface `eth0`, compatible with Android < 15 EthernetTracker regex. Android app bypasses the kernel path entirely regardless.

---

## 12. Stability Improvements

| Concern | Solution |
|---------|----------|
| Android USB suspend | Implement SUSPEND/RESUME in LUNA; send NCM NetworkConnection(0) notification |
| Transfer stalls | NCM spec requires endpoint STALL on errors; implement HALT/CLEAR recovery |
| Phone hot-plug | NCM NetworkConnection notification on interrupt EP; graceful reconnect in apps |
| iOS captive portal | FPGA responds to captive portal probes (HTTP 204 on port 80) or app uses requiredInterfaceType |
| Power draw | bMaxPower=250 (500mA) in config descriptor; proper USB power negotiation |
| Buffer overflow | Device NAKs bulk OUT when TX FIFOs full (standard USB flow control) |
| Data integrity | NTB sequence numbers detect drops; UDP length field validates payload |
| Cable quality | USB 2.0 HS CRC covers all packets; NCM framing adds structural validation |

---

## 13. Migration Path

### Phase 1: FPGA NCM Device
- Replace LUNA vendor-specific descriptors with NCM descriptors
- Implement NTB framer and deframer modules
- Add ARP responder, minimal DHCP server
- Port control handler from EP0 vendor requests to UDP port 5550
- Test with Linux desktop (native `cdc_ncm` kernel driver)

### Phase 2: Android NcmTransport
- Implement `NcmTransport.cpp` (~500 lines)
- Update `usb_device_filter.xml` for CDC class
- Test alongside existing `UsbTransport` with runtime transport selection
- Verify on Android 13, 14, and 15 devices

### Phase 3: iOS App
- `UdpTransport` using `Network.framework` (~200 lines Swift)
- Port C++ DSP engine (reuse VahyaDroid native code)
- SwiftUI radio interface
- Test on iPhone 15/16 (USB-C)

### Phase 4: Cleanup
- Remove vendor-specific USB descriptors from gateware
- Remove `UsbTransport` from Android app
- Update `libvahya` host library for NCM-based Linux operation
- Update documentation
