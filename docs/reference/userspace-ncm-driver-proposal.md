# Open Source Proposal: Android Userspace USB NCM Driver

**Date:** 2026-02-15
**Project Name:** `usb-ncm-android` (proposed)
**License:** Apache 2.0 or MIT
**Status:** Proposal

---

## Problem Statement

USB CDC NCM Ethernet adapters do not work on stock Android phones (versions 9-14) because of a framework-level bug in `EthernetTracker.java`. The regex `eth\d` rejects CDC Ethernet interfaces named `usb0` by the Linux kernel. While this is fixed in Android 15+, billions of phones running Android 9-14 remain affected.

The kernel-level fix cannot be backported because the `usb0` interface name collides with USB tethering interfaces. Custom ROMs (LineageOS, GrapheneOS) have declined to patch this independently, deferring to upstream.

**No existing solution enables standard USB Ethernet adapters on unrooted Android phones running Android 9-14.**

---

## Proposed Solution

A standalone Android app that acts as a **userspace USB NCM driver** by combining two existing Android APIs:

1. **USB Host API** (`android.hardware.usb`) — claims the NCM data interface, reads/writes NTB frames via bulk endpoints
2. **VpnService** (`android.net.VpnService`) — creates a TUN interface, captures/injects all IP traffic

The app bridges IP packets between the TUN interface and the USB NCM device, effectively creating a network adapter in userspace.

```
┌───────────────┐     ┌────────────────────────────────┐     ┌───────────────┐
│  Any Android  │     │  usb-ncm-android app           │     │  USB NCM      │
│  App          │     │                                │     │  Device       │
│               │     │  ┌────────┐    ┌───────────┐   │     │               │
│  TCP/UDP ────>│────>│  │  TUN   │───>│  NTB      │──>│────>│  Bulk OUT     │
│               │     │  │  fd    │    │  Builder   │   │     │               │
│  TCP/UDP <────│<────│  │        │<───│  NTB      │<──│<────│  Bulk IN      │
│               │     │  │        │    │  Parser    │   │     │               │
│               │     │  └────────┘    └───────────┘   │     │               │
└───────────────┘     └────────────────────────────────┘     └───────────────┘
```

---

## Architecture

### Core Components

```
usb-ncm-android/
├── app/src/main/
│   ├── java/com/usbncm/
│   │   ├── NcmService.kt              — Foreground service (VpnService subclass)
│   │   ├── NcmUsbManager.kt           — USB device detection, permission, lifecycle
│   │   ├── NcmBridge.kt               — Orchestrates TUN ↔ USB data flow
│   │   ├── ui/
│   │   │   ├── MainActivity.kt        — Status UI, connection info, toggle
│   │   │   └── SettingsActivity.kt    — IP config, MTU, auto-connect
│   │   └── receiver/
│   │       └── UsbAttachReceiver.kt   — Auto-launch on USB NCM device attach
│   │
│   ├── cpp/
│   │   ├── ncm_bridge.cpp             — High-performance TUN ↔ USB bridge
│   │   ├── ntb_parser.cpp             — NTB16/NTB32 parsing
│   │   ├── ntb_builder.cpp            — NTB16/NTB32 construction
│   │   ├── ncm_control.cpp            — NCM class-specific requests
│   │   ├── arp_handler.cpp            — ARP request/response
│   │   ├── dhcp_client.cpp            — Minimal DHCP client
│   │   └── jni_bridge.cpp             — JNI interface
│   │
│   └── res/
│       └── xml/
│           └── usb_device_filter.xml  — Match CDC NCM class/subclass
│
├── README.md
├── LICENSE
└── docs/
    ├── ARCHITECTURE.md
    ├── NCM_PROTOCOL.md
    └── TROUBLESHOOTING.md
```

### Data Flow (Detailed)

**Outbound (App → USB NCM Device):**

1. Android app sends TCP/UDP data
2. Kernel routes IP packet to TUN interface (VpnService has captured all routes)
3. `NcmBridge` reads raw IP packet from TUN file descriptor
4. `ntb_builder` wraps packet:
   - Prepends synthetic Ethernet header (dst=device MAC, src=phone MAC, type=0x0800)
   - Wraps Ethernet frame in NTB16: NTH16 header + datagram + NDP16 pointer table
   - Optionally aggregates multiple packets into one NTB for efficiency
5. `NcmBridge` writes NTB to USB bulk OUT endpoint via `UsbDeviceConnection.bulkTransfer()`

**Inbound (USB NCM Device → App):**

1. `NcmBridge` reads NTB from USB bulk IN endpoint
2. `ntb_parser` processes NTB:
   - Validates NTH16 signature (`NCMH`)
   - Follows NDP16 pointer table to locate datagrams
   - For each datagram: strips Ethernet header (14 bytes), extracts IP packet
3. `NcmBridge` writes IP packet to TUN file descriptor
4. Kernel delivers packet to the appropriate app socket

**Control Plane:**

1. On USB device attach: `NcmUsbManager` detects CDC NCM device by class/subclass
2. Sends NCM class requests on control endpoint:
   - `GET_NTB_PARAMETERS` — learn max NTB size, alignment
   - `SET_INTERFACE(1, alt=1)` — activate NCM data endpoints
3. Reads interrupt endpoint for `NetworkConnection` and `ConnectionSpeedChange` notifications
4. `dhcp_client` obtains IP address from device (or uses link-local)
5. Configures VpnService TUN with obtained IP, routes, DNS

---

## NCM Protocol Implementation

### NTB16 Format

```
NTB16 Structure:
┌─────────────────────────────────────────┐
│ NTH16 (12 bytes)                        │
│   dwSignature:    0x484D434E ("NCMH")   │
│   wHeaderLength:  12                    │
│   wSequence:      <counter>             │
│   wBlockLength:   <total NTB size>      │
│   wNdpIndex:      <offset to NDP>       │
├─────────────────────────────────────────┤
│ Ethernet Frame 1 (14 + IP packet)       │
├─────────────────────────────────────────┤
│ Ethernet Frame 2 (optional, aggregated) │
├─────────────────────────────────────────┤
│ ... more frames ...                     │
├─────────────────────────────────────────┤
│ NDP16                                   │
│   dwSignature:    0x00304D43 ("NCM0")   │
│   wLength:        <header + entries>    │
│   wNextNdpIndex:  0                     │
│   Datagram entries:                     │
│     wIndex, wLength (per datagram)      │
│     0, 0 (terminator)                   │
└─────────────────────────────────────────┘
```

### NTB32 Format

Same structure but with 32-bit fields for offset/length, supporting NTBs larger than 64KB. Signaled via `GET_NTB_PARAMETERS` response.

### NCM Class Requests

| bRequest | Name | Direction | Description |
|----------|------|-----------|-------------|
| 0x80 | GET_NTB_PARAMETERS | IN | Returns max NTB size, divisor, alignment |
| 0x81 | GET_NET_ADDRESS | IN | Returns device MAC address |
| 0x84 | GET_NTB_FORMAT | IN | Returns NTB16 or NTB32 |
| 0x85 | SET_NTB_FORMAT | OUT | Select NTB16 (0) or NTB32 (1) |
| 0x86 | GET_NTB_INPUT_SIZE | IN | Returns max NTB IN size |
| 0x87 | SET_NTB_INPUT_SIZE | OUT | Set max NTB IN size |

---

## Challenges & Solutions

### 1. Single VPN Service Limitation

**Problem:** Android allows only one active VpnService. If the user runs a VPN app (ExpressVPN, WireGuard, etc.), our app would disconnect it.

**Solutions:**
- Display a clear warning when a VPN is already active
- Allow "USB-only routing" mode: only route traffic for the USB subnet (e.g., 192.168.x.0/24) through the TUN, let other traffic bypass via the user's VPN. This is possible via `VpnService.Builder.addRoute()` with specific subnets instead of `0.0.0.0/0`.
- For limited use cases (specific app → USB device), use `addAllowedApplication()` to restrict which apps route through the USB link

### 2. Layer 3 Only (No Layer 2)

**Problem:** VpnService TUN operates at IP level. No raw Ethernet, mDNS, ARP, or DHCP passthrough.

**Solutions:**
- Implement ARP handling in userspace (synthesize Ethernet headers for outbound, strip for inbound)
- Implement DHCP client in userspace (parse DHCP offers from UDP port 67/68)
- For mDNS: implement an mDNS proxy that translates between multicast on the USB link and the TUN interface
- For most use cases (internet access via USB Ethernet), Layer 3 is sufficient

### 3. Performance

**Problem:** Double kernel-userspace crossing (TUN read/write) adds latency and CPU overhead.

**Solutions:**
- Implement the bridge loop in C++ (JNI) for minimum overhead
- Use NTB aggregation: batch multiple IP packets into a single NTB to reduce USB transaction count
- Use large NTB sizes (16KB-64KB) to amortize per-transfer overhead
- Use `UsbRequest` async API or libusb for non-blocking I/O
- Target: 50-100 Mbps sustained (sufficient for most use cases)

### 4. Kernel Driver Conflict

**Problem:** If the kernel has `cdc_ncm` compiled in, it may auto-bind before the app claims the interface.

**Solutions:**
- `claimInterface(force=true)` detaches the kernel driver
- Monitor for re-binding on USB suspend/resume
- If the kernel driver creates a `usb0` interface and Android 15+ is running, detect this and fall back to "just works" mode (no VPN needed, system handles it natively)

### 5. IP Address Configuration

**Problem:** The TUN interface needs an IP address, but we can't use kernel DHCP.

**Solutions:**
- Implement minimal DHCP client in userspace (send DISCOVER, parse OFFER, send REQUEST, parse ACK)
- Support static IP configuration in the app settings
- Support link-local addressing (169.254.x.x) as fallback
- Auto-detect common USB Ethernet device IP schemes (192.168.x.1 patterns)

---

## Feature Set

### MVP (v0.1)

- Detect and connect to USB CDC NCM devices
- NTB16 parsing and construction
- VpnService TUN bridge
- Static IP configuration
- Basic status UI (connected/disconnected, bytes transferred)
- Auto-launch on USB device attach

### v0.2

- DHCP client
- NTB32 support
- NTB frame aggregation (multiple packets per NTB)
- Performance optimizations (C++ bridge, async USB I/O)
- Subnet-only routing (coexist with user VPN)

### v0.3

- CDC ECM support (simpler, no NTB framing)
- CDC EEM support
- Per-app routing control
- mDNS proxy
- IPv6 support

### v1.0

- Stable, well-tested on major phone brands
- Google Play Store release
- Comprehensive device compatibility database
- Auto-detection of adapter capabilities (NCM vs ECM vs EEM)

---

## Compatibility

### Android Version Support

| Version | Kernel NCM | Framework | Our App |
|---------|-----------|-----------|---------|
| Android 9 (API 28) | Varies | Broken | Full support (userspace driver) |
| Android 10 (API 29) | Varies | Broken | Full support |
| Android 11 (API 30) | Varies | Broken | Full support |
| Android 12 (API 31) | GKI, NCM excluded | Broken | Full support |
| Android 13 (API 33) | GKI, NCM excluded | Broken | Full support |
| Android 14 (API 34) | GKI, NCM excluded | Broken | Full support |
| Android 15 (API 35) | GKI, NCM excluded | **Fixed** | Detect & defer to system (or full support) |

### Minimum Requirements

- Android 9+ (API 28) — for large `bulkTransfer()` buffers
- USB OTG host mode support
- VpnService permission from user

---

## Prior Art

| Project | Approach | Limitation |
|---------|----------|------------|
| [gnirehtet](https://github.com/Genymobile/gnirehtet) | VpnService → ADB → host PC | Requires PC running relay server |
| [vpn-reverse-tether](https://github.com/google/vpn-reverse-tether) | VpnService → ADB socket | Same; Google's own version |
| [usb-serial-for-android](https://github.com/mik3y/usb-serial-for-android) | Userspace CDC ACM | Serial only, no networking |
| [PCAPdroid](https://github.com/emanuele-f/PCAPdroid) | VpnService TUN capture | Capture/proxy, not bridging to USB |
| [NetGuard](https://github.com/M66B/NetGuard) | VpnService firewall | Filtering, not USB bridging |
| **usb-ncm-android (this)** | VpnService + USB Host + NCM | **Novel combination: generic USB Ethernet on unrooted Android** |

None of these projects combine VpnService TUN bridging with USB CDC NCM protocol handling. This would be the first.

---

## Development Plan

### Phase 1: Proof of Concept (2-3 weeks)
- Basic NCM endpoint detection and NTB parsing
- TUN interface creation via VpnService
- Bidirectional packet bridging (hardcoded IP)
- Test with Raspberry Pi as USB NCM gadget

### Phase 2: Robustness (2-3 weeks)
- DHCP client
- NTB aggregation
- Error handling, reconnection
- Performance optimization (C++ bridge)
- Test on 5+ phone models

### Phase 3: Polish (2 weeks)
- UI with connection status, throughput meter
- Settings (IP config, routing, auto-connect)
- CDC ECM/EEM support
- Documentation, README, screenshots

### Phase 4: Release (1 week)
- F-Droid / GitHub releases
- Google Play submission
- Community testing and bug reports

---

## References

- [USB CDC NCM Specification 1.0](https://www.usb.org/sites/default/files/NCM10_012011.zip)
- [Android VpnService API](https://developer.android.com/reference/android/net/VpnService)
- [Android USB Host API](https://developer.android.com/develop/connectivity/usb/host)
- [Why Android can't use CDC Ethernet](https://jordemort.dev/blog/why-android-cant-use-cdc-ethernet/)
- [AOSP Bug #304335605](https://issuetracker.google.com/issues/304335605)
- [gnirehtet (Genymobile)](https://github.com/Genymobile/gnirehtet)
- [usb-serial-for-android](https://github.com/mik3y/usb-serial-for-android)
- [Linux cdc_ncm.c source](https://github.com/torvalds/linux/blob/master/drivers/net/usb/cdc_ncm.c)
