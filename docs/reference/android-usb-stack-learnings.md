# Android USB Stack: Learnings & Pitfalls for Hardware Developers

**Date:** 2026-02-15
**Context:** Research conducted for the Vahya SDR transceiver USB connectivity design

This document captures hard-won knowledge about connecting custom USB hardware to Android phones. It covers the USB device class support matrix, the infamous EthernetTracker bug, throughput benchmarks, and practical workarounds.

---

## 1. Android USB Architecture Overview

Android's USB stack has three layers that hardware developers must understand:

```
┌──────────────────────────────────────────────────────────┐
│ Layer 3: Android Framework (Java/Kotlin)                 │
│   UsbManager, EthernetTracker, TetheringManager          │
│   → Policy decisions, permission prompts, interface      │
│     naming validation, network routing                   │
├──────────────────────────────────────────────────────────┤
│ Layer 2: Linux Kernel (C)                                │
│   cdc_ncm.ko, cdc_ether.ko, cdc_eem.ko, usbnet.ko      │
│   → Device enumeration, class driver binding,            │
│     network interface creation                           │
├──────────────────────────────────────────────────────────┤
│ Layer 1: USB Host Controller (Hardware + Driver)         │
│   xHCI/eHCI, ULPI PHY                                   │
│   → Electrical signaling, endpoint management            │
└──────────────────────────────────────────────────────────┘
```

Failures can occur at any layer. The most insidious bugs are at Layer 3 (framework), because the USB device enumerates correctly and the kernel driver loads, but Android's Java framework refuses to use the interface.

---

## 2. USB Device Class Support Matrix (No Root)

| USB Class | Kernel Support | Framework Support | App API | Throughput | Notes |
|-----------|---------------|-------------------|---------|------------|-------|
| **Vendor-Specific (0xFF)** | No kernel driver needed | N/A | `android.hardware.usb` USB Host API | 30-40 MB/s | Best for custom apps. App claims endpoints directly. |
| **CDC ACM (Serial)** | `cdc_acm.ko` (GKI) | No `/dev/ttyACM*` access | `usb-serial-for-android` library (userspace) | 1-3 MB/s | Library auto-detects CDC ACM since v3.5.0 |
| **CDC NCM (Ethernet)** | **Excluded from GKI** | **Broken** (Android < 15) / Fixed (15+) | System networking or USB Host API | 30-40 MB/s | See Section 3 for the full story |
| **CDC ECM (Ethernet)** | **Excluded from GKI** | **Broken** (same regex issue) | Same | ~15 MB/s | NCM preferred over ECM |
| **CDC EEM (Ethernet)** | Module in GKI | **Broken** (`usb0` naming) | Same | ~20 MB/s | Only CDC Ethernet variant in GKI |
| **USB Audio (UAC1)** | Built-in | Native | `AudioManager` | ~1 MB/s | UAC2 NOT supported |
| **USB MIDI** | Built-in | Native | `android.media.midi` | ~3 KB/s | Class-compliant MIDI only |
| **USB HID** | Built-in | Native | `InputDevice` / USB Host API | ~800 KB/s | Keyboards, mice, gamepads |
| **USB Mass Storage** | Built-in | Native (read) | `StorageManager` / SAF | 30-40 MB/s | Read from external drives |
| **RNDIS** | Vendor-dependent | Tethering only (gadget) | N/A | Variable | Being removed from Linux kernel |

**Key takeaway:** For custom hardware needing high throughput, **vendor-specific bulk endpoints** via the USB Host API is the most reliable path on Android. The app has full control, no kernel driver conflicts, and achieves maximum USB 2.0 HS throughput.

---

## 3. The EthernetTracker `eth\d` Bug: Full Story

### What It Is

`EthernetTracker.java` in Android's Connectivity module uses a regex (`eth\d`) to filter which network interfaces it will manage. When a CDC Ethernet adapter creates a `usb0` interface, the regex rejects it. The interface exists in the kernel but Android's framework ignores it.

**Location:** `packages/modules/Connectivity/service/src/com/android/server/ConnectivityService/EthernetTracker.java`
**Config:** `config_ethernet_iface_regex` in `frameworks/base/core/res/res/values/config.xml`

### Why It Exists

The regex was introduced ~2014 (Android 5.0) when Ethernet support targeted Android TV and set-top boxes with physical NICs (`eth0`). Nobody anticipated USB CDC Ethernet on phones.

### Why It Persists

The same `usb0` interface name is used by both:
1. **USB CDC Ethernet adapters** (phone is USB host, `usb0` = external network)
2. **USB tethering** (phone is USB gadget, `usb0` = tethered network for PC)

If `EthernetTracker` accepts `usb0`, it could claim a tethering interface and try to configure the phone as both router and client simultaneously.

### The Linux Kernel Naming Quirk

The `usbnet` driver names interfaces based on MAC address type:
- **Globally-administered MAC** (bit 1 of octet 0 = 0): → `eth%d`
- **Locally-administered MAC** (bit 1 of octet 0 = 1): → `usb%d`

Brand-name adapters (ASIX, Realtek) with vendor-assigned MACs get `eth0` and work. Standards-compliant CDC devices with random MACs get `usb0` and fail.

**Workaround for hardware developers:** Assign a globally-administered MAC address to your CDC Ethernet device. The Linux kernel will name it `eth0`, matching the regex on all Android versions.

### Fix Timeline

| Date | Event | Android Version |
|------|-------|-----------------|
| ~2014 | `eth\d` regex introduced | Android 5.0 |
| Oct 2023 | Google engineer Patrick Rohr submits fix (change regex to `"*"`) | Android 14 (U) |
| Nov 2023 | Google engineer Lorenzo Colitti **reverts** fix: "devices in the field use usbX for tethering" | — |
| Dec 2023 | Google engineer Maciej Zenczykowski re-lands fix, gated to **Android V+ only** | Android 15 (V) |
| Sep 2024 | Android 15 ships with fix | Android 15 |
| 2025+ | Android 13/14 remain broken, no backport planned | — |

### AOSP References

- Bug: [issuetracker.google.com/304335605](https://issuetracker.google.com/issues/304335605)
- Original fix: commit `11eac8e12b30643881951b640da0db9cbe78d3c7`
- Revert: commit `ac6ff00b831683f095566f2982d08c88b175ba1c`
- Re-land (V+): commit `5512a2b`

---

## 4. Android USB Host API: Practical Guide

### Basics

```kotlin
// 1. Detect device connection
val usbManager = getSystemService(UsbManager::class.java)
val device: UsbDevice = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE)

// 2. Request permission
usbManager.requestPermission(device, pendingIntent)

// 3. Open connection
val connection = usbManager.openDevice(device)

// 4. Claim interface (force=true detaches kernel driver)
connection.claimInterface(device.getInterface(0), true)

// 5. Bulk transfer
val endpoint = device.getInterface(0).getEndpoint(0)
val buffer = ByteArray(16384)
val bytesRead = connection.bulkTransfer(endpoint, buffer, buffer.size, 1000)
```

### Key Limitations

1. **Buffer size cap (pre-API 28):** `bulkTransfer()` truncated to 16,384 bytes before Android 9 (Pie). From API 28+, any size works.

2. **No isochronous transfers:** Only bulk, control, and interrupt transfers are exposed. Audio/video isochronous endpoints are kernel-only.

3. **Permission prompt per connection:** User must grant permission each time the device is connected. Use `<usb-device>` filter in AndroidManifest for auto-launch, but permission is still required.

4. **`claimInterface(force=true)` behavior:** Detaches the kernel driver. Behavior varies across devices/kernel versions. On some phones, the kernel may re-bind on USB suspend/resume.

5. **No async bulk transfer in Java API directly:** `UsbRequest.queue()` provides async, but `bulkTransfer()` is synchronous. For high throughput, use `libusb` via JNI with `libusb_wrap_sys_device()`.

### Using libusb on Android

```cpp
// In JNI/C++:
int fd = env->CallIntMethod(usbConnection, getFileDescriptorMethod);

libusb_context* ctx;
libusb_init(&ctx);

libusb_device_handle* handle;
libusb_wrap_sys_device(ctx, fd, &handle);

// Now use libusb_submit_transfer() for async bulk transfers
```

This gives you the full power of libusb (async transfers, queuing, timeout control) while using Android's permission system for device access.

---

## 5. Throughput Benchmarks

### USB 2.0 High-Speed on Android

| Metric | Value |
|--------|-------|
| Signaling rate | 480 Mbps |
| Max bulk payload (theoretical) | ~53 MB/s |
| Practical sustained (libusb async) | 30-40 MB/s |
| Practical sustained (Java bulkTransfer) | 15-25 MB/s |
| Practical sustained (usb-serial-for-android) | 1-3 MB/s |

### Factors Affecting Throughput

- **Transfer size:** Larger transfers reduce per-transfer overhead. 16KB is a good default.
- **Queue depth:** Multiple in-flight async transfers overlap USB scheduling latency. 8-16 transfers is optimal.
- **CPU overhead:** Java `bulkTransfer()` copies data across JNI boundary. Native libusb avoids this.
- **USB controller:** Qualcomm's DWC3 controller handles high throughput well. MediaTek varies.

---

## 6. USB Suspend/Resume on Android

Android aggressively suspends USB devices to save power. This causes problems for streaming devices:

### Symptoms
- Bulk transfers start returning errors after 2-5 seconds of inactivity
- Device appears to disconnect and reconnect
- Streaming stalls after phone screen turns off

### Solutions

1. **Device side:** Implement proper USB SUSPEND/RESUME handling. When suspended, stop sending data. When resumed, re-sync.

2. **App side:** Hold a partial wake lock (`PowerManager.PARTIAL_WAKE_LOCK`) during streaming to prevent the CPU from sleeping.

3. **App side:** Use `UsbDeviceConnection.setInterface()` periodically to prevent suspend (hacky but effective on some devices).

4. **Device side:** If the device supports USB remote wakeup, advertise it in the configuration descriptor (`bmAttributes` bit 5 = 1).

---

## 7. Android USB Tethering Internals

Understanding tethering helps avoid namespace collisions with custom hardware:

| Android Version | Tethering Protocol | Interface Name |
|-----------------|-------------------|----------------|
| Android 10- | RNDIS | `rndis0` |
| Android 11-13 | RNDIS or NCM | `usb0` or `ncm0` |
| Android 14+ | NCM (RNDIS deprecated) | `ncm0` or `usb0` |

The tethering subsystem (`TetheringManager`) claims these interfaces. If your custom hardware creates a `usb0` interface that tethering doesn't expect, conflicts can occur.

---

## 8. Device Descriptor Best Practices for Android

1. **Use pid.codes VID/PID** (0x1209/xxxx) for open hardware, or apply for your own USB VID.

2. **Set bcdUSB to 0x0200** (USB 2.0). Android's USB host controller handles speed negotiation.

3. **Include string descriptors** (manufacturer, product, serial). Android displays these in the permission dialog.

4. **Set bMaxPower appropriately.** Android phones typically supply 100-500mA in host mode. Request what you need.

5. **For CDC devices, use a globally-administered MAC** to get `eth0` naming on Linux (see Section 3).

6. **For composite devices,** use Interface Association Descriptors (IADs) to group related interfaces.

7. **Implement proper USB reset handling.** Android may reset the USB bus during enumeration or after suspend.

---

## 9. GKI (Generic Kernel Image) Module Availability

Android GKI kernels include specific USB class drivers as modules:

| Driver | Module Name | GKI Status |
|--------|-------------|------------|
| `cdc_acm` | `cdc-acm.ko` | Included |
| `cdc_eem` | `cdc_eem.ko` | Included |
| `cdc_ether` | `cdc_ether.ko` | **Excluded** |
| `cdc_ncm` | `cdc_ncm.ko` | **Excluded** |
| `cdc_subset` | `cdc_subset.ko` | **Excluded** |
| `usbnet` | `usbnet.ko` | Base module, included |
| `asix` | `asix.ko` | Vendor, excluded |
| `r8152` | `r8152.ko` | Some vendors include |

The exclusion of `cdc_ether` and `cdc_ncm` from GKI means that even on Android 15+ (where the framework regex is fixed), the kernel may not have the driver to bind the device. However, OEMs can add these modules to their kernel images.

---

## 10. Alternatives to Kernel-Dependent USB Networking

When the kernel path is unreliable, these userspace approaches work:

### A. USB Host API + Custom Protocol
- App claims vendor-specific or CDC endpoints directly
- Implements protocol parsing in Java/C++
- No kernel driver needed, no framework dependency
- **Best for custom hardware with dedicated apps**

### B. VpnService + USB Host API (Generic NCM Driver)
- `VpnService` creates TUN interface (Layer 3)
- App bridges IP packets between TUN and USB NCM endpoints
- Works for any app (captures all IP traffic)
- **Limitation:** Only one VPN service at a time; conflicts with user VPN apps
- See: [Proposed Open Source Project](./userspace-ncm-driver-proposal.md)

### C. Network-over-serial
- Present device as CDC ACM serial port
- Use `usb-serial-for-android` library
- Run SLIP or PPP over serial to create IP connectivity
- **Very low throughput** (~1-3 MB/s) but universally compatible

---

## References

- [Android USB Host Overview](https://developer.android.com/develop/connectivity/usb/host)
- [Why Android can't use CDC Ethernet](https://jordemort.dev/blog/why-android-cant-use-cdc-ethernet/)
- [Android USB Digital Audio](https://source.android.com/docs/core/audio/usb)
- [Android Open Accessory Protocol](https://source.android.com/docs/core/interaction/accessories/protocol)
- [usb-serial-for-android](https://github.com/mik3y/usb-serial-for-android)
- [AOSP EthernetTracker Bug #304335605](https://issuetracker.google.com/issues/304335605)
- [GKI Kernel Modules](https://source.android.com/docs/core/architecture/kernel/convert-or-add)
- [Porting a Kernel USB Driver to Android Userspace](https://brycethomas.github.io/2015/01/18/porting-a-kernel-space-usb-driver-to-android-user-space.html)
