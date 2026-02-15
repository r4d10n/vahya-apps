# Vahya Apps

Development repository for Vahya radio transceiver applications, focusing on cross-platform USB connectivity via CDC NCM.

## Overview

This repository contains the design and implementation for connecting the [Vahya radio transceiver](https://github.com/r4d10n/radio_gateware) (Lattice ECP5 + AT86RF215 dual-band SDR) to Android and iOS phones via USB CDC NCM.

## Key Design Decision

Instead of vendor-specific USB (class 0xFF) which only works on Android, the transceiver presents as a standard **CDC NCM Ethernet adapter**. This enables:

- **Android**: App claims NCM endpoints via USB Host API, parses NTB frames in userspace C++
- **iOS**: Native driverless support (iOS 16+), app uses `Network.framework` over `.wiredEthernet`
- **Linux/Mac/Windows**: Zero-config via native CDC NCM kernel drivers

IQ data and control commands are carried as UDP packets over the NCM Ethernet link.

## Repository Structure

```
docs/
├── plans/
│   └── 2026-02-15-usb-ncm-universal-design.md   — Full design document
└── reference/
    ├── android-usb-stack-learnings.md            — Android USB stack deep-dive
    └── userspace-ncm-driver-proposal.md          — Open-source generic NCM driver proposal

gateware-ref/    — Reference copies of current FPGA USB implementation
android-ref/     — Reference copies of current Android USB transport
host-ref/        — Reference copies of current host library
```

## Related Repositories

- [radio_gateware](https://github.com/r4d10n/radio_gateware) — FPGA gateware, host software, and VahyaDroid Android app

## License

TBD
