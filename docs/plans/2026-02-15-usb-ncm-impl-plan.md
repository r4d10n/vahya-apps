# USB CDC NCM Universal Transport Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace vendor-specific USB (class 0xFF) with CDC NCM to enable cross-platform Android + iOS connectivity for the Vahya dual-band SDR transceiver.

**Architecture:** The FPGA presents as a standard CDC NCM Ethernet adapter. IQ data streams as UDP packets inside NTB16 frames. Android parses NTBs in userspace C++ via the USB Host API. iOS uses native NCM support with `Network.framework`. Control commands (SPI, LED, reset) move from EP0 vendor requests to UDP port 5550.

**Tech Stack:** Amaranth HDL 0.5.4 / LUNA USB framework (gateware), C++17 / libusb (Android JNI), Swift / Network.framework (iOS)

**Design doc:** `docs/plans/2026-02-15-usb-ncm-universal-design.md`

---

## Phase 1: FPGA Gateware — NCM Device

All gateware files are in `/home/rfsoc/exp/Vahya/radio_gateware/gateware/`.
Tests are in `/home/rfsoc/exp/Vahya/radio_gateware/tests/sim/`.
Build: `source /home/rfsoc/exp/Vahya/vahya/env && cd /home/rfsoc/exp/Vahya/radio_gateware`.

---

### Task 1: NCM USB Descriptors

**Files:**
- Create: `gateware/usb/ncm_descriptors.py`
- Test: `tests/sim/test_ncm_descriptors.py`

**Step 1: Write the failing test**

```python
# tests/sim/test_ncm_descriptors.py
"""Test NCM descriptor structure and content."""
import unittest
from gateware.usb.ncm_descriptors import create_ncm_descriptors, NCM_MAC_STRING_INDEX

class TestNcmDescriptors(unittest.TestCase):
    def setUp(self):
        self.descriptors = create_ncm_descriptors()

    def test_device_descriptor_class(self):
        """Device class must be Communications (0x02) / NCM (0x0D)."""
        dd = bytes(self.descriptors.get_descriptor_bytes(0x01))  # DEVICE
        self.assertEqual(dd[4], 0x02)   # bDeviceClass = Communications
        self.assertEqual(dd[5], 0x0D)   # bDeviceSubClass = NCM

    def test_vid_pid(self):
        dd = bytes(self.descriptors.get_descriptor_bytes(0x01))
        vid = dd[8] | (dd[9] << 8)
        pid = dd[10] | (dd[11] << 8)
        self.assertEqual(vid, 0x1209)
        self.assertEqual(pid, 0x0001)

    def test_two_interfaces(self):
        """Must have 2 interfaces: NCM Control + NCM Data."""
        cd = bytes(self.descriptors.get_descriptor_bytes(0x02))
        bNumInterfaces = cd[4]
        self.assertEqual(bNumInterfaces, 2)

    def test_control_interface_class(self):
        """Interface 0 must be Communications/NCM."""
        # Parse config descriptor to find interface descriptors
        cd = bytes(self.descriptors.get_descriptor_bytes(0x02))
        # Find first interface descriptor (bDescriptorType=4)
        idx = 0
        while idx < len(cd):
            length = cd[idx]
            dtype = cd[idx + 1]
            if dtype == 0x04 and cd[idx + 2] == 0:  # interface 0
                self.assertEqual(cd[idx + 5], 0x02)  # bInterfaceClass
                self.assertEqual(cd[idx + 6], 0x0D)  # bInterfaceSubClass
                return
            idx += length
        self.fail("Interface 0 not found")

    def test_data_interface_class(self):
        """Interface 1 must be CDC Data (0x0A)."""
        cd = bytes(self.descriptors.get_descriptor_bytes(0x02))
        idx = 0
        iface1_found = False
        while idx < len(cd):
            length = cd[idx]
            dtype = cd[idx + 1]
            if dtype == 0x04 and cd[idx + 2] == 1:  # interface 1
                self.assertEqual(cd[idx + 5], 0x0A)  # bInterfaceClass = CDC Data
                iface1_found = True
                break
            idx += length
        self.assertTrue(iface1_found, "Interface 1 not found")

    def test_mac_string_descriptor_globally_administered(self):
        """MAC must be globally administered (bit 1 of first octet = 0)."""
        mac_str = self.descriptors.get_descriptor_bytes(
            0x03, index=NCM_MAC_STRING_INDEX)
        # String descriptor: [len, type, char0_lo, char0_hi, char1_lo, ...]
        mac_bytes = bytes(mac_str)
        # Extract first two chars (hex digits of first octet)
        first_octet_hex = chr(mac_bytes[2]) + chr(mac_bytes[4])
        first_octet = int(first_octet_hex, 16)
        self.assertEqual(first_octet & 0x02, 0,
                         f"MAC first octet 0x{first_octet:02X} has locally administered bit set")

if __name__ == "__main__":
    unittest.main()
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest tests/sim/test_ncm_descriptors.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'gateware.usb.ncm_descriptors'`

**Step 3: Write minimal implementation**

```python
# gateware/usb/ncm_descriptors.py
"""USB CDC NCM descriptors for Vahya radio.

Presents the device as a standard CDC NCM Ethernet adapter.
Two interfaces:
  - Interface 0: CDC NCM Control (Communications class, Interrupt IN EP)
  - Interface 1: CDC NCM Data (CDC Data class, Bulk IN + Bulk OUT EPs)
    - Alt Setting 0: no endpoints (idle, per NCM spec)
    - Alt Setting 1: active (EP2 IN bulk, EP3 OUT bulk)

MAC address uses globally-administered format so Linux names the
interface `eth0` (not `usb0`), working around Android < 15's
EthernetTracker regex bug.
"""

from usb_protocol.emitters import DeviceDescriptorCollection
from usb_protocol.types.descriptors.standard import StandardDescriptorNumbers

VENDOR_ID  = 0x1209
PRODUCT_ID = 0x0001

# Endpoint numbers
EP_INTERRUPT = 1   # IN  0x81 — NCM notifications
EP_BULK_IN   = 2   # IN  0x82 — NTB frames to host
EP_BULK_OUT  = 3   # OUT 0x03 — NTB frames from host

MAX_BULK_PACKET_SIZE = 512  # USB 2.0 HS
MAX_INTR_PACKET_SIZE = 16

# MAC address: globally-administered (bit 1 of first octet = 0)
# Using pid.codes OUI-like prefix 12:09:00 with device suffix
DEVICE_MAC = "120900000001"
NCM_MAC_STRING_INDEX = 4  # string descriptor index for MAC


def create_ncm_descriptors():
    """Build USB CDC NCM descriptors."""
    descriptors = DeviceDescriptorCollection()

    with descriptors.DeviceDescriptor() as d:
        d.bcdUSB           = 0x0200
        d.bDeviceClass     = 0x02    # Communications
        d.bDeviceSubClass  = 0x0D    # NCM
        d.bDeviceProtocol  = 0x00
        d.bMaxPacketSize0  = 64
        d.idVendor         = VENDOR_ID
        d.idProduct        = PRODUCT_ID
        d.iManufacturer    = "Vahya"
        d.iProduct         = "Vahya Radio"
        d.iSerialNumber    = "0001"
        d.bNumConfigurations = 1

    with descriptors.ConfigurationDescriptor() as c:
        c.bmAttributes = 0x80   # bus-powered
        c.bMaxPower    = 250    # 500 mA

        # ── Interface 0: CDC NCM Control ──────────────────────
        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber   = 0
            i.bAlternateSetting  = 0
            i.bInterfaceClass    = 0x02  # Communications
            i.bInterfaceSubClass = 0x0D  # NCM
            i.bInterfaceProtocol = 0x00

            # CDC Header Functional Descriptor
            i.add_subordinate_descriptor(bytes([
                5,     # bLength
                0x24,  # bDescriptorType = CS_INTERFACE
                0x00,  # bDescriptorSubtype = Header
                0x20, 0x01,  # bcdCDC = 1.20
            ]))

            # CDC Union Functional Descriptor
            i.add_subordinate_descriptor(bytes([
                5,     # bLength
                0x24,  # bDescriptorType = CS_INTERFACE
                0x06,  # bDescriptorSubtype = Union
                0,     # bControlInterface = 0
                1,     # bSubordinateInterface = 1
            ]))

            # CDC Ethernet Networking Functional Descriptor
            i.add_subordinate_descriptor(bytes([
                13,    # bLength
                0x24,  # bDescriptorType = CS_INTERFACE
                0x0F,  # bDescriptorSubtype = Ethernet Networking
                NCM_MAC_STRING_INDEX,  # iMACAddress (string descriptor index)
                0x00, 0x00, 0x00, 0x00,  # bmEthernetStatistics
                0xEA, 0x05,  # wMaxSegmentSize = 1514
                0x00, 0x00,  # wNumberMCFilters = 0
                0,     # bNumberPowerFilters = 0
            ]))

            # CDC NCM Functional Descriptor
            i.add_subordinate_descriptor(bytes([
                6,     # bLength
                0x24,  # bDescriptorType = CS_INTERFACE
                0x1A,  # bDescriptorSubtype = NCM
                0x00, 0x01,  # bcdNcmVersion = 1.00
                0x00,  # bmNetworkCapabilities = 0
            ]))

            # Interrupt IN endpoint (notifications)
            with i.EndpointDescriptor() as e:
                e.bEndpointAddress = 0x80 | EP_INTERRUPT
                e.bmAttributes    = 0x03  # Interrupt
                e.wMaxPacketSize  = MAX_INTR_PACKET_SIZE
                e.bInterval       = 32    # 32ms polling

        # ── Interface 1: CDC NCM Data ─────────────────────────
        # Alt Setting 0: no endpoints (idle per NCM spec)
        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber   = 1
            i.bAlternateSetting  = 0
            i.bInterfaceClass    = 0x0A  # CDC Data
            i.bInterfaceSubClass = 0x00
            i.bInterfaceProtocol = 0x01  # NCM
            # No endpoints in alt setting 0

        # Alt Setting 1: active
        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber   = 1
            i.bAlternateSetting  = 1
            i.bInterfaceClass    = 0x0A  # CDC Data
            i.bInterfaceSubClass = 0x00
            i.bInterfaceProtocol = 0x01  # NCM

            with i.EndpointDescriptor() as e:
                e.bEndpointAddress = 0x80 | EP_BULK_IN
                e.wMaxPacketSize   = MAX_BULK_PACKET_SIZE

            with i.EndpointDescriptor() as e:
                e.bEndpointAddress = EP_BULK_OUT
                e.wMaxPacketSize   = MAX_BULK_PACKET_SIZE

    # Add MAC address string descriptor
    # NCM spec requires MAC as a Unicode string of hex digits (12 chars)
    descriptors.add_descriptor(
        _make_string_descriptor(DEVICE_MAC),
        descriptor_type=0x03,
        index=NCM_MAC_STRING_INDEX,
    )

    return descriptors


def _make_string_descriptor(text):
    """Build a USB string descriptor from ASCII text."""
    encoded = text.encode('utf-16-le')
    return bytes([2 + len(encoded), 0x03]) + encoded
```

**Note:** The `add_descriptor` and `get_descriptor_bytes` methods depend on the exact LUNA `usb_protocol` API. This implementation may need adjustment based on the actual API. The test will tell us.

**Step 4: Run test to verify it passes**

Run: `python -m pytest tests/sim/test_ncm_descriptors.py -v`
Expected: PASS (or adjust implementation to match `usb_protocol` API)

**Step 5: Commit**

```bash
git add gateware/usb/ncm_descriptors.py tests/sim/test_ncm_descriptors.py
git commit -m "feat: add CDC NCM USB descriptor module"
```

---

### Task 2: NTB16 Framer Module

**Files:**
- Create: `gateware/ncm/ntb_framer.py`
- Test: `tests/sim/test_ntb_framer.py`

**Context:** The NTB Framer reads IQ samples from the existing RxFIFO bridges and wraps them in NTB16 frames (NTH16 + Ethernet + IPv4 + UDP + IQ payload + NDP16). It outputs a byte stream to the USB bulk IN endpoint.

**Step 1: Write the failing test**

```python
# tests/sim/test_ntb_framer.py
"""Test NTB16 framer: IQ samples → NTB byte stream."""
import unittest
import struct
from amaranth import *
from amaranth.sim import Simulator

from gateware.ncm.ntb_framer import NtbFramer


class TestNtbFramer(unittest.TestCase):

    def test_nth16_signature(self):
        """First 4 bytes of output must be NTH16 signature 0x484D434E."""
        dut = NtbFramer()
        output = []

        def testbench():
            # Feed one IQ sample pair (I=0x1234, Q=0x5678)
            yield dut.iq_09_valid.eq(1)
            yield dut.iq_09_i.eq(0x1234)
            yield dut.iq_09_q.eq(0x5678)
            yield dut.iq_24_valid.eq(0)
            yield dut.out_ready.eq(1)
            yield

            # Collect output bytes
            for _ in range(200):
                yield
                valid = yield dut.out_valid
                if valid:
                    byte = yield dut.out_data
                    output.append(byte)

        sim = Simulator(dut)
        sim.add_clock(1 / 60e6, domain="usb")
        sim.add_sync_process(testbench, domain="usb")
        with sim.write_vcd("ntb_framer.vcd"):
            sim.run()

        self.assertGreaterEqual(len(output), 4)
        sig = struct.unpack_from('<I', bytes(output[:4]))[0]
        self.assertEqual(sig, 0x484D434E, f"NTH16 signature = 0x{sig:08X}")

    def test_ndp16_present(self):
        """NTB must contain NDP16 with signature 0x00304D43."""
        dut = NtbFramer()
        output = []

        def testbench():
            yield dut.iq_09_valid.eq(1)
            yield dut.iq_09_i.eq(0x0100)
            yield dut.iq_09_q.eq(0x0200)
            yield dut.iq_24_valid.eq(0)
            yield dut.out_ready.eq(1)
            yield

            for _ in range(500):
                yield
                valid = yield dut.out_valid
                if valid:
                    byte = yield dut.out_data
                    output.append(byte)

        sim = Simulator(dut)
        sim.add_clock(1 / 60e6, domain="usb")
        sim.add_sync_process(testbench, domain="usb")
        with sim.write_vcd("ntb_ndp.vcd"):
            sim.run()

        data = bytes(output)
        # Find NDP16 signature anywhere in output
        ndp_sig = struct.pack('<I', 0x00304D43)
        self.assertIn(ndp_sig, data, "NDP16 signature not found in output")

    def test_udp_port_5551_for_900mhz(self):
        """900 MHz IQ data must use UDP dst port 5551."""
        dut = NtbFramer()
        output = []

        def testbench():
            yield dut.iq_09_valid.eq(1)
            yield dut.iq_09_i.eq(0x1111)
            yield dut.iq_09_q.eq(0x2222)
            yield dut.iq_24_valid.eq(0)
            yield dut.out_ready.eq(1)
            yield

            for _ in range(500):
                yield
                valid = yield dut.out_valid
                if valid:
                    byte = yield dut.out_data
                    output.append(byte)

        sim = Simulator(dut)
        sim.add_clock(1 / 60e6, domain="usb")
        sim.add_sync_process(testbench, domain="usb")
        with sim.write_vcd("ntb_port.vcd"):
            sim.run()

        data = bytes(output)
        # UDP header starts at offset 12 (NTH16) + 14 (Eth) + 20 (IP) = 46
        # dst port is at UDP offset +2, big-endian
        if len(data) >= 50:
            udp_dst_port = struct.unpack_from('>H', data, 48)[0]
            self.assertEqual(udp_dst_port, 5551)


if __name__ == "__main__":
    unittest.main()
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest tests/sim/test_ntb_framer.py -v`
Expected: FAIL — `ModuleNotFoundError`

**Step 3: Write the NTB Framer implementation**

Create `gateware/ncm/__init__.py` (empty) and `gateware/ncm/ntb_framer.py`.

The NTB Framer is a state machine that:
1. Waits for IQ samples from the RxFIFO bridges
2. Accumulates samples into a buffer (configurable NTB payload size)
3. When enough samples are collected (or a timeout fires), constructs:
   - NTH16 header (12 bytes)
   - Ethernet header (14 bytes) + IPv4 header (20 bytes) + UDP header (8 bytes)
   - IQ payload
   - NDP16 pointer table
4. Outputs the complete NTB as a byte stream

This is a complex module (~200-300 lines Amaranth). Implementation details are in the design doc Section 5.

**Key constants to embed as ROM:**
- Ethernet dst MAC: broadcast `FF:FF:FF:FF:FF:FF` (simplified; phone MAC learned via ARP)
- Ethernet src MAC: `12:09:00:00:00:01`
- EtherType: `0x0800` (IPv4)
- IP src: `192.168.73.1`, IP dst: `192.168.73.2`
- IP protocol: `17` (UDP)
- UDP src/dst ports: `5551` (900 MHz), `5552` (2.4 GHz)

**Step 4: Run test to verify it passes**

Run: `python -m pytest tests/sim/test_ntb_framer.py -v`
Expected: PASS

**Step 5: Commit**

```bash
git add gateware/ncm/__init__.py gateware/ncm/ntb_framer.py tests/sim/test_ntb_framer.py
git commit -m "feat: add NTB16 framer module for IQ → NCM conversion"
```

---

### Task 3: NTB16 Deframer Module

**Files:**
- Create: `gateware/ncm/ntb_deframer.py`
- Test: `tests/sim/test_ntb_deframer.py`

**Context:** The NTB Deframer receives NTB byte streams from the USB bulk OUT endpoint, parses NTH16 + NDP16, extracts datagrams, strips Ethernet+IP+UDP headers, and routes IQ data by UDP port to the TX FIFOs or control handler.

**Step 1: Write the failing test**

```python
# tests/sim/test_ntb_deframer.py
"""Test NTB16 deframer: NTB byte stream → IQ samples + control."""
import unittest
import struct
from amaranth import *
from amaranth.sim import Simulator

from gateware.ncm.ntb_deframer import NtbDeframer


def build_ntb16(datagrams):
    """Helper: build a valid NTB16 from a list of (udp_port, payload) tuples."""
    nth16 = bytearray(12)
    struct.pack_into('<I', nth16, 0, 0x484D434E)   # dwSignature
    struct.pack_into('<H', nth16, 4, 12)            # wHeaderLength
    struct.pack_into('<H', nth16, 6, 0)             # wSequence

    offset = 12
    dg_entries = []
    dg_data = bytearray()

    for udp_port, payload in datagrams:
        # Ethernet header (14 bytes)
        eth = bytes([0xFF]*6 + [0x12,0x09,0x00,0x00,0x00,0x01] + [0x08, 0x00])
        # IPv4 header (20 bytes) - minimal valid
        ip_len = 20 + 8 + len(payload)
        ip_hdr = bytearray(20)
        ip_hdr[0] = 0x45  # version + IHL
        struct.pack_into('>H', ip_hdr, 2, ip_len)
        ip_hdr[8] = 64    # TTL
        ip_hdr[9] = 17    # protocol = UDP
        ip_hdr[12:16] = bytes([192, 168, 73, 2])  # src
        ip_hdr[16:20] = bytes([192, 168, 73, 1])  # dst
        # UDP header (8 bytes)
        udp_hdr = bytearray(8)
        struct.pack_into('>H', udp_hdr, 0, udp_port)  # src port
        struct.pack_into('>H', udp_hdr, 2, udp_port)  # dst port
        struct.pack_into('>H', udp_hdr, 4, 8 + len(payload))

        frame = eth + bytes(ip_hdr) + bytes(udp_hdr) + payload
        dg_entries.append((offset, len(frame)))
        dg_data += frame
        offset += len(frame)

    # NDP16
    ndp_offset = offset
    ndp = bytearray(8 + 4 * (len(dg_entries) + 1))
    struct.pack_into('<I', ndp, 0, 0x00304D43)  # dwSignature
    struct.pack_into('<H', ndp, 4, len(ndp))     # wLength
    struct.pack_into('<H', ndp, 6, 0)            # wNextNdpIndex
    for j, (dg_off, dg_len) in enumerate(dg_entries):
        struct.pack_into('<H', ndp, 8 + j*4, dg_off)
        struct.pack_into('<H', ndp, 8 + j*4 + 2, dg_len)
    # Terminator already zero from bytearray init

    # Finalize NTH16
    total_len = 12 + len(dg_data) + len(ndp)
    struct.pack_into('<H', nth16, 8, total_len)     # wBlockLength
    struct.pack_into('<H', nth16, 10, ndp_offset)   # wNdpIndex

    return bytes(nth16) + bytes(dg_data) + bytes(ndp)


class TestNtbDeframer(unittest.TestCase):

    def test_extracts_iq_from_port_5553(self):
        """IQ data on UDP port 5553 should route to TX 900 MHz output."""
        iq_payload = struct.pack('<hh', 0x1234, 0x5678)  # one IQ pair
        ntb = build_ntb16([(5553, iq_payload)])

        dut = NtbDeframer()
        tx09_output = []

        def testbench():
            # Feed NTB bytes one at a time
            for byte in ntb:
                yield dut.in_data.eq(byte)
                yield dut.in_valid.eq(1)
                yield
            yield dut.in_valid.eq(0)

            # Collect TX 900 output
            for _ in range(50):
                yield
                valid = yield dut.tx09_valid
                if valid:
                    i_val = yield dut.tx09_i
                    q_val = yield dut.tx09_q
                    tx09_output.append((i_val, q_val))

        sim = Simulator(dut)
        sim.add_clock(1 / 60e6, domain="usb")
        sim.add_sync_process(testbench, domain="usb")
        with sim.write_vcd("ntb_deframer.vcd"):
            sim.run()

        self.assertEqual(len(tx09_output), 1)
        self.assertEqual(tx09_output[0], (0x1234, 0x5678))

    def test_routes_control_to_port_5550(self):
        """Control data on UDP port 5550 should route to control output."""
        ctrl_payload = bytes([0x01, 0x00, 0x0A, 0x42])  # SPI_WRITE addr=0x000A data=0x42
        ntb = build_ntb16([(5550, ctrl_payload)])

        dut = NtbDeframer()
        ctrl_output = []

        def testbench():
            for byte in ntb:
                yield dut.in_data.eq(byte)
                yield dut.in_valid.eq(1)
                yield
            yield dut.in_valid.eq(0)

            for _ in range(50):
                yield
                valid = yield dut.ctrl_valid
                if valid:
                    data = yield dut.ctrl_data
                    ctrl_output.append(data)

        sim = Simulator(dut)
        sim.add_clock(1 / 60e6, domain="usb")
        sim.add_sync_process(testbench, domain="usb")
        with sim.write_vcd("ntb_deframer_ctrl.vcd"):
            sim.run()

        self.assertGreater(len(ctrl_output), 0)

    def test_rejects_invalid_signature(self):
        """NTB with wrong signature should be silently dropped."""
        ntb = build_ntb16([(5553, b'\x00\x00\x00\x00')])
        # Corrupt the signature
        ntb = b'\x00\x00\x00\x00' + ntb[4:]

        dut = NtbDeframer()
        tx09_output = []

        def testbench():
            for byte in ntb:
                yield dut.in_data.eq(byte)
                yield dut.in_valid.eq(1)
                yield
            yield dut.in_valid.eq(0)

            for _ in range(50):
                yield
                valid = yield dut.tx09_valid
                if valid:
                    tx09_output.append(1)

        sim = Simulator(dut)
        sim.add_clock(1 / 60e6, domain="usb")
        sim.add_sync_process(testbench, domain="usb")
        with sim.write_vcd("ntb_deframer_bad.vcd"):
            sim.run()

        self.assertEqual(len(tx09_output), 0, "Invalid NTB should produce no output")


if __name__ == "__main__":
    unittest.main()
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest tests/sim/test_ntb_deframer.py -v`
Expected: FAIL — `ModuleNotFoundError`

**Step 3: Write implementation**

Create `gateware/ncm/ntb_deframer.py`. The deframer is a state machine:
1. `IDLE` → read 4 bytes, validate NTH16 signature
2. `NTH16` → parse header length, block length, NDP index
3. `SKIP_TO_NDP` → skip to NDP offset
4. `NDP16` → parse datagram pointers
5. `EXTRACT` → for each datagram: skip 42 bytes (Eth+IP+UDP), read UDP dst port at offset 36-37, route payload

**Outputs:**
- `tx09_i`, `tx09_q`, `tx09_valid` — TX 900 MHz IQ samples
- `tx24_i`, `tx24_q`, `tx24_valid` — TX 2.4 GHz IQ samples
- `ctrl_data`, `ctrl_valid` — Control command bytes

**Step 4: Run tests**

Run: `python -m pytest tests/sim/test_ntb_deframer.py -v`
Expected: PASS

**Step 5: Commit**

```bash
git add gateware/ncm/ntb_deframer.py tests/sim/test_ntb_deframer.py
git commit -m "feat: add NTB16 deframer for NCM → IQ + control routing"
```

---

### Task 4: ARP Responder

**Files:**
- Create: `gateware/ncm/arp_responder.py`
- Test: `tests/sim/test_arp_responder.py`

**Context:** Responds to ARP "who-has 192.168.73.1?" requests with the device's MAC address. Required for iOS and Linux hosts to resolve the device's IP address over the NCM link.

**Step 1: Write the failing test**

```python
# tests/sim/test_arp_responder.py
"""Test ARP responder: replies to ARP requests for 192.168.73.1."""
import unittest
import struct
from amaranth import *
from amaranth.sim import Simulator

from gateware.ncm.arp_responder import ArpResponder

DEVICE_MAC = bytes([0x12, 0x09, 0x00, 0x00, 0x00, 0x01])
DEVICE_IP  = bytes([192, 168, 73, 1])
HOST_MAC   = bytes([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF])
HOST_IP    = bytes([192, 168, 73, 2])


def build_arp_request(sender_mac, sender_ip, target_ip):
    """Build an ARP request Ethernet frame."""
    eth = bytes([0xFF]*6) + sender_mac + b'\x08\x06'  # EtherType = ARP
    arp = struct.pack('>HH', 0x0001, 0x0800)  # hw=Ethernet, proto=IPv4
    arp += bytes([6, 4])  # hw_len, proto_len
    arp += struct.pack('>H', 1)  # opcode = request
    arp += sender_mac + sender_ip
    arp += bytes(6) + target_ip  # target MAC unknown
    return eth + arp


class TestArpResponder(unittest.TestCase):

    def test_replies_to_arp_for_device_ip(self):
        """ARP request for 192.168.73.1 should produce ARP reply."""
        req = build_arp_request(HOST_MAC, HOST_IP, DEVICE_IP)
        dut = ArpResponder()
        reply_bytes = []

        def testbench():
            for byte in req:
                yield dut.in_data.eq(byte)
                yield dut.in_valid.eq(1)
                yield
            yield dut.in_valid.eq(0)

            for _ in range(100):
                yield
                valid = yield dut.out_valid
                if valid:
                    b = yield dut.out_data
                    reply_bytes.append(b)

        sim = Simulator(dut)
        sim.add_clock(1 / 60e6, domain="usb")
        sim.add_sync_process(testbench, domain="usb")
        with sim.write_vcd("arp_resp.vcd"):
            sim.run()

        self.assertGreater(len(reply_bytes), 0, "No ARP reply generated")

        reply = bytes(reply_bytes)
        # Check ARP opcode = 2 (reply) at offset 14+6 = 20
        if len(reply) >= 22:
            opcode = struct.unpack_from('>H', reply, 20)[0]
            self.assertEqual(opcode, 2, "ARP opcode should be 2 (reply)")

    def test_ignores_arp_for_other_ip(self):
        """ARP request for 192.168.73.99 should be ignored."""
        other_ip = bytes([192, 168, 73, 99])
        req = build_arp_request(HOST_MAC, HOST_IP, other_ip)
        dut = ArpResponder()
        reply_bytes = []

        def testbench():
            for byte in req:
                yield dut.in_data.eq(byte)
                yield dut.in_valid.eq(1)
                yield
            yield dut.in_valid.eq(0)

            for _ in range(100):
                yield
                valid = yield dut.out_valid
                if valid:
                    reply_bytes.append(1)

        sim = Simulator(dut)
        sim.add_clock(1 / 60e6, domain="usb")
        sim.add_sync_process(testbench, domain="usb")
        with sim.write_vcd("arp_ignore.vcd"):
            sim.run()

        self.assertEqual(len(reply_bytes), 0, "Should not reply to ARP for other IP")


if __name__ == "__main__":
    unittest.main()
```

**Step 2-5:** Same pattern — run failing, implement, run passing, commit.

```bash
git add gateware/ncm/arp_responder.py tests/sim/test_arp_responder.py
git commit -m "feat: add ARP responder for NCM network stack"
```

---

### Task 5: Minimal DHCP Server

**Files:**
- Create: `gateware/ncm/dhcp_server.py`
- Test: `tests/sim/test_dhcp_server.py`

**Context:** Minimal DHCP server that assigns 192.168.73.2 to the phone. Handles only DISCOVER→OFFER and REQUEST→ACK. Runs entirely in the `usb` clock domain.

The DHCP server processes Ethernet frames extracted from NTBs (by the NTB deframer routing UDP port 67 traffic here). It responds with DHCP OFFER/ACK packets that get wrapped back into NTBs by the framer.

**Implementation note:** The DHCP server only needs to handle 4 message types and can use hardcoded responses. Total size: ~200-300 lines Amaranth.

```bash
git add gateware/ncm/dhcp_server.py tests/sim/test_dhcp_server.py
git commit -m "feat: add minimal DHCP server for NCM link"
```

---

### Task 6: Control-over-UDP Handler

**Files:**
- Create: `gateware/ncm/control_handler.py`
- Test: `tests/sim/test_control_handler.py`

**Context:** Replaces the EP0 vendor request handler. Receives control commands from UDP port 5550 (extracted by the NTB deframer), dispatches SPI read/write, LED, reset commands. Queues responses for the NTB framer to include in outbound NTBs.

**Interface to existing SPI master:** Same signals as `RadioVendorRequestHandler` (spi_addr, spi_wdata, spi_rdata, spi_rw, spi_start, spi_busy, led_r/g/b, at86_rst). The top.py CDC synchronization code for SPI is reused unchanged.

**Protocol:**
- Input: `ctrl_data` (8-bit), `ctrl_valid` from NTB deframer
- Parses: cmd (1 byte) + addr (2 bytes LE) + data (0-N bytes)
- Output: `resp_data` (8-bit), `resp_valid` to NTB framer (for responses)

```bash
git add gateware/ncm/control_handler.py tests/sim/test_control_handler.py
git commit -m "feat: add control-over-UDP handler replacing EP0 vendor requests"
```

---

### Task 7: NCM Control Request Handler

**Files:**
- Create: `gateware/usb/ncm_control.py`
- Test: `tests/sim/test_ncm_control.py`

**Context:** Handles NCM class-specific control requests on EP0:
- `GET_NTB_PARAMETERS` (0x80): returns max NTB size (16384), alignment, etc.
- `SET_INTERFACE` to alt setting 1: activates data endpoints.
- Interrupt EP: sends `NetworkConnection(1)` and `ConnectionSpeedChange` notifications.

This is a LUNA `USBRequestHandler` subclass, similar to the existing `RadioVendorRequestHandler` but for CDC/NCM class requests instead of vendor requests.

```bash
git add gateware/usb/ncm_control.py tests/sim/test_ncm_control.py
git commit -m "feat: add NCM class-specific control request handler"
```

---

### Task 8: Top-Level Integration

**Files:**
- Modify: `gateware/top.py` (major rewrite of USB section)
- Modify: `gateware/usb/device.py` → replace with import from `ncm_descriptors.py`
- Test: `tests/sim/test_ncm_top.py` (integration test)

**Context:** Replace the 4-endpoint vendor-specific USB device with the NCM composite device. Wire the NTB framer/deframer between the RxFIFO/TxFIFO bridges and the new bulk endpoints. Wire the control handler, ARP responder, and DHCP server.

**Key changes to `top.py`:**

1. Replace `create_descriptors()` with `create_ncm_descriptors()`
2. Replace 4 bulk endpoints with: 1 interrupt IN + 1 bulk IN + 1 bulk OUT
3. Replace `RadioVendorRequestHandler` with `NcmControlRequestHandler`
4. Add NTB Framer between RxFIFO bridges and bulk IN endpoint
5. Add NTB Deframer between bulk OUT endpoint and TxFIFO bridges + control handler
6. Add ARP responder and DHCP server
7. Wire control handler to existing SPI CDC synchronization logic
8. Keep all LVDS, SPI, PLL, startup FSM code unchanged

**Step 1: Write integration test**

```python
# tests/sim/test_ncm_top.py
"""Smoke test: NCM top-level module elaborates without errors."""
import unittest
from amaranth import *
from gateware.top import RadioTop
from gateware.platform import VahyaPlatform

class TestNcmTopElaboration(unittest.TestCase):
    def test_elaborates(self):
        """RadioTop with NCM descriptors should elaborate without errors."""
        platform = VahyaPlatform()
        top = RadioTop()
        # This will raise if there are Amaranth elaboration errors
        # We can't fully simulate without hardware, but elaboration
        # catches wiring errors, signal width mismatches, etc.
        try:
            from amaranth.back import rtlil
            rtlil.convert(top, platform=platform)
        except Exception as e:
            self.fail(f"Elaboration failed: {e}")

if __name__ == "__main__":
    unittest.main()
```

**Step 2-5:** Modify top.py, run elaboration test, commit.

```bash
git add gateware/top.py gateware/usb/device.py tests/sim/test_ncm_top.py
git commit -m "feat: integrate NCM transport into RadioTop"
```

---

### Task 9: Hardware Build & Smoke Test

**Files:** No new files. Verification only.

**Step 1: Build the FPGA bitstream**

```bash
cd /home/rfsoc/exp/Vahya/radio_gateware
source /home/rfsoc/exp/Vahya/vahya/env
make gateware
```

Expected: Bitstream generated in `build/top.bit`. Check yosys/nextpnr resource usage stays under 50% LUT utilization.

**Step 2: Flash and verify USB enumeration**

```bash
make program
# Wait for device to enumerate
lsusb -d 1209:0001 -v 2>/dev/null | head -30
```

Expected: Device appears with `bDeviceClass = Communications`, `bDeviceSubClass = NCM`.

**Step 3: Verify Linux host recognizes NCM**

```bash
# Check kernel binds cdc_ncm driver
dmesg | tail -20
ip link show  # Should show new ethX or usbX interface
```

Expected: Linux creates a network interface. Verify with:
```bash
sudo dhclient ethX   # or the interface name
ip addr show ethX    # Should get 192.168.73.2
ping -c 3 192.168.73.1  # ICMP echo (if implemented)
```

**Step 4: Commit build verification**

```bash
git commit --allow-empty -m "verify: NCM gateware builds and enumerates on Linux"
```

---

## Phase 2: Android NcmTransport

All Android files in `/home/rfsoc/exp/Vahya/radio_gateware/VahyaDroid/`.

---

### Task 10: NTB16 Parser (C++ Utility)

**Files:**
- Create: `app/src/main/cpp/ncm/ntb_parser.h`
- Create: `app/src/main/cpp/ncm/ntb_parser.cpp`
- Create: `app/src/main/cpp/ncm/ntb_parser_test.cpp`

**Context:** Standalone C++ NTB16 parser. Extracts datagrams from NTB bytes, strips Eth+IP+UDP headers, returns payload + UDP port for each datagram.

```cpp
// ncm/ntb_parser.h
#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>

struct NtbDatagram {
    uint16_t udpPort;
    const uint8_t *payload;
    size_t payloadLen;
};

class NtbParser {
public:
    // Parse an NTB16 buffer. Returns number of datagrams found (0 on error).
    // Datagrams are valid only while `data` pointer is valid.
    int parse(const uint8_t *data, size_t len, std::vector<NtbDatagram> &out);
};
```

Test with a standalone test binary before integrating into the app.

```bash
git add app/src/main/cpp/ncm/
git commit -m "feat: add NTB16 parser for Android NCM transport"
```

---

### Task 11: NTB16 Builder (C++ Utility)

**Files:**
- Create: `app/src/main/cpp/ncm/ntb_builder.h`
- Create: `app/src/main/cpp/ncm/ntb_builder.cpp`

**Context:** Builds NTB16 frames from IQ data and control commands. Constructs NTH16 + Ethernet + IPv4 + UDP + payload + NDP16.

```cpp
// ncm/ntb_builder.h
#pragma once
#include <cstdint>
#include <cstddef>

class NtbBuilder {
public:
    NtbBuilder();

    // Build an NTB containing one UDP datagram.
    // Returns total NTB size written to `out`.
    size_t build(uint8_t *out, size_t maxLen,
                 uint16_t udpPort,
                 const uint8_t *payload, size_t payloadLen);

    // Build an NTB containing two UDP datagrams (dual-band TX).
    size_t buildDual(uint8_t *out, size_t maxLen,
                     uint16_t port1, const uint8_t *payload1, size_t len1,
                     uint16_t port2, const uint8_t *payload2, size_t len2);

private:
    uint16_t sequence_;
    void writeEthIpUdp(uint8_t *buf, uint16_t udpPort, size_t payloadLen);
};
```

```bash
git add app/src/main/cpp/ncm/ntb_builder.*
git commit -m "feat: add NTB16 builder for Android NCM transport"
```

---

### Task 12: NcmTransport Class

**Files:**
- Create: `app/src/main/cpp/transport/ncm_transport.h`
- Create: `app/src/main/cpp/transport/ncm_transport.cpp`
- Modify: `app/src/main/cpp/CMakeLists.txt` — add new source files

**Context:** Implements `ITransport` interface using NCM protocol. Claims NCM Data interface (interface 1, alt setting 1), reads/writes NTBs via bulk endpoints, parses NTBs to extract IQ data, builds NTBs for TX + control.

Key differences from `UsbTransport`:
- Claims interface 1 (not 0), sets alt setting 1
- RX callback: calls `NtbParser::parse()`, routes by UDP port
- TX callback: calls `NtbBuilder::build()` with IQ data from ring buffer
- Control (SPI, LED, reset): builds control UDP packets on port 5550

The `ITransport` interface stays unchanged — `RadioEngine` doesn't know or care which transport is used.

```bash
git add app/src/main/cpp/transport/ncm_transport.* app/src/main/cpp/CMakeLists.txt
git commit -m "feat: add NcmTransport implementing ITransport over CDC NCM"
```

---

### Task 13: USB Device Filter & Transport Selection

**Files:**
- Modify: `app/src/main/res/xml/usb_device_filter.xml`
- Modify: `app/src/main/java/com/vahya/radio/usb/UsbPermissionHandler.kt` (or equivalent)
- Modify: `app/src/main/java/com/vahya/radio/native/VahyaNative.kt`

**Step 1: Update USB device filter**

```xml
<!-- Match CDC NCM device (class=2, subclass=13) -->
<resources>
    <usb-device vendor-id="4617" product-id="1"
        class="2" subclass="13" />
</resources>
```

**Step 2: Add transport selection logic**

In the native bridge, detect whether the USB device uses vendor-specific or NCM interface and instantiate the appropriate transport:

```kotlin
// Detect device type by interface class
val iface = device.getInterface(0)
val isNcm = iface.interfaceClass == 2 && iface.interfaceSubclass == 13
// Pass flag to native layer
nativeInit(fd, isNcm)
```

**Step 3: Test on Android device**

Build the APK, install on test phone, connect Vahya device, verify:
1. USB permission prompt appears
2. NCM interface is claimed
3. IQ data streams correctly
4. SPI control commands work (frequency set, status read)

```bash
git add app/src/main/res/xml/usb_device_filter.xml
git add app/src/main/java/com/vahya/radio/
git commit -m "feat: add NCM transport selection and USB device filter"
```

---

### Task 14: Android End-to-End Test

**Files:** No new files. Hardware verification.

**Step 1: Build and install APK**

```bash
cd /home/rfsoc/exp/Vahya/radio_gateware/VahyaDroid
./gradlew assembleDebug
adb install app/build/outputs/apk/debug/app-debug.apk
```

**Step 2: Connect device and verify**

1. Connect Vahya device via USB-C OTG
2. Grant USB permission when prompted
3. Verify RX IQ streaming (waterfall display should show signals)
4. Verify TX (PTT, CW keyer)
5. Verify control (frequency change, band switch, LED)
6. Monitor logcat for errors:

```bash
adb logcat -s NcmTransport RadioEngine
```

**Step 3: Test on multiple Android versions**

- Android 13 device (pre-fix, kernel path broken — our userspace path should work)
- Android 14 device (same)
- Android 15 device (kernel path fixed — verify no conflicts)

```bash
git commit --allow-empty -m "verify: Android NCM transport working end-to-end"
```

---

## Phase 3: iOS App (Future)

Tasks 15-20 cover the iOS app. These are deferred until Phase 1 and 2 are validated.

### Task 15: iOS Project Setup
- Create Xcode project with SwiftUI
- Add Network.framework dependency
- Create `UdpTransport.swift` using `NWConnection` with `.wiredEthernet`

### Task 16: Port C++ DSP Engine
- Add VahyaDroid C++ files to Xcode project via Swift-C++ interop
- Replace Oboe audio with AVAudioEngine adapter
- Build and link KissFFT

### Task 17: iOS Radio UI
- SwiftUI waterfall display (Metal or CoreGraphics)
- VFO, mode selector, band selector, PTT, S-meter

### Task 18: iOS End-to-End Test
- Test on iPhone 15/16 with USB-C
- Verify NCM auto-detection, DHCP, IQ streaming

---

## Phase 4: Cleanup

### Task 19: Remove Legacy Vendor-Specific Code

**Files:**
- Remove: `gateware/usb/vendor_requests.py` (replaced by `ncm/control_handler.py`)
- Modify: `gateware/usb/device.py` — keep only as import shim or remove
- Update: `host/lib/vahya.c` — port to NCM-based Linux operation

### Task 20: Documentation Update

**Files:**
- Modify: `docs/README-project-status.md` — document NCM migration
- Update: `Makefile` — update test targets
- Update: `README.md` — note NCM transport

---

## Dependency Graph

```
Task 1 (NCM Descriptors)
  └─► Task 7 (NCM Control Handler)
       └─► Task 8 (Top Integration)

Task 2 (NTB Framer)
  └─► Task 8 (Top Integration)

Task 3 (NTB Deframer)
  └─► Task 8 (Top Integration)

Task 4 (ARP Responder)
  └─► Task 8 (Top Integration)

Task 5 (DHCP Server)
  └─► Task 8 (Top Integration)

Task 6 (Control Handler)
  └─► Task 8 (Top Integration)

Task 8 (Top Integration)
  └─► Task 9 (HW Build & Smoke)

Task 10 (NTB Parser C++)  ─┐
Task 11 (NTB Builder C++) ─┤
                            └─► Task 12 (NcmTransport)
                                 └─► Task 13 (Filter & Selection)
                                      └─► Task 14 (Android E2E)

Task 9 + Task 14 ─► Phase 3 (iOS) ─► Phase 4 (Cleanup)
```

**Tasks 1-7 can be developed in parallel** (independent modules with tests).
**Task 8** depends on all of Tasks 1-7.
**Tasks 10-11 can run in parallel with Phase 1.**
