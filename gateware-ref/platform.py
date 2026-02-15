"""
Vahya Mini v1.0b Platform Definition for Amaranth / LUNA

Defines all hardware resources for the Vahya radio gateware project:
  - 26 MHz system clock (from AT86RF215 CLKOUT)
  - RGB LED (active-low)
  - USB3343 ULPI PHY
  - AT86RF215IQ SPI, control, LVDS RX/TX
"""

import os
import logging
import subprocess
import gzip
import serial
import time
from pathlib import Path

from amaranth import *
from amaranth.build import *
from amaranth.vendor import LatticeECP5Platform
from luna.gateware.architecture.car import LunaECP5DomainGenerator


__all__ = ["VahyaPlatform"]


class VahyaDomainGenerator(LunaECP5DomainGenerator):
    """ECP5 clock domain generator using the internal oscillator (OSCG).

    The AT86RF215 CLKOUT defaults to disabled on power-up (RF_CLKO=0x00),
    so we cannot rely on it as the initial system clock.  Instead, the ECP5
    internal oscillator (OSCG) at ~25.83 MHz (310 MHz / 12) feeds the PLL:

        f_VCO   ~ 775 MHz   (25.83 MHz x 10 / 1 x 3)
        CLKOP   ~ 258 MHz   (fast domain)
        CLKOS   ~ 129 MHz   (sync domain)

    The ``usb`` clock domain is created here but its clock signal is **not**
    driven.  LUNA's ``UTMITranslator`` (inside ``USBDevice``) drives
    ``ClockSignal("usb")`` directly from the USB3343's 60 MHz ULPI CLK
    output, avoiding a double-driver conflict with Amaranth 0.5.
    """

    def elaborate(self, platform):
        m = Module()

        # Merge platform defaults with any caller overrides.
        new_clock_frequencies = platform.DEFAULT_CLOCK_FREQUENCIES_MHZ.copy()
        if self.clock_frequencies:
            new_clock_frequencies.update(self.clock_frequencies)
        self.clock_frequencies = new_clock_frequencies

        # -- Create clock domains ------------------------------------------
        # Note: the ``usb`` domain is NOT created here.  It must be created
        # in the top-level module so that LUNA's UTMITranslator (inside
        # USBDevice) can drive both its clock and reset from the ULPI PHY.
        m.domains.fast = ClockDomain("fast")
        m.domains.sync = ClockDomain("sync")

        # -- ECP5 internal oscillator (OSCG) as clock source ----------------
        # OSCG: ~310 MHz / 12 = ~25.83 MHz (no external clock dependency).
        # AT86RF215 CLKOUT is disabled at power-up, so we cannot use it
        # until the radio is configured via SPI over USB.
        oscg_clk = Signal()
        m.submodules.oscg = Instance("OSCG",
            p_DIV=12,
            o_OSC=oscg_clk,
        )

        # -- PLL signals ---------------------------------------------------
        pll_lock     = Signal()
        clk_260      = Signal()
        clk_130      = Signal()
        clk_60       = Signal()

        # PLL parameters for ~25.83 MHz (OSCG) -> VCO ~775 MHz:
        #   CLKI_DIV   = 1   -> f_PFD  ~ 25.83 MHz
        #   CLKFB_DIV  = 10  -> f_CLKOP ~ 258.3 MHz
        #   CLKOP_DIV  = 3   -> f_VCO   ~ 775 MHz
        #   CLKOS_DIV  = 6   -> f_CLKOS ~ 129.2 MHz
        #   CLKOS2_DIV = 13  -> f_CLKOS2~  59.6 MHz
        m.submodules.pll = Instance("EHXPLLL",
            # Clock in
            i_CLKI=oscg_clk,

            # Generated clock outputs
            o_CLKOP=clk_260,
            o_CLKOS=clk_130,
            o_CLKOS2=clk_60,

            # Status
            o_LOCK=pll_lock,

            # PLL parameters
            p_PLLRST_ENA="DISABLED",
            p_INTFB_WAKE="DISABLED",
            p_STDBY_ENABLE="DISABLED",
            p_DPHASE_SOURCE="DISABLED",
            p_CLKOS3_FPHASE=0,
            p_CLKOS3_CPHASE=0,
            p_CLKOS2_FPHASE=0,
            p_CLKOS2_CPHASE=12,
            p_CLKOS_FPHASE=0,
            p_CLKOS_CPHASE=5,
            p_CLKOP_FPHASE=0,
            p_CLKOP_CPHASE=2,
            p_PLL_LOCK_MODE=0,
            p_CLKOS_TRIM_DELAY="0",
            p_CLKOS_TRIM_POL="FALLING",
            p_CLKOP_TRIM_DELAY="0",
            p_CLKOP_TRIM_POL="FALLING",
            p_OUTDIVIDER_MUXD="DIVD",
            p_CLKOS3_ENABLE="DISABLED",
            p_OUTDIVIDER_MUXC="DIVC",
            p_CLKOS2_ENABLE="ENABLED",
            p_OUTDIVIDER_MUXB="DIVB",
            p_CLKOS_ENABLE="ENABLED",
            p_OUTDIVIDER_MUXA="DIVA",
            p_CLKOP_ENABLE="ENABLED",
            p_CLKOS3_DIV=1,
            p_CLKOS2_DIV=13,
            p_CLKOS_DIV=6,
            p_CLKOP_DIV=3,
            p_CLKFB_DIV=10,
            p_CLKI_DIV=1,
            p_FEEDBK_PATH="CLKOP",

            # Internal feedback
            i_CLKFB=clk_260,

            # Control signals
            i_RST=0,
            i_PHASESEL0=0,
            i_PHASESEL1=0,
            i_PHASEDIR=0,
            i_PHASESTEP=0,
            i_PHASELOADREG=0,
            i_STDBY=0,
            i_PLLWAKESYNC=0,

            # Output Enables
            i_ENCLKOP=0,
            i_ENCLKOS=0,
            i_ENCLKOS2=0,
            i_ENCLKOS3=0,

            # Synthesis attributes
            a_FREQUENCY_PIN_CLKI="25.830000",
            a_FREQUENCY_PIN_CLKOP="258.300000",
            a_FREQUENCY_PIN_CLKOS="129.200000",
            a_FREQUENCY_PIN_CLKOS2="59.600000",
            a_ICP_CURRENT="9",
            a_LPF_RESISTOR="8",
        )

        # -- Connect clock domains -----------------------------------------
        # fast and sync are driven by the PLL.
        # usb is intentionally NOT driven here; the ULPI translator will
        # connect it to the USB3343 PHY's 60 MHz clock output.
        m.d.comb += [
            ClockSignal("fast") .eq(clk_260),
            ClockSignal("sync") .eq(clk_130),
        ]

        # -- Reset logic ---------------------------------------------------
        # Hold fast/sync in reset until the PLL locks.
        m.d.comb += [
            ResetSignal("sync") .eq(~pll_lock),
            ResetSignal("fast") .eq(~pll_lock),
        ]

        return m


class VahyaPlatform(LatticeECP5Platform):
    name        = "Vahya v1.0b"
    device      = "LFE5U-25F"
    package     = "BG256"
    speed       = "7"

    DEFAULT_CLOCK_FREQUENCIES_MHZ = {
        "fast": 260,
        "sync": 130,
        "usb":  60,
    }

    resources = [
        # ── AT86RF215 CLKOUT (26 MHz, disabled at power-up) ──────────
        # Not used as system clock -- OSCG drives PLL instead.
        # Available for monitoring after RF_CLKO is configured via SPI.
        Resource("clk", 0,
            Pins("J14", dir="i"),
            Clock(26e6),
            Attrs(IO_TYPE="LVCMOS33")),

        # ── RGB LED (active-low) ────────────────────────────────────
        Resource("rgb_led", 0,
            Subsignal("r", Pins("B13", dir="o")),
            Subsignal("g", Pins("B14", dir="o")),
            Subsignal("b", Pins("B12", dir="o")),
            Attrs(IO_TYPE="LVCMOS33")),

        # ── ULPI interface to USB3343 ───────────────────────────────
        Resource("ulpi", 0,
            Subsignal("data", Pins("G1 F2 F1 E2 E1 D1 C2 C1", dir="io")),
            Subsignal("clk",  Pins("K1",  dir="i")),
            Subsignal("dir",  Pins("J5",  dir="i")),
            Subsignal("nxt",  Pins("G2",  dir="i")),
            Subsignal("stp",  Pins("J4",  dir="o")),
            Subsignal("rst",  PinsN("H2", dir="o")),
            Attrs(IO_TYPE="LVCMOS33")),

        # ── AT86RF215IQ SPI ─────────────────────────────────────────
        Resource("at86_spi", 0,
            Subsignal("clk",  Pins("K14", dir="o")),
            Subsignal("mosi", Pins("F16", dir="o")),
            Subsignal("miso", Pins("G16", dir="i")),
            Subsignal("cs",   PinsN("E16", dir="o")),
            Attrs(IO_TYPE="LVCMOS33")),

        # ── AT86RF215IQ Control ─────────────────────────────────────
        Resource("at86_ctrl", 0,
            Subsignal("rstn", PinsN("C14", dir="o")),
            Subsignal("irq",  Pins("F15", dir="i")),
            Attrs(IO_TYPE="LVCMOS33")),

        # ── AT86RF215IQ LVDS RX ─────────────────────────────────────
        # True LVDS differential inputs with 100Ω internal termination.
        # The AT86RF215 SLVDS output (1.2V CMV ± ~200mV) is well within
        # the ECP5 LVDS receiver's common-mode range (0.1-2.35V) and
        # differential threshold (100mV).  LVCMOS33 single-ended input
        # does NOT work: the SLVDS swing (1.0-1.4V) falls in the
        # LVCMOS33 undefined zone (VIL=0.8V, VIH=2.0V), causing
        # unreliable clock recovery (~9 MHz effective instead of 64 MHz).
        Resource("at86_rxclk", 0,
            DiffPairs(p="J16", n="J15", dir="i"),
            Clock(64e6),
            Attrs(IO_TYPE="LVDS", DIFFRESISTOR="100")),
        Resource("at86_rxd09", 0,
            DiffPairs(p="D16", n="E15", dir="i"),
            Attrs(IO_TYPE="LVDS", DIFFRESISTOR="100")),
        Resource("at86_rxd24", 0,
            DiffPairs(p="K16", n="K15", dir="i"),
            Attrs(IO_TYPE="LVDS", DIFFRESISTOR="100")),

        # ── AT86RF215IQ LVDS TX ─────────────────────────────────────
        # Pseudo-differential LVCMOS33D outputs.  Bank 2 has 3.3V VCCIO
        # so true LVDS25 output is not possible; LVCMOS33D provides a
        # complementary pair that drives the AT86RF215 LVDS RX inputs
        # through the board's termination network.
        Resource("at86_txclk", 0,
            DiffPairs(p="B16", n="B15", dir="o"),
            Attrs(IO_TYPE="LVCMOS33D", DRIVE="4")),
        Resource("at86_txd", 0,
            DiffPairs(p="C16", n="C15", dir="o"),
            Attrs(IO_TYPE="LVCMOS33D", DRIVE="4")),
    ]

    connectors = []

    default_clk            = "clk"
    default_usb_connection = "ulpi"
    clock_domain_generator = VahyaDomainGenerator

    def _do_program(self, bitstream_path):
        """
        Program the FPGA via ESP32 using mpremote:
        1. Compress bitstream to gzip
        2. Upload via mpremote cp to ESP32 filesystem
        3. Issue ecp5.prog() via mpremote exec

        Environment variables:
          VahyaPlatformTTY -- serial port for ESP32 (default /dev/ttyACM0)
        """
        print("--- Vahya v1.0b Programmer ---")

        vahya_tty = os.environ.get("VahyaPlatformTTY", "/dev/ttyACM0")
        print(f"   TTY : {vahya_tty}")

        bitstream_path = Path(bitstream_path)
        if not bitstream_path.is_file():
            raise FileNotFoundError(f"Bitstream not found: {bitstream_path}")

        # 1. Compress
        gz_path = bitstream_path.with_suffix(".bit.gz")
        print(f"1. Compressing {bitstream_path.name} -> {gz_path.name} ...")
        with open(bitstream_path, "rb") as f_in:
            with gzip.open(gz_path, "wb") as f_out:
                f_out.writelines(f_in)

        # 2. Upload via mpremote
        print(f"2. Uploading via mpremote ...")
        subprocess.run(
            ["mpremote", "connect", vahya_tty,
             "cp", str(gz_path), ":/top.bit.gz"],
            check=True)

        # 3. Program FPGA
        print("3. Programming FPGA ...")
        subprocess.run(
            ["mpremote", "connect", vahya_tty,
             "exec", "import ecp5; print(ecp5.prog('/top.bit.gz'))"],
            check=True)

        print("Programming complete!")

    def toolchain_program(self, products, name, **kwargs):
        """Called by the Amaranth build system after synthesis."""
        with products.extract(f"{name}.bit") as bitstream_path:
            self._do_program(bitstream_path)
