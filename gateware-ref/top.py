"""Top-level gateware for Vahya AT86RF215 radio interface."""

import os
from amaranth import *
from amaranth.lib.cdc import FFSynchronizer

from luna import top_level_cli
from luna.gateware.usb.usb2.device import USBDevice
from luna.gateware.usb.usb2.endpoints.stream import (
    USBStreamInEndpoint, USBStreamOutEndpoint,
)

from .usb.device import (
    create_descriptors, MAX_PACKET_SIZE,
    EP_RX_900, EP_RX_2400, EP_TX_900, EP_TX_2400,
)
from .at86.spi_master import SPIMaster
from .at86.lvds_rx import AT86LVDSReceiver
from .at86.lvds_tx import AT86LVDSTransmitter
from .cdc.async_fifo_bridge import RxFIFOBridge, TxFIFOBridge
from .usb.vendor_requests import RadioVendorRequestHandler


class RadioTop(Elaboratable):
    """Top-level radio gateware with USB interface."""

    def elaborate(self, platform):
        m = Module()

        # Clock domain generation (creates fast, sync domains from OSCG PLL)
        m.submodules.car = platform.clock_domain_generator()

        # USB domain — created here so LUNA's UTMITranslator (inside
        # USBDevice) can drive its clock and reset from the ULPI PHY.
        m.domains.usb = ClockDomain("usb")

        # RGB LED (active-low)
        rgb = platform.request("rgb_led", 0)

        # ── AT86RF215 SPI master (sync domain, ~129.2 MHz) ──────────
        # AT86RF215 max SPI clock = 25 MHz, but the FPGA input path delay
        # for MISO (tDO + pin→fabric routing ≈ 40-50ns) limits reliable
        # operation.  Tested: half_period=7 (9.2 MHz) ~50% bit-7 errors,
        # half_period=8 (8.1 MHz) ~8% errors, half_period≥9 (≤7.2 MHz)
        # 0% errors.  Using 7 MHz for reliable operation.
        m.submodules.spi = spi = SPIMaster(clk_freq=129.2e6, spi_freq=7e6)

        spi_pins = platform.request("at86_spi", 0)
        m.d.comb += [
            spi_pins.clk.o  .eq(spi.spi_clk),
            spi_pins.mosi.o .eq(spi.spi_mosi),
            spi.spi_miso     .eq(spi_pins.miso.i),
            # PinsN inverts: cs.o=1 → pin LOW (active).  spi_cs_n is
            # already active-low (0=active), so negate to match PinsN.
            spi_pins.cs.o   .eq(~spi.spi_cs_n),
        ]

        # ── AT86RF215 control (reset, IRQ) ───────────────────────────
        at86_ctrl = platform.request("at86_ctrl", 0)

        # ── USB Device ───────────────────────────────────────────────
        ulpi = platform.request(platform.default_usb_connection)
        m.submodules.usb = usb = USBDevice(bus=ulpi)

        descriptors = create_descriptors()
        control_ep = usb.add_standard_control_endpoint(descriptors)

        # ── Vendor request handler (usb domain) ─────────────────────
        vendor_handler = RadioVendorRequestHandler()
        control_ep.add_request_handler(vendor_handler)

        # ════════════════════════════════════════════════════════════════
        # ── AT86RF215 startup controller (sync domain) ─────────────────
        # ════════════════════════════════════════════════════════════════
        # The AT86RF215 CLKOUT is disabled at power-up (RF_CLKO = 0x00).
        # The USB3343 needs CLKOUT as its reference clock.  This FSM
        # resets the AT86RF215, then enables 26 MHz CLKOUT via SPI
        # before USB can enumerate.
        #
        # Sequence: assert reset ~1 ms -> release -> wait ~2 ms ->
        #           SPI write RF_CLKO(0x0007) = 0x01 -> done

        SYNC_FREQ       = 129_200_000          # ~129.2 MHz from OSCG PLL
        RESET_CYCLES    = SYNC_FREQ // 1000    # ~1 ms
        BOOT_CYCLES     = SYNC_FREQ // 500     # ~2 ms

        startup_done    = Signal()
        startup_counter = Signal(range(max(RESET_CYCLES, BOOT_CYCLES)))
        startup_rst     = Signal()             # 1 = assert AT86 reset

        su_spi_addr     = Signal(13)
        su_spi_wdata    = Signal(8)
        su_spi_rw       = Signal()
        su_spi_start    = Signal()

        with m.FSM(domain="sync", name="startup"):
            with m.State("RESET_HOLD"):
                m.d.comb += startup_rst.eq(1)
                m.d.sync += startup_counter.eq(startup_counter + 1)
                with m.If(startup_counter == RESET_CYCLES - 1):
                    m.d.sync += startup_counter.eq(0)
                    m.next = "RESET_RELEASE"

            with m.State("RESET_RELEASE"):
                m.d.sync += startup_counter.eq(startup_counter + 1)
                with m.If(startup_counter == BOOT_CYCLES - 1):
                    m.d.sync += startup_counter.eq(0)
                    m.next = "SPI_CLKOUT"

            with m.State("SPI_CLKOUT"):
                # Write RF_CLKO register (0x0007) = 0x01 (26 MHz output)
                m.d.comb += [
                    su_spi_addr .eq(0x0007),
                    su_spi_wdata.eq(0x01),
                    su_spi_rw   .eq(0),
                    su_spi_start.eq(1),
                ]
                m.next = "SPI_WAIT"

            with m.State("SPI_WAIT"):
                m.d.comb += [
                    su_spi_addr .eq(0x0007),
                    su_spi_wdata.eq(0x01),
                    su_spi_rw   .eq(0),
                ]
                with m.If(spi.done):
                    m.next = "DONE"

            with m.State("DONE"):
                m.d.sync += startup_done.eq(1)

        # ════════════════════════════════════════════════════════════════
        # ── CDC: USB (60 MHz) ↔ sync (~129 MHz) for SPI control ──────
        # ════════════════════════════════════════════════════════════════
        # The vendor handler runs in the USB domain; the SPI master in
        # sync.  Without synchronisation, signals crossing domains cause
        # metastability → intermittent data corruption (bit-7 bug).

        # ── Forward path: start pulse (USB → sync) ───────────────────
        # 2-FF synchroniser resolves metastability on the start pulse.
        vh_start_sync = Signal()
        m.submodules += FFSynchronizer(
            vendor_handler.spi_start, vh_start_sync, o_domain="sync")

        # Rising-edge detector → one sync-cycle pulse
        vh_start_prev = Signal()
        m.d.sync += vh_start_prev.eq(vh_start_sync)
        vh_start_edge = Signal()
        m.d.comb += vh_start_edge.eq(vh_start_sync & ~vh_start_prev)

        # Capture addr / wdata / rw in sync domain on the edge.
        # The USB domain holds these stable (CMD → GUARD → BUSY),
        # so by the time the synchronised edge arrives (~2-3 sync
        # cycles later) they have been stable for multiple cycles.
        vh_addr_sync  = Signal(13)
        vh_wdata_sync = Signal(8)
        vh_rw_sync    = Signal()
        with m.If(vh_start_edge):
            m.d.sync += [
                vh_addr_sync .eq(vendor_handler.spi_addr),
                vh_wdata_sync.eq(vendor_handler.spi_wdata),
                vh_rw_sync   .eq(vendor_handler.spi_rw),
            ]

        # Delay start by 1 sync cycle so the captured registers
        # are available before the SPI master samples them.
        vh_start_go = Signal()
        m.d.sync += vh_start_go.eq(vh_start_edge)

        # ── Return path: busy (sync → USB) ───────────────────────────
        spi_busy_usb = Signal()
        m.submodules += FFSynchronizer(
            spi.busy, spi_busy_usb, o_domain="usb")

        # USB-domain "pending" flag prevents the vendor handler from
        # checking ~busy before the CDC-delayed busy has arrived.
        # Goes high immediately when start is sent; cleared once the
        # real busy propagates back through the synchroniser.
        usb_spi_pending = Signal()
        with m.If(vendor_handler.spi_start):
            m.d.usb += usb_spi_pending.eq(1)
        with m.Elif(spi_busy_usb):
            m.d.usb += usb_spi_pending.eq(0)

        usb_busy_ext = Signal()
        m.d.comb += usb_busy_ext.eq(usb_spi_pending | spi_busy_usb)

        # ── SPI + reset mux: startup FSM vs USB vendor handler ─────────
        # During startup, the USB domain has no clock so the vendor
        # handler outputs are inert.  After startup, hand control over.
        with m.If(~startup_done):
            m.d.comb += [
                spi.addr  .eq(su_spi_addr),
                spi.wdata .eq(su_spi_wdata),
                spi.rw    .eq(su_spi_rw),
                spi.start .eq(su_spi_start),
                at86_ctrl.rstn.o.eq(startup_rst),
            ]
        with m.Else():
            m.d.comb += [
                spi.addr  .eq(vh_addr_sync),
                spi.wdata .eq(vh_wdata_sync),
                spi.rw    .eq(vh_rw_sync),
                spi.start .eq(vh_start_go),
                vendor_handler.spi_rdata .eq(spi.rdata),
                vendor_handler.spi_busy  .eq(usb_busy_ext),
                # AT86 reset from vendor handler
                at86_ctrl.rstn.o.eq(vendor_handler.at86_rst),
            ]

        # ── Bulk IN endpoints (device -> host) ───────────────────────
        ep1_in = USBStreamInEndpoint(
            endpoint_number=EP_RX_900,
            max_packet_size=MAX_PACKET_SIZE)
        ep2_in = USBStreamInEndpoint(
            endpoint_number=EP_RX_2400,
            max_packet_size=MAX_PACKET_SIZE)
        usb.add_endpoint(ep1_in)
        usb.add_endpoint(ep2_in)

        # ── Bulk OUT endpoints (host -> device) ──────────────────────
        ep3_out = USBStreamOutEndpoint(
            endpoint_number=EP_TX_900,
            max_packet_size=MAX_PACKET_SIZE)
        ep4_out = USBStreamOutEndpoint(
            endpoint_number=EP_TX_2400,
            max_packet_size=MAX_PACKET_SIZE)
        usb.add_endpoint(ep3_out)
        usb.add_endpoint(ep4_out)

        # ════════════════════════════════════════════════════════════════
        # ── LVDS clock domains ──────────────────────────────────────────
        # ════════════════════════════════════════════════════════════════

        # --- at86_rx clock domain: derived from AT86RF215 RXCLK LVDS input ---
        rxclk = platform.request("at86_rxclk", 0)

        m.domains.at86_rx = ClockDomain("at86_rx")
        m.d.comb += [
            ClockSignal("at86_rx").eq(rxclk.i),
            ResetSignal("at86_rx").eq(ResetSignal("sync")),
        ]

        # --- at86_tx clock domain: sync/2 ≈ 64.6 MHz ---
        # Must be independent of RXCLK — the AT86RF215 needs TXCLK present
        # before TXPREP→TX transition, but RXCLK may not be stable during
        # state changes.  The 1% rate mismatch (64.6 vs 64.0 MHz) is
        # absorbed by TxFIFOBridge backpressure (FIFO stays near-full,
        # USB is NAKed when full, naturally rate-matching).
        at86_tx_div = Signal()
        m.d.sync += at86_tx_div.eq(~at86_tx_div)

        m.domains.at86_tx = ClockDomain("at86_tx")
        m.d.comb += [
            ClockSignal("at86_tx").eq(at86_tx_div),
            ResetSignal("at86_tx").eq(ResetSignal("sync")),
        ]

        # ════════════════════════════════════════════════════════════════
        # ── LVDS RX data path ───────────────────────────────────────────
        # ════════════════════════════════════════════════════════════════

        # Request LVDS RX data pins
        rxd09 = platform.request("at86_rxd09", 0)
        rxd24 = platform.request("at86_rxd24", 0)

        # DDR input registers (ECP5 IDDRX1F primitives)
        # The AT86RF215 SKEWDRV=2 (default) provides center-aligned
        # data with 3.906ns skew, giving adequate setup/hold margin
        # at 64 MHz DDR without needing DELAYG.
        rise_09 = Signal()
        fall_09 = Signal()
        rise_24 = Signal()
        fall_24 = Signal()

        m.submodules.iddr_09 = Instance("IDDRX1F",
            i_D=rxd09.i,
            i_SCLK=ClockSignal("at86_rx"),
            i_RST=Const(0),
            o_Q0=rise_09,
            o_Q1=fall_09,
        )

        m.submodules.iddr_24 = Instance("IDDRX1F",
            i_D=rxd24.i,
            i_SCLK=ClockSignal("at86_rx"),
            i_RST=Const(0),
            o_Q0=rise_24,
            o_Q1=fall_24,
        )

        # LVDS receivers (at86_rx domain)
        m.submodules.lvds_rx_09 = lvds_rx_09 = AT86LVDSReceiver()
        m.submodules.lvds_rx_24 = lvds_rx_24 = AT86LVDSReceiver()

        m.d.comb += [
            lvds_rx_09.ddr_rise.eq(rise_09),
            lvds_rx_09.ddr_fall.eq(fall_09),
            lvds_rx_24.ddr_rise.eq(rise_24),
            lvds_rx_24.ddr_fall.eq(fall_24),
        ]

        # RX FIFO bridges (at86_rx -> usb domain)
        m.submodules.rx_fifo_09 = rx_fifo_09 = RxFIFOBridge()
        m.submodules.rx_fifo_24 = rx_fifo_24 = RxFIFOBridge()

        # Wire LVDS RX 900 -> RxFIFOBridge 900
        m.d.comb += [
            rx_fifo_09.w_i_sample.eq(lvds_rx_09.i_sample),
            rx_fifo_09.w_q_sample.eq(lvds_rx_09.q_sample),
            rx_fifo_09.w_valid   .eq(lvds_rx_09.sample_valid),
        ]

        # Wire LVDS RX 2400 -> RxFIFOBridge 2400
        m.d.comb += [
            rx_fifo_24.w_i_sample.eq(lvds_rx_24.i_sample),
            rx_fifo_24.w_q_sample.eq(lvds_rx_24.q_sample),
            rx_fifo_24.w_valid   .eq(lvds_rx_24.sample_valid),
        ]

        # Wire RxFIFOBridge 900 -> USB IN EP1 (900 MHz RX)
        m.d.comb += [
            ep1_in.stream.payload .eq(rx_fifo_09.r_data),
            ep1_in.stream.valid   .eq(rx_fifo_09.r_valid),
            ep1_in.stream.first   .eq(0),
            ep1_in.stream.last    .eq(0),
            rx_fifo_09.r_ready    .eq(ep1_in.stream.ready),
        ]

        # Wire RxFIFOBridge 2400 -> USB IN EP2 (2.4 GHz RX)
        m.d.comb += [
            ep2_in.stream.payload .eq(rx_fifo_24.r_data),
            ep2_in.stream.valid   .eq(rx_fifo_24.r_valid),
            ep2_in.stream.first   .eq(0),
            ep2_in.stream.last    .eq(0),
            rx_fifo_24.r_ready    .eq(ep2_in.stream.ready),
        ]

        # ════════════════════════════════════════════════════════════════
        # ── LVDS TX data path ───────────────────────────────────────────
        # ════════════════════════════════════════════════════════════════

        # Request LVDS TX pins
        txclk = platform.request("at86_txclk", 0)
        txd   = platform.request("at86_txd", 0)

        # TX FIFO bridges (usb -> at86_tx domain)
        m.submodules.tx_fifo_09 = tx_fifo_09 = TxFIFOBridge()
        m.submodules.tx_fifo_24 = tx_fifo_24 = TxFIFOBridge()

        # Wire USB OUT EP3 (900 MHz TX) -> TxFIFOBridge 900
        m.d.comb += [
            tx_fifo_09.w_data  .eq(ep3_out.stream.payload),
            tx_fifo_09.w_valid .eq(ep3_out.stream.valid),
            ep3_out.stream.ready .eq(tx_fifo_09.w_ready),
        ]

        # Wire USB OUT EP4 (2.4 GHz TX) -> TxFIFOBridge 2400
        m.d.comb += [
            tx_fifo_24.w_data  .eq(ep4_out.stream.payload),
            tx_fifo_24.w_valid .eq(ep4_out.stream.valid),
            ep4_out.stream.ready .eq(tx_fifo_24.w_ready),
        ]

        # LVDS transmitter (at86_tx domain) — shared between both bands
        m.submodules.lvds_tx = lvds_tx = AT86LVDSTransmitter()

        # Mux between the two TX FIFO bridges: priority to 900 MHz band,
        # fall back to 2.4 GHz if 900 has no data. The AT86RF215 can only
        # transmit one band at a time, so only one bridge will have data.
        tx_sel_24 = Signal()  # 1 = using 2.4 GHz bridge

        with m.If(tx_fifo_09.r_valid):
            m.d.comb += [
                lvds_tx.i_sample    .eq(tx_fifo_09.r_i_sample),
                lvds_tx.q_sample    .eq(tx_fifo_09.r_q_sample),
                lvds_tx.sample_valid.eq(tx_fifo_09.r_valid & lvds_tx.sample_ready),
                tx_fifo_09.r_ready  .eq(lvds_tx.sample_ready),
                tx_fifo_24.r_ready  .eq(0),
                tx_sel_24           .eq(0),
            ]
        with m.Else():
            m.d.comb += [
                lvds_tx.i_sample    .eq(tx_fifo_24.r_i_sample),
                lvds_tx.q_sample    .eq(tx_fifo_24.r_q_sample),
                lvds_tx.sample_valid.eq(tx_fifo_24.r_valid & lvds_tx.sample_ready),
                tx_fifo_24.r_ready  .eq(lvds_tx.sample_ready),
                tx_fifo_09.r_ready  .eq(0),
                tx_sel_24           .eq(1),
            ]

        # DDR output for TX data (ECP5 ODDRX1F primitive, output to pin)
        m.submodules.oddr_txd = Instance("ODDRX1F",
            i_D0=lvds_tx.ddr_rise,
            i_D1=lvds_tx.ddr_fall,
            i_SCLK=ClockSignal("at86_tx"),
            i_RST=Const(0),
            o_Q=txd.o,
        )

        # DDR output for TX clock (generates clean forwarded clock to AT86RF215)
        m.submodules.oddr_txclk = Instance("ODDRX1F",
            i_D0=Const(1),
            i_D1=Const(0),
            i_SCLK=ClockSignal("at86_tx"),
            i_RST=Const(0),
            o_Q=txclk.o,
        )

        # ── USB connect + LED indicators ─────────────────────────────
        m.d.comb += [
            usb.connect         .eq(1),
            usb.full_speed_only .eq(1 if os.getenv("LUNA_FULL_ONLY") else 0),
        ]

        with m.If(vendor_handler.led_set):
            m.d.comb += [
                rgb.r.o .eq(~vendor_handler.led_r),
                rgb.g.o .eq(~vendor_handler.led_g),
                rgb.b.o .eq(~vendor_handler.led_b),
            ]
        with m.Else():
            m.d.comb += [
                rgb.r.o .eq(~usb.tx_activity_led),
                rgb.g.o .eq(~usb.rx_activity_led),
                rgb.b.o .eq(~usb.suspended),
            ]

        # ── Debug instrumentation ──────────────────────────────────────
        # RXCLK heartbeat: counter in at86_rx domain, MSB toggles ~4Hz
        rxclk_hb = Signal(24)
        m.d.at86_rx += rxclk_hb.eq(rxclk_hb + 1)

        # Synchronize debug signals from at86_rx to usb domain
        rxclk_hb_usb = Signal()
        frame_sync_09_usb = Signal()
        frame_sync_24_usb = Signal()
        fifo_09_rvalid_usb = Signal()
        fifo_24_rvalid_usb = Signal()

        m.submodules += FFSynchronizer(
            rxclk_hb[23], rxclk_hb_usb, o_domain="usb")
        m.submodules += FFSynchronizer(
            lvds_rx_09.frame_sync, frame_sync_09_usb, o_domain="usb")
        m.submodules += FFSynchronizer(
            lvds_rx_24.frame_sync, frame_sync_24_usb, o_domain="usb")
        m.submodules += FFSynchronizer(
            rx_fifo_09.r_valid, fifo_09_rvalid_usb, o_domain="usb")
        m.submodules += FFSynchronizer(
            rx_fifo_24.r_valid, fifo_24_rvalid_usb, o_domain="usb")

        # Synchronize FIFO overflow/underflow flags to usb domain
        rx09_overflow_usb = Signal()
        rx24_overflow_usb = Signal()
        tx09_underflow_usb = Signal()
        tx24_underflow_usb = Signal()

        m.submodules += FFSynchronizer(
            rx_fifo_09.overflow, rx09_overflow_usb, o_domain="usb")
        m.submodules += FFSynchronizer(
            rx_fifo_24.overflow, rx24_overflow_usb, o_domain="usb")
        m.submodules += FFSynchronizer(
            tx_fifo_09.underflow, tx09_underflow_usb, o_domain="usb")
        m.submodules += FFSynchronizer(
            tx_fifo_24.underflow, tx24_underflow_usb, o_domain="usb")

        # Synchronize AT86RF215 IRQ pin to usb domain
        irq_pin_usb = Signal()
        m.submodules += FFSynchronizer(
            at86_ctrl.irq.i, irq_pin_usb, o_domain="usb")
        m.d.comb += vendor_handler.dbg_irq_pin.eq(irq_pin_usb)

        m.d.comb += [
            vendor_handler.dbg_rxclk_active    .eq(rxclk_hb_usb),
            vendor_handler.dbg_frame_sync_09   .eq(frame_sync_09_usb),
            vendor_handler.dbg_frame_sync_24   .eq(frame_sync_24_usb),
            vendor_handler.dbg_fifo_24_has_data.eq(fifo_24_rvalid_usb),

            # FIFO overflow/underflow/has_data for GET_STATUS
            vendor_handler.dbg_rx09_overflow   .eq(rx09_overflow_usb),
            vendor_handler.dbg_rx24_overflow   .eq(rx24_overflow_usb),
            vendor_handler.dbg_tx09_underflow  .eq(tx09_underflow_usb),
            vendor_handler.dbg_tx24_underflow  .eq(tx24_underflow_usb),
            vendor_handler.dbg_rx09_has_data   .eq(fifo_09_rvalid_usb),
            vendor_handler.dbg_rx24_has_data   .eq(fifo_24_rvalid_usb),
        ]

        return m


if __name__ == "__main__":
    os.environ["LUNA_PLATFORM"] = "gateware.platform:VahyaPlatform"
    top_level_cli(RadioTop)
