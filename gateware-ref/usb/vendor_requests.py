"""USB vendor request handler for AT86RF215 radio control.

Bridges EP0 control transfers to:
  - SPI master (register read/write)
  - LED control
  - AT86RF215 reset
  - Status readback

All logic runs in the ``usb`` clock domain.  The SPI master runs in ``sync``.
CDC synchronisation (FFSynchronizer + registered handshake) is performed in
top.py to avoid metastability on the domain crossing.

Pattern follows the proven VahyaVendorHandler from at86_firmware.
"""

from amaranth import *

from luna.gateware.usb.usb2.request import USBRequestHandler

from usb_protocol.types import USBRequestType


# Vendor request codes
REQ_SPI_WRITE    = 0x01
REQ_SPI_READ     = 0x02
REQ_GET_STATUS   = 0x03
REQ_SET_LED      = 0x04
REQ_RESET_AT86   = 0x05
REQ_GET_RX_DEBUG = 0x06
REQ_GET_IRQ      = 0x07

# Firmware / gateware version constants for GET_STATUS
FW_VERSION_MAJOR = 0
FW_VERSION_MINOR = 1


class RadioVendorRequestHandler(USBRequestHandler):
    """Handle vendor-specific control requests for the radio.

    External signals (directly connected in top.py):
        spi_addr   : Signal(13), out  -- SPI register address
        spi_wdata  : Signal(8),  out  -- SPI write data
        spi_rdata  : Signal(8),  in   -- SPI read data (latched by SPI master)
        spi_rw     : Signal(1),  out  -- 0 = write, 1 = read
        spi_start  : Signal(1),  out  -- pulse to begin SPI transaction
        spi_done   : Signal(1),  in   -- pulses when SPI transaction completes
        spi_busy   : Signal(1),  in   -- high while SPI in progress

        led_r      : Signal(1),  out  -- red LED control
        led_g      : Signal(1),  out  -- green LED control
        led_b      : Signal(1),  out  -- blue LED control
        led_set    : Signal(1),  out  -- high when vendor LED override is active

        at86_rst   : Signal(1),  out  -- pulse high to reset AT86RF215
    """

    def __init__(self):
        super().__init__()

        # SPI interface signals
        self.spi_addr  = Signal(13)
        self.spi_wdata = Signal(8)
        self.spi_rdata = Signal(8)
        self.spi_rw    = Signal()
        self.spi_start = Signal()
        self.spi_done  = Signal()
        self.spi_busy  = Signal()

        # LED control
        self.led_r   = Signal()
        self.led_g   = Signal()
        self.led_b   = Signal()
        self.led_set = Signal()

        # AT86 reset
        self.at86_rst = Signal()

        # AT86RF215 IRQ pin (synchronized to usb domain in top.py)
        self.dbg_irq_pin = Signal()  # Raw AT86RF215 IRQ pin state

        # Debug inputs (synchronized to usb domain in top.py)
        self.dbg_rxclk_active  = Signal()  # RXCLK heartbeat toggle
        self.dbg_frame_sync_09 = Signal()  # 900 MHz LVDS RX locked
        self.dbg_frame_sync_24 = Signal()  # 2.4 GHz LVDS RX locked
        self.dbg_fifo_24_has_data = Signal()  # RX FIFO 2400 has data

        # FIFO status inputs (synchronized to usb domain in top.py)
        self.dbg_rx09_overflow = Signal()
        self.dbg_rx24_overflow = Signal()
        self.dbg_tx09_underflow = Signal()
        self.dbg_tx24_underflow = Signal()
        self.dbg_rx09_has_data = Signal()
        self.dbg_rx24_has_data = Signal()

    def elaborate(self, platform):
        m = Module()
        interface = self.interface
        setup = interface.setup

        # Internal state
        active = Signal()

        # Latched copies of setup fields (kept stable through entire request)
        spi_addr_reg  = Signal(13)
        spi_rdata_reg = Signal(8)

        # Byte index for multi-byte IN responses
        byte_idx = Signal(3)

        # Reset pulse counter (hold reset for ~256 USB clock cycles ~ 4us)
        reset_cnt = Signal(8)
        with m.If(reset_cnt != 0):
            m.d.usb += reset_cnt.eq(reset_cnt - 1)
        m.d.comb += self.at86_rst.eq(reset_cnt != 0)

        # Claim when active or when a matching vendor request arrives.
        # This follows the proven pattern from the working at86_firmware.
        is_vendor = setup.type == USBRequestType.VENDOR
        m.d.comb += interface.claim.eq(active | (setup.received & is_vendor))

        # Default: deassert one-shot SPI signals
        m.d.usb += self.spi_start.eq(0)

        with m.FSM(domain="usb"):

            # ---- IDLE ----
            with m.State("IDLE"):
                m.d.usb += active.eq(0)

                with m.If(setup.received & is_vendor):
                    m.d.usb += [
                        active.eq(1),
                        spi_addr_reg.eq(setup.index[:13]),
                    ]
                    with m.Switch(setup.request):
                        with m.Case(REQ_SPI_WRITE):
                            m.next = "SPI_WRITE_CMD"
                        with m.Case(REQ_SPI_READ):
                            m.next = "SPI_READ_CMD"
                        with m.Case(REQ_GET_STATUS):
                            m.next = "GET_STATUS_WAIT"
                        with m.Case(REQ_SET_LED):
                            m.next = "SET_LED"
                        with m.Case(REQ_RESET_AT86):
                            m.next = "RESET_AT86"
                        with m.Case(REQ_GET_RX_DEBUG):
                            m.next = "RX_DEBUG_WAIT"
                        with m.Case(REQ_GET_IRQ):
                            m.next = "IRQ_WAIT"
                        with m.Default():
                            m.next = "UNHANDLED"

            # ----------------------------------------------------------------
            # SPI_WRITE (0x01): wValue = data byte, wIndex = register addr
            # OUT request with no data phase (wLength=0)
            # ----------------------------------------------------------------
            with m.State("SPI_WRITE_CMD"):
                m.d.usb += [
                    self.spi_start.eq(1),
                ]
                m.d.comb += [
                    self.spi_addr .eq(spi_addr_reg),
                    self.spi_wdata.eq(setup.value[:8]),
                    self.spi_rw   .eq(0),
                ]
                # Guard cycle: let start propagate to sync domain
                m.next = "SPI_WRITE_GUARD"

            with m.State("SPI_WRITE_GUARD"):
                m.d.comb += [
                    self.spi_addr .eq(spi_addr_reg),
                    self.spi_wdata.eq(setup.value[:8]),
                    self.spi_rw   .eq(0),
                ]
                m.next = "SPI_WRITE_BUSY"

            with m.State("SPI_WRITE_BUSY"):
                m.d.comb += [
                    self.spi_addr .eq(spi_addr_reg),
                    self.spi_wdata.eq(setup.value[:8]),
                    self.spi_rw   .eq(0),
                ]
                # Use ~busy (level) instead of done (pulse) for reliable
                # CDC between sync (130 MHz) and usb (60 MHz) domains.
                with m.If(~self.spi_busy):
                    m.next = "STATUS_OUT_ZLP"

            # ----------------------------------------------------------------
            # SPI_READ (0x02): wIndex = register addr, returns 1 byte
            # IN request with 1-byte data phase
            # ----------------------------------------------------------------
            with m.State("SPI_READ_CMD"):
                m.d.usb += [
                    self.spi_start.eq(1),
                ]
                m.d.comb += [
                    self.spi_addr.eq(spi_addr_reg),
                    self.spi_rw  .eq(1),
                ]
                # Guard cycle: let start propagate to sync domain
                m.next = "SPI_READ_GUARD"

            with m.State("SPI_READ_GUARD"):
                m.d.comb += [
                    self.spi_addr.eq(spi_addr_reg),
                    self.spi_rw  .eq(1),
                ]
                m.next = "SPI_READ_BUSY"

            with m.State("SPI_READ_BUSY"):
                m.d.comb += [
                    self.spi_addr.eq(spi_addr_reg),
                    self.spi_rw  .eq(1),
                ]
                # Use ~busy (level) instead of done (pulse) for reliable
                # CDC between sync (130 MHz) and usb (60 MHz) domains.
                # rdata is valid as it was set in the SPI master's DONE
                # state one sync cycle before busy goes low.
                with m.If(~self.spi_busy):
                    m.d.usb += spi_rdata_reg.eq(self.spi_rdata)
                    m.next = "SPI_READ_SEND_WAIT"

            with m.State("SPI_READ_SEND_WAIT"):
                with m.If(interface.data_requested):
                    m.next = "SPI_READ_SEND"

            with m.State("SPI_READ_SEND"):
                m.d.comb += [
                    interface.tx.payload.eq(spi_rdata_reg),
                    interface.tx.valid  .eq(1),
                    interface.tx.first  .eq(1),
                    interface.tx.last   .eq(1),
                ]
                with m.If(interface.tx.ready):
                    m.next = "STATUS_IN_ACK"

            # ----------------------------------------------------------------
            # GET_STATUS (0x03): returns 6 bytes
            #   byte 0: firmware version major
            #   byte 1: firmware version minor
            #   byte 2: SPI busy flag
            #   byte 3: LED state
            #   byte 4: RX FIFO status (bit0=rx09_overflow, bit1=rx24_overflow,
            #            bit2=rx09_has_data, bit3=rx24_has_data)
            #   byte 5: TX FIFO status (bit0=tx09_underflow, bit1=tx24_underflow)
            # ----------------------------------------------------------------
            with m.State("GET_STATUS_WAIT"):
                with m.If(interface.data_requested):
                    m.d.usb += byte_idx.eq(0)
                    m.next = "GET_STATUS_SEND"

            with m.State("GET_STATUS_SEND"):
                status_byte = Signal(8)
                with m.Switch(byte_idx):
                    with m.Case(0):
                        m.d.comb += status_byte.eq(FW_VERSION_MAJOR)
                    with m.Case(1):
                        m.d.comb += status_byte.eq(FW_VERSION_MINOR)
                    with m.Case(2):
                        m.d.comb += status_byte.eq(self.spi_busy)
                    with m.Case(3):
                        m.d.comb += status_byte.eq(
                            Cat(self.led_r, self.led_g, self.led_b))
                    with m.Case(4):
                        m.d.comb += status_byte.eq(
                            Cat(self.dbg_rx09_overflow, self.dbg_rx24_overflow,
                                self.dbg_rx09_has_data, self.dbg_rx24_has_data))
                    with m.Case(5):
                        m.d.comb += status_byte.eq(
                            Cat(self.dbg_tx09_underflow, self.dbg_tx24_underflow))

                m.d.comb += [
                    interface.tx.payload.eq(status_byte),
                    interface.tx.valid  .eq(1),
                    interface.tx.first  .eq(byte_idx == 0),
                    interface.tx.last   .eq(byte_idx == 5),
                ]
                with m.If(interface.tx.ready):
                    with m.If(byte_idx == 5):
                        m.next = "STATUS_IN_ACK"
                    with m.Else():
                        m.d.usb += byte_idx.eq(byte_idx + 1)

            # ----------------------------------------------------------------
            # SET_LED (0x04): wValue[2:0] = {b, g, r}
            # OUT request with no data phase
            # ----------------------------------------------------------------
            with m.State("SET_LED"):
                m.d.usb += [
                    self.led_r  .eq(setup.value[0]),
                    self.led_g  .eq(setup.value[1]),
                    self.led_b  .eq(setup.value[2]),
                    self.led_set.eq(1),
                ]
                m.next = "STATUS_OUT_ZLP"

            # ----------------------------------------------------------------
            # RESET_AT86 (0x05): triggers AT86RF215 reset
            # OUT request with no data phase
            # ----------------------------------------------------------------
            with m.State("RESET_AT86"):
                m.d.usb += reset_cnt.eq(0xFF)
                m.next = "STATUS_OUT_ZLP"

            # ----------------------------------------------------------------
            # GET_RX_DEBUG (0x06): returns 1 byte of packed debug flags
            #   bit 0: RXCLK heartbeat (toggles ~4Hz if RXCLK running)
            #   bit 1: frame_sync_09
            #   bit 2: frame_sync_24
            #   bit 3: fifo_24_has_data
            # ----------------------------------------------------------------
            with m.State("RX_DEBUG_WAIT"):
                with m.If(interface.data_requested):
                    m.next = "RX_DEBUG_SEND"

            with m.State("RX_DEBUG_SEND"):
                m.d.comb += [
                    interface.tx.payload.eq(Cat(
                        self.dbg_rxclk_active,
                        self.dbg_frame_sync_09,
                        self.dbg_frame_sync_24,
                        self.dbg_fifo_24_has_data,
                    )),
                    interface.tx.valid.eq(1),
                    interface.tx.first.eq(1),
                    interface.tx.last .eq(1),
                ]
                with m.If(interface.tx.ready):
                    m.next = "STATUS_IN_ACK"

            # ----------------------------------------------------------------
            # GET_IRQ (0x07): returns 1 byte with AT86RF215 IRQ pin state
            #   bit 0: raw IRQ pin level (active low on AT86RF215)
            # ----------------------------------------------------------------
            with m.State("IRQ_WAIT"):
                with m.If(interface.data_requested):
                    m.next = "IRQ_SEND"

            with m.State("IRQ_SEND"):
                m.d.comb += [
                    interface.tx.payload.eq(self.dbg_irq_pin),
                    interface.tx.valid.eq(1),
                    interface.tx.first.eq(1),
                    interface.tx.last .eq(1),
                ]
                with m.If(interface.tx.ready):
                    m.next = "STATUS_IN_ACK"

            # ----------------------------------------------------------------
            # Shared status-phase states
            # ----------------------------------------------------------------

            # STATUS_OUT_ZLP: OUT requests (wLength=0) need a ZLP IN status
            with m.State("STATUS_OUT_ZLP"):
                with m.If(interface.status_requested):
                    m.d.comb += self.send_zlp()
                    m.next = "IDLE"

            # STATUS_IN_ACK: IN requests need an ACK on the OUT status phase
            with m.State("STATUS_IN_ACK"):
                with m.If(interface.status_requested):
                    m.d.comb += interface.handshakes_out.ack.eq(1)
                    m.next = "IDLE"

            # ----------------------------------------------------------------
            # UNHANDLED -- stall unknown vendor requests
            # ----------------------------------------------------------------
            with m.State("UNHANDLED"):
                with m.If(interface.data_requested | interface.status_requested):
                    m.d.comb += interface.handshakes_out.stall.eq(1)
                    m.next = "IDLE"

        return m
