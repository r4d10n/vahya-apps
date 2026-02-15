"""Clock domain crossing FIFO bridges for I/Q sample streaming."""

from amaranth import *
from amaranth.lib.fifo import AsyncFIFO


class RxFIFOBridge(Elaboratable):
    """Bridges I/Q samples from at86_rx domain to USB byte stream.

    Write side (at86_rx domain): 32-bit words (I[15:0] | Q[15:0])
    Read side (usb domain): 8-bit bytes serialized little-endian
    """

    DEPTH = 2048

    def __init__(self):
        # Write interface (at86_rx domain)
        self.w_i_sample = Signal(16)
        self.w_q_sample = Signal(16)
        self.w_valid    = Signal()
        self.w_ready    = Signal()

        # Read interface (usb domain) — byte stream for USB IN endpoint
        self.r_data     = Signal(8)
        self.r_valid    = Signal()
        self.r_ready    = Signal()

        # Status
        self.overflow   = Signal()
        self.underflow  = Signal()

    def elaborate(self, platform):
        m = Module()

        m.submodules.fifo = fifo = AsyncFIFO(
            width=32, depth=self.DEPTH,
            w_domain="at86_rx", r_domain="usb")

        # --- Write side: pack I+Q into 32 bits ---
        m.d.comb += [
            fifo.w_data.eq(Cat(self.w_i_sample, self.w_q_sample)),
            fifo.w_en.eq(self.w_valid & fifo.w_rdy),
            self.w_ready.eq(fifo.w_rdy),
        ]
        # Overflow: tried to write when full
        with m.If(self.w_valid & ~fifo.w_rdy):
            m.d.at86_rx += self.overflow.eq(1)

        # --- Read side: serialize 32-bit word to 4 bytes ---
        word_buf  = Signal(32)
        byte_idx  = Signal(2)

        with m.FSM(domain="usb"):
            with m.State("FETCH"):
                # Try to read a word from FIFO
                with m.If(fifo.r_rdy):
                    m.d.comb += fifo.r_en.eq(1)
                    m.d.usb += [
                        word_buf.eq(fifo.r_data),
                        byte_idx.eq(0),
                    ]
                    m.next = "SEND"

            with m.State("SEND"):
                m.d.comb += self.r_valid.eq(1)
                with m.Switch(byte_idx):
                    with m.Case(0): m.d.comb += self.r_data.eq(word_buf[0:8])
                    with m.Case(1): m.d.comb += self.r_data.eq(word_buf[8:16])
                    with m.Case(2): m.d.comb += self.r_data.eq(word_buf[16:24])
                    with m.Case(3): m.d.comb += self.r_data.eq(word_buf[24:32])

                with m.If(self.r_ready):
                    with m.If(byte_idx == 3):
                        m.next = "FETCH"
                    with m.Else():
                        m.d.usb += byte_idx.eq(byte_idx + 1)

        return m


class TxFIFOBridge(Elaboratable):
    """Bridges USB byte stream to I/Q samples in at86_tx domain.

    Write side (usb domain): 8-bit bytes assembled little-endian
    Read side (at86_tx domain): 32-bit words (I[15:0] | Q[15:0])
    """

    DEPTH = 4096

    def __init__(self):
        # Write interface (usb domain) — byte stream from USB OUT endpoint
        self.w_data     = Signal(8)
        self.w_valid    = Signal()
        self.w_ready    = Signal()

        # Read interface (at86_tx domain)
        self.r_i_sample = Signal(16)
        self.r_q_sample = Signal(16)
        self.r_valid    = Signal()
        self.r_ready    = Signal()

        # Status
        self.overflow   = Signal()
        self.underflow  = Signal()

    def elaborate(self, platform):
        m = Module()

        m.submodules.fifo = fifo = AsyncFIFO(
            width=32, depth=self.DEPTH,
            w_domain="usb", r_domain="at86_tx")

        # --- Write side: assemble 4 bytes into 32-bit word ---
        word_buf = Signal(32)
        byte_idx = Signal(2)

        # Backpressure: accept bytes 0-2 freely (just filling word_buf).
        # On byte 3, only accept if FIFO has space.  This gates the USB
        # endpoint's stream.ready, causing NAK on OUT tokens when full.
        # Without this, USB pushes at ~334 Mbps while the radio drains at
        # 128 Mbps, overflowing the FIFO and corrupting the IQ stream.
        m.d.comb += self.w_ready.eq((byte_idx != 3) | fifo.w_rdy)

        with m.If(self.w_valid & self.w_ready):
            with m.Switch(byte_idx):
                with m.Case(0): m.d.usb += word_buf[0:8].eq(self.w_data)
                with m.Case(1): m.d.usb += word_buf[8:16].eq(self.w_data)
                with m.Case(2): m.d.usb += word_buf[16:24].eq(self.w_data)
                with m.Case(3):
                    # Last byte — write complete word to FIFO
                    m.d.comb += [
                        fifo.w_data.eq(Cat(word_buf[0:24], self.w_data)),
                        fifo.w_en.eq(1),
                    ]

            m.d.usb += byte_idx.eq(byte_idx + 1)

        # --- Read side: split 32-bit word to I + Q ---
        m.d.comb += [
            self.r_i_sample.eq(fifo.r_data[0:16]),
            self.r_q_sample.eq(fifo.r_data[16:32]),
            self.r_valid.eq(fifo.r_rdy),
            fifo.r_en.eq(self.r_ready & fifo.r_rdy),
        ]

        with m.If(self.r_ready & ~fifo.r_rdy):
            m.d.at86_tx += self.underflow.eq(1)

        return m
