#include "usb_transport.h"
#include <android/log.h>
#include <chrono>
#include <thread>
#include <cstring>

#define TAG "UsbTransport"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, TAG, __VA_ARGS__)

// ── Constructor / Destructor ────────────────────────────────────────────────

UsbTransport::UsbTransport()
    : ctx_(nullptr)
    , handle_(nullptr)
    , rxRing_{RingBuffer<int16_t>(RING_SIZE), RingBuffer<int16_t>(RING_SIZE)}
    , txRing_{RingBuffer<int16_t>(RING_SIZE), RingBuffer<int16_t>(RING_SIZE)}
    , eventRunning_(false)
{}

UsbTransport::~UsbTransport() {
    shutdown();
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

int UsbTransport::initWithFd(int fd) {
    // Disable device enumeration — Android SELinux blocks /dev/bus/usb access.
    // We don't need it; the fd from UsbManager is sufficient.
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY);

    int rc = libusb_init(&ctx_);
    if (rc != 0) {
        LOGE("libusb_init failed: %s", libusb_strerror((libusb_error)rc));
        return rc;
    }

    // Wrap the Android-provided file descriptor
    rc = libusb_wrap_sys_device(ctx_, (intptr_t)fd, &handle_);
    if (rc != 0) {
        LOGE("libusb_wrap_sys_device failed: %s", libusb_strerror((libusb_error)rc));
        libusb_exit(ctx_);
        ctx_ = nullptr;
        return rc;
    }

    // Claim interface 0 (main radio interface)
    rc = libusb_claim_interface(handle_, 0);
    if (rc != 0) {
        LOGW("libusb_claim_interface failed: %s (non-fatal)", libusb_strerror((libusb_error)rc));
        // Non-fatal on Android -- permission may already grant exclusive access
    }

    // Start libusb event handling thread
    eventRunning_.store(true);
    eventThread_ = std::thread(&UsbTransport::eventLoop, this);

    LOGI("UsbTransport initialized (fd=%d)", fd);

    // Verify SPI path by reading AT86RF215 part number
    uint8_t partNum = 0;
    spiRead(0x000D, &partNum);
    LOGI("AT86RF215 part=0x%02X (%s)", partNum,
         partNum == 0x35 ? "IQ" : partNum == 0x34 ? "RF215" : "unknown");

    return 0;
}

void UsbTransport::shutdown() {
    // Stop all streams
    for (int i = 0; i < 2; i++) {
        if (rxStream_[i].active.load()) {
            stopRx(static_cast<Band>(i));
        }
        if (txStream_[i].active.load()) {
            stopTx(static_cast<Band>(i));
        }
    }

    // Stop event thread
    eventRunning_.store(false);
    if (eventThread_.joinable()) {
        eventThread_.join();
    }

    // Release USB resources
    if (handle_) {
        libusb_release_interface(handle_, 0);
        libusb_close(handle_);
        handle_ = nullptr;
    }
    if (ctx_) {
        libusb_exit(ctx_);
        ctx_ = nullptr;
    }

    LOGI("UsbTransport shut down");
}

// ── Event loop ──────────────────────────────────────────────────────────────

void UsbTransport::eventLoop() {
    LOGI("Event loop started");
    while (eventRunning_.load(std::memory_order_relaxed)) {
        struct timeval tv = {0, 100000};  // 100ms timeout
        libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
    }
    LOGI("Event loop exited");
}

// ── EP0 Vendor Requests ─────────────────────────────────────────────────────

int UsbTransport::spiWrite(uint16_t addr, uint8_t value) {
    std::lock_guard<std::mutex> lock(controlMutex_);
    int rc = libusb_control_transfer(handle_,
        0x40,           // VENDOR | HOST_TO_DEVICE
        REQ_SPI_WRITE,  // bRequest
        value,          // wValue = data byte
        addr,           // wIndex = register address
        nullptr, 0,     // no data phase
        1000);          // timeout ms
    if (rc < 0) {
        LOGE("spiWrite(0x%04X, 0x%02X) failed: %s", addr, value,
             libusb_strerror((libusb_error)rc));
    }
    return (rc >= 0) ? 0 : rc;
}

int UsbTransport::spiRead(uint16_t addr, uint8_t *value) {
    std::lock_guard<std::mutex> lock(controlMutex_);
    int rc = libusb_control_transfer(handle_,
        0xC0,           // VENDOR | DEVICE_TO_HOST
        REQ_SPI_READ,   // bRequest
        0,              // wValue
        addr,           // wIndex = register address
        value, 1,       // 1 byte response
        1000);
    if (rc < 0) {
        LOGE("spiRead(0x%04X) failed: %s", addr,
             libusb_strerror((libusb_error)rc));
        return rc;
    }
    return 0;
}

int UsbTransport::getStatus(VahyaStatus *status) {
    std::lock_guard<std::mutex> lock(controlMutex_);
    uint8_t buf[6];
    int rc = libusb_control_transfer(handle_,
        0xC0,            // VENDOR | DEVICE_TO_HOST
        REQ_GET_STATUS,  // bRequest
        0, 0,            // wValue, wIndex
        buf, 6,          // 6 bytes
        1000);
    if (rc < 0) {
        LOGE("getStatus failed: %s", libusb_strerror((libusb_error)rc));
        return rc;
    }
    status->fw_major       = buf[0];
    status->fw_minor       = buf[1];
    status->spi_busy       = buf[2];
    status->led_state      = buf[3];
    status->rx_fifo_status = buf[4];
    status->tx_fifo_status = buf[5];
    return 0;
}

int UsbTransport::resetRadio() {
    std::lock_guard<std::mutex> lock(controlMutex_);
    int rc = libusb_control_transfer(handle_,
        0x40,            // VENDOR | HOST_TO_DEVICE
        REQ_RESET_AT86,  // bRequest
        0, 0,            // wValue, wIndex
        nullptr, 0,
        1000);
    if (rc < 0) {
        LOGE("resetRadio failed: %s", libusb_strerror((libusb_error)rc));
    }
    return (rc >= 0) ? 0 : rc;
}

int UsbTransport::setLed(uint8_t r, uint8_t g, uint8_t b) {
    std::lock_guard<std::mutex> lock(controlMutex_);
    uint16_t rgb = ((r ? 1 : 0) << 2) | ((g ? 1 : 0) << 1) | (b ? 1 : 0);
    int rc = libusb_control_transfer(handle_,
        0x40,          // VENDOR | HOST_TO_DEVICE
        REQ_SET_LED,   // bRequest
        rgb, 0,        // wValue = RGB bits, wIndex = 0
        nullptr, 0,
        1000);
    if (rc < 0) {
        LOGE("setLed failed: %s", libusb_strerror((libusb_error)rc));
    }
    return (rc >= 0) ? 0 : rc;
}

// ── Endpoint Mapping ────────────────────────────────────────────────────────

uint8_t UsbTransport::rxEndpoint(Band band) const {
    return (band == Band::BAND_900) ? 0x81 : 0x82;
}

uint8_t UsbTransport::txEndpoint(Band band) const {
    return (band == Band::BAND_900) ? 0x03 : 0x04;
}

uint16_t UsbTransport::regBase(Band band) const {
    return (band == Band::BAND_900) ? 0x0100 : 0x0200;
}

// ── RX Streaming ────────────────────────────────────────────────────────────

int UsbTransport::startRx(Band band) {
    int idx = static_cast<int>(band);
    auto &stream = rxStream_[idx];
    if (stream.active.load()) return 0;

    // Reset ring buffer
    rxRing_[idx].reset();

    // Allocate transfers
    stream.endpoint = rxEndpoint(band);
    stream.buffers.resize(RX_QUEUE_DEPTH);
    stream.transfers.resize(RX_QUEUE_DEPTH);
    stream.contexts.resize(RX_QUEUE_DEPTH);

    for (int i = 0; i < RX_QUEUE_DEPTH; i++) {
        stream.buffers[i] = new uint8_t[RX_TRANSFER_SIZE];
        stream.transfers[i] = libusb_alloc_transfer(0);
        stream.contexts[i] = new TransferContext{this, idx};

        libusb_fill_bulk_transfer(
            stream.transfers[i],
            handle_,
            stream.endpoint,
            stream.buffers[i],
            RX_TRANSFER_SIZE,
            rxCallback,
            stream.contexts[i],
            1000);  // timeout ms
    }

    // Set active BEFORE submitting (so callbacks can resubmit)
    stream.active.store(true);

    for (int i = 0; i < RX_QUEUE_DEPTH; i++) {
        int rc = libusb_submit_transfer(stream.transfers[i]);
        if (rc != 0) {
            LOGE("startRx: submit transfer %d failed: %s", i,
                 libusb_strerror((libusb_error)rc));
        }
    }

    LOGI("RX started (band %d, EP 0x%02X, queue %d)", idx, stream.endpoint, RX_QUEUE_DEPTH);
    return 0;
}

int UsbTransport::stopRx(Band band) {
    int idx = static_cast<int>(band);
    auto &stream = rxStream_[idx];
    if (!stream.active.load()) return 0;

    // Signal callbacks to stop resubmitting
    stream.active.store(false);

    // Cancel all pending transfers
    for (auto *xfer : stream.transfers) {
        if (xfer) libusb_cancel_transfer(xfer);
    }

    // Wait for callbacks to drain
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Free resources
    freeStream(stream);

    LOGI("RX stopped (band %d)", idx);
    return 0;
}

void UsbTransport::freeStream(Stream &stream) {
    for (size_t i = 0; i < stream.transfers.size(); i++) {
        if (stream.transfers[i]) {
            libusb_free_transfer(stream.transfers[i]);
        }
        delete[] stream.buffers[i];
        delete stream.contexts[i];
    }
    stream.transfers.clear();
    stream.buffers.clear();
    stream.contexts.clear();
}

void LIBUSB_CALL UsbTransport::rxCallback(libusb_transfer *xfer) {
    auto *ctx = static_cast<TransferContext*>(xfer->user_data);
    UsbTransport *self = ctx->transport;
    int idx = ctx->bandIdx;

    if (xfer->status == LIBUSB_TRANSFER_COMPLETED && xfer->actual_length > 0) {
        // Copy received IQ data into ring buffer
        // Data is interleaved int16_t: [I0, Q0, I1, Q1, ...]
        auto *iq = reinterpret_cast<int16_t*>(xfer->buffer);
        size_t numSamples = xfer->actual_length / sizeof(int16_t);
        self->rxRing_[idx].write(iq, numSamples);
    } else if (xfer->status != LIBUSB_TRANSFER_CANCELLED) {
        __android_log_print(ANDROID_LOG_WARN, TAG,
            "RX transfer status=%d band=%d", xfer->status, idx);
    }

    // Resubmit if stream is still active
    if (self->rxStream_[idx].active.load(std::memory_order_relaxed)) {
        int rc = libusb_submit_transfer(xfer);
        if (rc != 0) {
            __android_log_print(ANDROID_LOG_ERROR, TAG,
                "RX resubmit failed: %s", libusb_strerror((libusb_error)rc));
        }
    }
}

size_t UsbTransport::readIQ(Band band, int16_t *buf, size_t maxPairs) {
    int idx = static_cast<int>(band);
    // Each pair = 2 int16_t values (I, Q)
    size_t samplesRead = rxRing_[idx].read(buf, maxPairs * 2);
    return samplesRead / 2;
}

// ── TX Streaming ────────────────────────────────────────────────────────────

int UsbTransport::startTx(Band band) {
    int idx = static_cast<int>(band);
    auto &stream = txStream_[idx];
    if (stream.active.load()) return 0;

    // Reset TX ring buffer
    txRing_[idx].reset();

    // Allocate transfers
    stream.endpoint = txEndpoint(band);
    stream.buffers.resize(TX_QUEUE_DEPTH);
    stream.transfers.resize(TX_QUEUE_DEPTH);
    stream.contexts.resize(TX_QUEUE_DEPTH);

    for (int i = 0; i < TX_QUEUE_DEPTH; i++) {
        stream.buffers[i] = new uint8_t[TX_TRANSFER_SIZE];
        stream.transfers[i] = libusb_alloc_transfer(0);
        stream.contexts[i] = new TransferContext{this, idx};

        libusb_fill_bulk_transfer(
            stream.transfers[i],
            handle_,
            stream.endpoint,
            stream.buffers[i],
            TX_TRANSFER_SIZE,
            txCallback,
            stream.contexts[i],
            1000);
    }

    // Set active BEFORE submitting
    stream.active.store(true);

    // Submit initial transfers (filled with silence if ring is empty)
    for (int i = 0; i < TX_QUEUE_DEPTH; i++) {
        // Try to fill from ring buffer, zero-fill if not enough data
        auto *iq = reinterpret_cast<int16_t*>(stream.buffers[i]);
        size_t numSamples = TX_TRANSFER_SIZE / sizeof(int16_t);
        size_t got = txRing_[idx].read(iq, numSamples);
        if (got < numSamples) {
            std::memset(iq + got, 0, (numSamples - got) * sizeof(int16_t));
        }

        int rc = libusb_submit_transfer(stream.transfers[i]);
        if (rc != 0) {
            LOGE("startTx: submit transfer %d failed: %s", i,
                 libusb_strerror((libusb_error)rc));
        }
    }

    LOGI("TX streaming started (band %d, EP 0x%02X, queue %d)", idx, stream.endpoint, TX_QUEUE_DEPTH);

    // Wait for IQ data to reach the radio's LVDS interface, then issue CMD_TX
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    int rc = cmdTx(band);
    if (rc != 0) {
        LOGE("startTx: cmdTx failed (radio may not be in TXPREP)");
        return rc;
    }

    LOGI("TX active (band %d)", idx);
    return 0;
}

int UsbTransport::stopTx(Band band) {
    int idx = static_cast<int>(band);
    auto &stream = txStream_[idx];
    if (!stream.active.load()) return 0;

    // Put radio back to TRXOFF before stopping streaming
    cmdTrxoff(band);

    stream.active.store(false);

    for (auto *xfer : stream.transfers) {
        if (xfer) libusb_cancel_transfer(xfer);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    freeStream(stream);

    LOGI("TX stopped (band %d)", idx);
    return 0;
}

void LIBUSB_CALL UsbTransport::txCallback(libusb_transfer *xfer) {
    auto *ctx = static_cast<TransferContext*>(xfer->user_data);
    UsbTransport *self = ctx->transport;
    int idx = ctx->bandIdx;

    if (xfer->status != LIBUSB_TRANSFER_COMPLETED &&
        xfer->status != LIBUSB_TRANSFER_CANCELLED) {
        __android_log_print(ANDROID_LOG_WARN, TAG,
            "TX transfer status=%d band=%d", xfer->status, idx);
    }

    // Resubmit with new data if stream is active
    if (self->txStream_[idx].active.load(std::memory_order_relaxed)) {
        auto *iq = reinterpret_cast<int16_t*>(xfer->buffer);
        size_t numSamples = TX_TRANSFER_SIZE / sizeof(int16_t);
        size_t got = self->txRing_[idx].read(iq, numSamples);
        if (got < numSamples) {
            // Zero-fill remainder to avoid stale data
            std::memset(iq + got, 0, (numSamples - got) * sizeof(int16_t));
        }
        xfer->length = TX_TRANSFER_SIZE;

        int rc = libusb_submit_transfer(xfer);
        if (rc != 0) {
            __android_log_print(ANDROID_LOG_ERROR, TAG,
                "TX resubmit failed: %s", libusb_strerror((libusb_error)rc));
        }
    }
}

size_t UsbTransport::writeIQ(Band band, const int16_t *buf, size_t numPairs) {
    int idx = static_cast<int>(band);
    // Each pair = 2 int16_t values (I, Q)
    size_t written = txRing_[idx].write(buf, numPairs * 2);
    return written / 2;
}

// ── Radio Init: RX ──────────────────────────────────────────────────────────

int UsbTransport::initRx(Band band, uint32_t freqHz) {
    uint16_t base = regBase(band);
    int rc;

    LOGI("initRx band=%d freq=%u", static_cast<int>(band), freqHz);

    // Transition to TRXOFF FIRST — IQIFC0 DRV bits only writable in TRXOFF
    rc = cmdTrxoff(band);
    if (rc != 0) {
        LOGE("initRx: cmdTrxoff failed");
        return rc;
    }

    // Global IQ interface config (must be in TRXOFF)
    spiWrite(0x000A, 0x32);  // IQIFC0: DRV=3(4mA), CMV1V2=1(1.2V), EEC=0
    spiWrite(0x000B, 0x13);  // IQIFC1: CHPM=1(IQ mode), SKEWDRV=3(RX)

    // Band-specific registers
    spiWrite(base + 0x01, 0x0A);  // AUXS: AVEN=1, PAVC=2
    spiWrite(base + 0x00, 0x32);  // IRQM: TRXRDY+TRXERR+IQIFSF

    // Set frequency
    rc = setFrequency(band, freqHz);
    if (rc != 0) {
        LOGE("initRx: setFrequency failed");
        return rc;
    }

    // RX configuration
    spiWrite(base + 0x09, 0x1B);  // RXBWC: BW=2MHz, IFS=1
    spiWrite(base + 0x0A, 0x81);  // RXDFE: RCUT=4, SR=4MSps
    spiWrite(base + 0x0B, 0x61);  // AGCC: AGCI=1(unfiltered), AVGS=32, EN=1
    spiWrite(base + 0x0C, 0x60);  // AGCS: TGT=3(-30dBm)

    // Enter RX state
    rc = cmdRx(band);
    if (rc != 0) {
        LOGE("initRx: cmdRx failed");
        return rc;
    }

    return 0;
}

// ── Radio Init: TX ──────────────────────────────────────────────────────────

int UsbTransport::initTx(Band band, uint32_t freqHz, uint8_t power) {
    uint16_t base = regBase(band);
    int rc;

    LOGI("initTx band=%d freq=%u power=%d", static_cast<int>(band), freqHz, power);

    // Transition to TRXOFF FIRST — IQIFC0 DRV bits only writable in TRXOFF
    rc = cmdTrxoff(band);
    if (rc != 0) {
        LOGE("initTx: cmdTrxoff failed");
        return rc;
    }

    // Global IQ interface config (must be in TRXOFF)
    spiWrite(0x000A, 0x32);  // IQIFC0: DRV=3, CMV1V2=1, EEC=0
    spiWrite(0x000B, 0x12);  // IQIFC1: CHPM=1(IQ mode), SKEWDRV=1(TX)

    // Band-specific
    spiWrite(base + 0x01, 0x0A);  // AUXS: AVEN=1, PAVC=2
    spiWrite(base + 0x00, 0x32);  // IRQM: TRXRDY+TRXERR+IQIFSF

    // Set frequency
    rc = setFrequency(band, freqHz);
    if (rc != 0) {
        LOGE("initTx: setFrequency failed");
        return rc;
    }

    // TX configuration
    spiWrite(base + 0x10, 0x81);  // TXDFE: RCUT=4, SR=4MSps
    spiWrite(base + 0x12, 0x60 | (power & 0x1F));  // PAC: PA_CURR=3, TXPWR
    spiWrite(base + 0x11, 0x83);  // TXCUTC: PA ramp=16us, LPFCUT=3

    // Transition to TXPREP (radio waits for IQ data before TX)
    rc = cmdTxprep(band);
    if (rc != 0) {
        LOGE("initTx: cmdTxprep failed");
        return rc;
    }

    LOGI("initTx complete (TXPREP): band=%d freq=%u power=%d", static_cast<int>(band), freqHz, power);
    return 0;
}

// ── Radio Command Helpers ───────────────────────────────────────────────────

int UsbTransport::cmdTrxoff(Band band) {
    uint16_t base = regBase(band);

    // Errata #6: TRXOFF may need retries
    for (int attempt = 0; attempt < 5; attempt++) {
        spiWrite(base + 0x03, 0x02);  // CMD = TRXOFF

        int rc = waitState(band, 0x02, 10, 100);
        if (rc == 0) return 0;

        LOGW("cmdTrxoff: attempt %d failed, retrying...", attempt + 1);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    LOGE("cmdTrxoff: all retries exhausted");
    return -1;
}

int UsbTransport::cmdTxprep(Band band) {
    uint16_t base = regBase(band);
    spiWrite(base + 0x03, 0x03);  // CMD = TXPREP
    return waitState(band, 0x03, 20, 200);
}

int UsbTransport::cmdRx(Band band) {
    uint16_t base = regBase(band);
    spiWrite(base + 0x03, 0x05);  // CMD = RX
    return waitState(band, 0x05, 20, 200);
}

int UsbTransport::cmdTx(Band band) {
    uint16_t base = regBase(band);
    spiWrite(base + 0x03, 0x04);  // CMD = TX
    return waitState(band, 0x04, 50, 200);
}

int UsbTransport::waitState(Band band, uint8_t target, int maxAttempts, int delayUs) {
    uint16_t base = regBase(band);

    for (int i = 0; i < maxAttempts; i++) {
        uint8_t state = 0;
        spiRead(base + 0x02, &state);  // STATE register
        state &= 0x1F;  // mask to state bits

        if (state == target) return 0;

        std::this_thread::sleep_for(std::chrono::microseconds(delayUs));
    }

    uint8_t finalState = 0;
    spiRead(regBase(band) + 0x02, &finalState);
    LOGE("waitState: timeout waiting for state 0x%02X, got 0x%02X", target, finalState & 0x1F);
    return -1;
}

// ── Frequency Setting ───────────────────────────────────────────────────────

int UsbTransport::setFrequency(Band band, uint32_t freqHz) {
    uint16_t base = regBase(band);

    // Compute target register values
    uint8_t tgt_ccf0l, tgt_ccf0h, tgt_cnl, tgt_cnm;

    if (band == Band::BAND_2400) {
        // Fine Resolution Mode 3: f = 2366 MHz + (CCF0*256 + CN) * 26 MHz / 65536
        uint32_t nchannel = (uint32_t)((uint64_t)(freqHz - 2366000000ULL) * 65536 / 26000000);
        uint16_t ccf0 = (nchannel >> 8) & 0xFFFF;
        uint8_t cn = nchannel & 0xFF;
        tgt_ccf0l = ccf0 & 0xFF;
        tgt_ccf0h = (ccf0 >> 8) & 0xFF;
        tgt_cnl = cn;
        tgt_cnm = 0xC0;  // Fine Mode 3
        LOGI("setFreq: RF24 fine3 nchan=%u ccf0=0x%04x cn=0x%02x", nchannel, ccf0, cn);
    } else if (freqHz >= 377000000 && freqHz < 510000000) {
        // RF09 Fine Mode 1: f = 377 MHz + (CCF0*256 + CN) * 6.5 MHz / 65536
        uint32_t nchannel = (uint32_t)((uint64_t)(freqHz - 377000000ULL) * 65536 / 6500000);
        uint16_t ccf0 = (nchannel >> 8) & 0xFFFF;
        uint8_t cn = nchannel & 0xFF;
        tgt_ccf0l = ccf0 & 0xFF;
        tgt_ccf0h = (ccf0 >> 8) & 0xFF;
        tgt_cnl = cn;
        tgt_cnm = 0x40;  // Fine Mode 1
        uint32_t actualHz = 377000000ULL + (uint64_t)nchannel * 6500000 / 65536;
        LOGI("setFreq: RF09 fine1 nchan=%u ccf0=0x%04x cn=0x%02x actual=%u Hz", nchannel, ccf0, cn, actualHz);
    } else if (freqHz >= 754000000 && freqHz <= 1020000000) {
        // RF09 Fine Mode 2: f = 754 MHz + (CCF0*256 + CN) * 13 MHz / 65536
        uint32_t nchannel = (uint32_t)((uint64_t)(freqHz - 754000000ULL) * 65536 / 13000000);
        uint16_t ccf0 = (nchannel >> 8) & 0xFFFF;
        uint8_t cn = nchannel & 0xFF;
        tgt_ccf0l = ccf0 & 0xFF;
        tgt_ccf0h = (ccf0 >> 8) & 0xFF;
        tgt_cnl = cn;
        tgt_cnm = 0x80;  // Fine Mode 2
        uint32_t actualHz = 754000000ULL + (uint64_t)nchannel * 13000000 / 65536;
        LOGI("setFreq: RF09 fine2 nchan=%u ccf0=0x%04x cn=0x%02x actual=%u Hz", nchannel, ccf0, cn, actualHz);
    } else {
        // IEEE mode: freq = CCF0 * 25 kHz
        uint16_t ccf0 = freqHz / 25000;
        tgt_ccf0l = ccf0 & 0xFF;
        tgt_ccf0h = (ccf0 >> 8) & 0xFF;
        tgt_cnl = 0x00;
        tgt_cnm = 0x00;
        LOGI("setFreq: RF09 IEEE ccf0=%u", ccf0);
    }

    // Write-verify-retry loop for frequency registers
    for (int attempt = 0; attempt < 3; attempt++) {
        spiWrite(base + 0x04, 0x01);      // CS = 1
        spiWrite(base + 0x05, tgt_ccf0l); // CCF0L
        spiWrite(base + 0x06, tgt_ccf0h); // CCF0H
        spiWrite(base + 0x07, tgt_cnl);   // CNL
        spiWrite(base + 0x08, tgt_cnm);   // CNM (triggers PLL re-tune)

        // Verify all four registers
        uint8_t rb_ccf0l = 0, rb_ccf0h = 0, rb_cnl = 0, rb_cnm = 0;
        spiRead(base + 0x05, &rb_ccf0l);
        spiRead(base + 0x06, &rb_ccf0h);
        spiRead(base + 0x07, &rb_cnl);
        spiRead(base + 0x08, &rb_cnm);

        bool match = (rb_ccf0l == tgt_ccf0l) && (rb_ccf0h == tgt_ccf0h) &&
                     (rb_cnl == tgt_cnl) && (rb_cnm == tgt_cnm);

        LOGI("setFreq verify[%d]: wrote CCF0=%02x%02x CNL=%02x CNM=%02x, "
             "read CCF0=%02x%02x CNL=%02x CNM=%02x %s",
             attempt, tgt_ccf0h, tgt_ccf0l, tgt_cnl, tgt_cnm,
             rb_ccf0h, rb_ccf0l, rb_cnl, rb_cnm,
             match ? "OK" : "MISMATCH");

        if (match) {
            LOGI("setFrequency: band=%d freq=%u Hz (attempt %d)",
                 static_cast<int>(band), freqHz, attempt);
            return 0;
        }

        LOGW("setFreq: register mismatch, retrying...");
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    LOGE("setFrequency: FAILED after 3 attempts! band=%d freq=%u", static_cast<int>(band), freqHz);
    return -1;
}
