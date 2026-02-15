#pragma once
#include "itransport.h"
#include "../util/ring_buffer.h"
#include <libusb.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>

class UsbTransport : public ITransport {
public:
    UsbTransport();
    ~UsbTransport() override;

    // Initialize with Android USB file descriptor
    int initWithFd(int fd);
    void shutdown();

    // ITransport interface
    int spiWrite(uint16_t addr, uint8_t value) override;
    int spiRead(uint16_t addr, uint8_t *value) override;
    int getStatus(VahyaStatus *status) override;
    int resetRadio() override;
    int setLed(uint8_t r, uint8_t g, uint8_t b) override;

    int startRx(Band band) override;
    int stopRx(Band band) override;
    int startTx(Band band) override;
    int stopTx(Band band) override;

    size_t readIQ(Band band, int16_t *buf, size_t maxPairs) override;
    size_t writeIQ(Band band, const int16_t *buf, size_t numPairs) override;

    int initRx(Band band, uint32_t freqHz) override;
    int initTx(Band band, uint32_t freqHz, uint8_t power) override;

private:
    libusb_context *ctx_;
    libusb_device_handle *handle_;
    std::mutex controlMutex_;  // serialize EP0 access

    // Transfer sizes and queue depths
    static constexpr int RX_TRANSFER_SIZE = 16384;
    static constexpr int RX_QUEUE_DEPTH = 16;
    static constexpr int TX_TRANSFER_SIZE = 16384;
    static constexpr int TX_QUEUE_DEPTH = 4;
    static constexpr size_t RING_SIZE = 131072;

    // EP0 vendor request codes
    static constexpr uint8_t REQ_SPI_WRITE  = 0x01;
    static constexpr uint8_t REQ_SPI_READ   = 0x02;
    static constexpr uint8_t REQ_GET_STATUS = 0x03;
    static constexpr uint8_t REQ_SET_LED    = 0x04;
    static constexpr uint8_t REQ_RESET_AT86 = 0x05;

    // Callback context passed via xfer->user_data
    struct TransferContext {
        UsbTransport *transport;
        int bandIdx;
    };

    struct Stream {
        std::atomic<bool> active{false};
        std::vector<libusb_transfer*> transfers;
        std::vector<uint8_t*> buffers;
        std::vector<TransferContext*> contexts;
        uint8_t endpoint;
    };

    Stream rxStream_[2];  // [BAND_900, BAND_2400]
    Stream txStream_[2];
    RingBuffer<int16_t> rxRing_[2];
    RingBuffer<int16_t> txRing_[2];

    // Event handling thread
    std::thread eventThread_;
    std::atomic<bool> eventRunning_;
    void eventLoop();

    // Bulk transfer callbacks
    static void LIBUSB_CALL rxCallback(libusb_transfer *xfer);
    static void LIBUSB_CALL txCallback(libusb_transfer *xfer);

    // Stream cleanup helper
    void freeStream(Stream &stream);

    // Endpoint mapping
    uint8_t rxEndpoint(Band band) const;
    uint8_t txEndpoint(Band band) const;
    uint16_t regBase(Band band) const;

    // Radio init helpers
    int cmdTrxoff(Band band);
    int cmdTxprep(Band band);
    int cmdRx(Band band);
    int cmdTx(Band band);
    int setFrequency(Band band, uint32_t freqHz);
    int waitState(Band band, uint8_t target, int maxAttempts, int delayUs);
};
