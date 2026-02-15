"""USB device descriptors for Vahya radio gateware."""

from usb_protocol.emitters import DeviceDescriptorCollection

VENDOR_ID  = 0x1209
PRODUCT_ID = 0x0001

EP_RX_900  = 1   # IN  0x81
EP_RX_2400 = 2   # IN  0x82
EP_TX_900  = 3   # OUT 0x03
EP_TX_2400 = 4   # OUT 0x04

MAX_PACKET_SIZE = 512  # USB 2.0 HS


def create_descriptors():
    """Build USB descriptors for the radio device."""
    descriptors = DeviceDescriptorCollection()

    with descriptors.DeviceDescriptor() as d:
        d.idVendor           = VENDOR_ID
        d.idProduct          = PRODUCT_ID
        d.iManufacturer      = "Vahya"
        d.iProduct           = "AT86RF215 Radio"
        d.iSerialNumber      = "0001"
        d.bNumConfigurations = 1

    with descriptors.ConfigurationDescriptor() as c:
        c.bmAttributes = 0x80
        c.bMaxPower    = 250

        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber = 0
            i.bInterfaceClass  = 0xFF

            with i.EndpointDescriptor() as e:
                e.bEndpointAddress = 0x80 | EP_RX_900
                e.wMaxPacketSize   = MAX_PACKET_SIZE

            with i.EndpointDescriptor() as e:
                e.bEndpointAddress = 0x80 | EP_RX_2400
                e.wMaxPacketSize   = MAX_PACKET_SIZE

            with i.EndpointDescriptor() as e:
                e.bEndpointAddress = EP_TX_900
                e.wMaxPacketSize   = MAX_PACKET_SIZE

            with i.EndpointDescriptor() as e:
                e.bEndpointAddress = EP_TX_2400
                e.wMaxPacketSize   = MAX_PACKET_SIZE

    return descriptors
