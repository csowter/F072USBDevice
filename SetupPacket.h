#ifndef setuppacket_h_
#define setuppacket_h_

#include <stdint.h>

namespace StandardRequestCodes
{
    constexpr uint8_t GET_STATUS = 0U;
    constexpr uint8_t CLEAR_FEATURE = 1U;
    constexpr uint8_t SET_FEATURE = 3U;
    constexpr uint8_t SET_ADDRESS = 5U;
    constexpr uint8_t GET_DESCRIPTOR = 6U;
    constexpr uint8_t SET_DESCRIPTOR = 7U;
    constexpr uint8_t GET_CONFIGURATION = 8U;
    constexpr uint8_t SET_CONFIGURATION = 9U;
    constexpr uint8_t GET_INTERFACE = 10U;
    constexpr uint8_t SET_INTERFACE = 11U;
    constexpr uint8_t SYNCH_FRAME = 12U;
}

namespace DescriptorTypes
{
    constexpr uint8_t DEVICE = 1U;
    constexpr uint8_t CONFIGURATION = 2U;
    constexpr uint8_t STRING = 3U;
    constexpr uint8_t INTERFACE = 4U;
    constexpr uint8_t ENDPOINT = 5U;
    constexpr uint8_t DEVICE_QUALIFIER = 6U;
    constexpr uint8_t OTHER_SPEED_CONFIGURATION = 7U;
    constexpr uint8_t INTERFACE_POWER = 8U;
}

namespace RequestType
{
    enum class DataTransferDirection
    {
        HostToDevice,
        DeviceToHost,
    };

    enum class Type
    {
        Standard,
        Class,
        Vendor,
    };

    enum class Recipient
    {
        Device,
        Interface,
        Endpoint,
        Other,
    };
}

class SetupPacket
{
public:
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

#endif