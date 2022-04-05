#ifndef usbdevice_h_
#define usbdevice_h_

#include <cstdint>
#include "SetupPacket.h"

class USBDevice
{
public:
    virtual ~USBDevice(){}

    virtual void Init() = 0;

    //ep0 
    virtual void SetupPacketRx(const SetupPacket *setupPacket) = 0;
    virtual void EP0In() = 0;
    virtual void EP0Out() = 0;

    // device specific
    virtual void DataIn(uint8_t endpoint) = 0;
    virtual void DataOut(uint8_t endpoint) = 0;
    
    virtual void SOF() = 0;

    //getters
    virtual uint8_t *GetDeviceDescriptor(uint16_t *length) = 0;
    virtual uint8_t *GetConfigurationDescriptor(uint16_t *length) = 0;
};

#endif
