#ifndef usbdevice_h_
#define usbdevice_h_

#include <stdint.h>
#include "SetupPacket.h"
#include "F0USB.h"

class USBDevice
{
public:
	USBDevice() = delete;
    USBDevice(F0USB &usb);
    virtual ~USBDevice(){}

    virtual void Init() = 0;

    //ep0 
    void SetupPacketRx(const SetupPacket &setupPacket);
    void EP0In();
    void EP0Out(uint16_t byteCount);

    // device specific
    virtual void DataIn(uint8_t endpoint) = 0;
    virtual void DataOut(uint8_t endpoint, uint16_t byteCount) = 0;
    
    virtual void SOF() = 0;

    //getters
    virtual uint8_t *GetDeviceDescriptor(uint16_t *length) = 0;
    virtual uint8_t *GetConfigurationDescriptor(uint16_t *length) = 0;

private:
	F0USB &mUSB;
};

#endif
