#include "USBDevice.h"

namespace 
{
    uint8_t EP0RxBuffer[64];
}

USBDevice::USBDevice(F0USB &usb)
: mUSB{usb}
{
    usb.RegisterEndpoint(0, EPType::Control, 64, 64, EP0RxBuffer);
    usb.RegisterDevice(this);
}

void USBDevice::SetupPacketRx(const SetupPacket &setupPacket)
{
    switch(setupPacket.bRequest)
    {
        case StandardRequestCodes::GET_STATUS:
			break;
		case StandardRequestCodes::CLEAR_FEATURE:
			break;
		case StandardRequestCodes::SET_FEATURE:
			break;
		case StandardRequestCodes::SET_ADDRESS:
			break;
		case StandardRequestCodes::GET_DESCRIPTOR:
			{
			uint16_t descriptorLength;
			uint8_t *deviceDesc = GetDeviceDescriptor(&descriptorLength);
			mUSB.TxData(0, deviceDesc, descriptorLength);
			}				
			break;
		case StandardRequestCodes::SET_DESCRIPTOR:
			break;
		case StandardRequestCodes::GET_CONFIGURATION:
			break;
		case StandardRequestCodes::SET_CONFIGURATION:
			break;
		case StandardRequestCodes::GET_INTERFACE:
			break;
		case StandardRequestCodes::SET_INTERFACE:
			break;
		case StandardRequestCodes::SYNCH_FRAME:
			break;
		default:
			break;
	}
	asm("isb");

}

void USBDevice::EP0In()
{

}

void USBDevice::EP0Out(uint16_t byteCount)
{
	SetupPacket packet;
	packet.bmRequestType = EP0RxBuffer[0];
	packet.bRequest = EP0RxBuffer[1];
	packet.wValue = static_cast<uint16_t>(EP0RxBuffer[2]) | (static_cast<uint16_t>(EP0RxBuffer[3]) << 8);
	packet.wIndex = static_cast<uint16_t>(EP0RxBuffer[4]) | (static_cast<uint16_t>(EP0RxBuffer[5]) << 8);
	packet.wLength = static_cast<uint16_t>(EP0RxBuffer[6]) | (static_cast<uint16_t>(EP0RxBuffer[7]) << 8);
	SetupPacketRx(packet);
	
}