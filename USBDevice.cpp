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
	mLastRxSetupPacket = setupPacket;
    switch(setupPacket.bRequest)
    {
        case StandardRequestCodes::GET_STATUS:
			break;
		case StandardRequestCodes::CLEAR_FEATURE:
			break;
		case StandardRequestCodes::SET_FEATURE:
			break;
		case StandardRequestCodes::SET_ADDRESS:
			mUSB.TxData(0, nullptr, 0);
		
			break;
		case StandardRequestCodes::GET_DESCRIPTOR:
			if(((setupPacket.wValue & 0xFF00) >> 8U) == DescriptorTypes::DEVICE)
			{
				uint16_t descriptorLength;
				const uint8_t *deviceDesc = GetDeviceDescriptor(&descriptorLength);
				mUSB.TxData(0, deviceDesc, descriptorLength);
			}
			else if(((setupPacket.wValue & 0xFF00) >> 8U) == DescriptorTypes::CONFIGURATION)
			{
				mEP0InTransaction.mData = GetConfigurationDescriptor(&mEP0InTransaction.mLength);
				if(mEP0InTransaction.mLength > setupPacket.wLength)
				{
					mEP0InTransaction.mLength = setupPacket.wLength;
				}
				if(mEP0InTransaction.mLength > 64U)
				{
					mUSB.TxData(0, mEP0InTransaction.mData, 64U);
					mEP0InTransaction.mPosition = 64U;
				} else {
					mUSB.TxData(0, mEP0InTransaction.mData, mEP0InTransaction.mLength);
					mEP0InTransaction.mPosition = mEP0InTransaction.mLength;
				}
			}
			else if(((setupPacket.wValue & 0xFF00) >> 8U) == DescriptorTypes::STRING)
			{
				mEP0InTransaction.mData = GetStringDescriptor(setupPacket.wValue & 0xFF, setupPacket.wIndex, &mEP0InTransaction.mLength);
				if(mEP0InTransaction.mLength > setupPacket.wLength)
				{
					mEP0InTransaction.mLength = setupPacket.wLength;
				}
				if(mEP0InTransaction.mLength > 64U)
				{
					mUSB.TxData(0, mEP0InTransaction.mData, 64U);
					mEP0InTransaction.mPosition = 64U;
				} else {
					mUSB.TxData(0, mEP0InTransaction.mData, mEP0InTransaction.mLength);
					mEP0InTransaction.mPosition = mEP0InTransaction.mLength;
				}
			}
			break;
		case StandardRequestCodes::SET_DESCRIPTOR:
			break;
		case StandardRequestCodes::GET_CONFIGURATION:
			break;
		case StandardRequestCodes::SET_CONFIGURATION:
			if(SetConfiguration(setupPacket.wValue))
			{
				mUSB.TxData(0, nullptr, 0);
			}
			else
			{
				//todo stall
			}
			break;
		case StandardRequestCodes::GET_INTERFACE:
			break;
		case StandardRequestCodes::SET_INTERFACE:
			if(SetInterface(setupPacket.wIndex, setupPacket.wValue))
			{
				mUSB.TxData(0, nullptr, 0);
			}
			else
			{
				
			}
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
	if(StandardRequestCodes::SET_ADDRESS == mLastRxSetupPacket.bRequest)
	{
		mUSB.SetDeviceAddress(static_cast<uint8_t>(mLastRxSetupPacket.wValue));
	}
	
	if(mEP0InTransaction.mPosition != mEP0InTransaction.mLength)
	{ //more to send
		if((mEP0InTransaction.mLength - mEP0InTransaction.mPosition) > 64U)
		{
			mUSB.TxData(0, &mEP0InTransaction.mData[mEP0InTransaction.mPosition], 64U);
			mEP0InTransaction.mPosition += 64U;
		} else {
			uint32_t thisTxLength = mEP0InTransaction.mLength - mEP0InTransaction.mPosition;
			mUSB.TxData(0, &mEP0InTransaction.mData[mEP0InTransaction.mPosition], thisTxLength);
			mEP0InTransaction.mPosition += thisTxLength;
		}
	}
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