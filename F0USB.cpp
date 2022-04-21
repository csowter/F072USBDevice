#include "F0USB.h"
#include <stdint.h>
#include <algorithm>
#include "USBDevice.h"

enum class EPTxState {
	Disabled,
	Stall,
	NAK,
	Valid
};

enum class EPRxState {
	Disabled,
	Stall,
	NAK,
	Valid
};



namespace F072 
{
  constexpr uint32_t USBBaseAddress = 0x40005C00U;
  constexpr uint32_t CommonRegistersOffset = 0x40U;
  
  __packed struct CommonRegisters
  {
	  volatile uint32_t CNTR;
	  volatile uint32_t ISTR;
	  volatile uint32_t FNR;
	  volatile uint32_t DADDR;
	  volatile uint32_t BTABLE;
	  volatile uint32_t LPMCSR;
	  volatile uint32_t BCDR;
  };
  
  __packed struct EndpointRegisters
  {
	  volatile uint32_t EPxR[8];
  };
  
  CommonRegisters volatile * const USBRegisters = reinterpret_cast<CommonRegisters *>(USBBaseAddress + CommonRegistersOffset);
  EndpointRegisters volatile * const USBEndpointRegisters = reinterpret_cast<EndpointRegisters *>(USBBaseAddress);
  
  namespace ISTR
  {
	  constexpr uint32_t CTRPos = 15U;
	  constexpr uint32_t PMAOVRPos = 14U;
	  constexpr uint32_t ERRPos = 13U;
	  constexpr uint32_t WKUPPos = 12U;
	  constexpr uint32_t SUSPPos = 11U;
	  constexpr uint32_t RESETPos = 10U;
	  constexpr uint32_t SOFPos = 9U;
	  constexpr uint32_t ESOFPos = 8U;
	  constexpr uint32_t L1REQPos = 7U;
	  constexpr uint32_t DIRPos = 4U;
	  constexpr uint32_t EP_IDPos = 0U;
	  constexpr uint32_t EP_IDMask = 0x0FU;
  }
  
  namespace Endpoint 
  {
	namespace USBEPxR
	{
		constexpr uint32_t EAPos = 0U;
		constexpr uint32_t EAMask = 0xFU;
		constexpr uint32_t STAT_TXPos = 4U;
		constexpr uint32_t STAT_TXMask = (3U << STAT_TXPos);
		constexpr uint32_t DTOG_TXPos = 6U;
		constexpr uint32_t CTR_TXPos = 7U;
		constexpr uint32_t EP_KINDPos = 8U;
		constexpr uint32_t EP_TYPEPos = 9U;
		constexpr uint32_t EP_TYPEMask = (3u << EP_TYPEPos);
		constexpr uint32_t SETUPPos = 11U;
		constexpr uint32_t STAT_RXPos = 12U;
		constexpr uint32_t STAT_RXMask = (3U << STAT_RXPos);
		constexpr uint32_t DTOG_RXPos = 14U;
		constexpr uint32_t CTR_RXPos = 15U;
	}
	  
	void SetAddress(uint8_t endpoint, uint8_t address)
	{
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		//clear toggle bits
		epVal &= ~( (1 << USBEPxR::DTOG_RXPos) |
					USBEPxR::STAT_RXMask       |
					(1 << USBEPxR::SETUPPos)   |
					(1 << USBEPxR::DTOG_TXPos) |
					USBEPxR::STAT_TXMask);
		//clear address bits
		epVal &= ~USBEPxR::EAMask;
		//set write 0 to clear bits
		epVal |= (1 << USBEPxR::CTR_RXPos) |
				 (1 << USBEPxR::CTR_TXPos);
		//set address
		epVal |= (address << USBEPxR::EAPos);
		
		USBEndpointRegisters->EPxR[endpoint] = epVal;
	}
	
	void SetTxState(uint8_t endpoint, EPTxState state)
	{
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		const uint32_t currentState = (epVal & USBEPxR::STAT_TXMask) >> USBEPxR::STAT_TXPos;
		uint32_t newState = static_cast<uint32_t>(state) ^ currentState;
		newState <<= USBEPxR::STAT_TXPos;
		
		//clear toggle bits
		epVal &= ~( (1 << USBEPxR::DTOG_RXPos) |
					USBEPxR::STAT_RXMask       |
					(1 << USBEPxR::SETUPPos)   |
					(1 << USBEPxR::DTOG_TXPos) |
					USBEPxR::STAT_TXMask);
		
		//set write 0 to clear bits
		epVal |= (1 << USBEPxR::CTR_RXPos) |
				 (1 << USBEPxR::CTR_TXPos);
		
		epVal |= newState;
		
		USBEndpointRegisters->EPxR[endpoint] = epVal;
	}
	
	void SetRxState(uint8_t endpoint, EPRxState state)
	{
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		const uint32_t currentState = (epVal & USBEPxR::STAT_RXMask) >> USBEPxR::STAT_RXPos;
		uint32_t newState = static_cast<uint32_t>(state) ^ currentState;
		newState <<= USBEPxR::STAT_RXPos;
		
		//clear toggle bits
		epVal &= ~( (1 << USBEPxR::DTOG_RXPos) |
					USBEPxR::STAT_RXMask       |
					(1 << USBEPxR::SETUPPos)   |
					(1 << USBEPxR::DTOG_TXPos) |
					USBEPxR::STAT_TXMask);
		
		//set write 0 to clear bits
		epVal |= (1 << USBEPxR::CTR_RXPos) |
				 (1 << USBEPxR::CTR_TXPos);
		
		epVal |= newState;
		
		USBEndpointRegisters->EPxR[endpoint] = epVal;		
	}
	
	void SetEPType(uint8_t endpoint, EPType type)
	{
		const uint32_t epTypeValue = static_cast<uint32_t>(type) << USBEPxR::EP_TYPEPos;
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		
		//clear toggle bits
		epVal &= ~( (1 << USBEPxR::DTOG_RXPos) |
					USBEPxR::STAT_RXMask       |
					(1 << USBEPxR::SETUPPos)   |
					(1 << USBEPxR::DTOG_TXPos) |
					USBEPxR::STAT_TXMask);
		
		//set write 0 to clear bits
		epVal |= (1 << USBEPxR::CTR_RXPos) |
				 (1 << USBEPxR::CTR_TXPos);
		
		//clear current type bits
		epVal &= ~USBEPxR::EP_TYPEMask;
		epVal |= epTypeValue;
		
		USBEndpointRegisters->EPxR[endpoint] = epVal;
	}
	
	bool CorrectReceptionTransfer(uint8_t endpoint)
	{
		return ((USBEndpointRegisters->EPxR[endpoint] & (1 << USBEPxR::CTR_RXPos)) == (1 << USBEPxR::CTR_RXPos));
	}
	
	bool CorrectTransmissionTransfer(uint8_t endpoint)
	{
		return ((USBEndpointRegisters->EPxR[endpoint] & (1 << USBEPxR::CTR_RXPos)) == (1 << USBEPxR::CTR_RXPos));
	}
	
	void ClearCorrectReceptionTransfer(uint8_t endpoint)
	{
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		
		//clear toggle bits
		epVal &= ~( (1 << USBEPxR::DTOG_RXPos) |
					USBEPxR::STAT_RXMask       |
					(1 << USBEPxR::SETUPPos)   |
					(1 << USBEPxR::DTOG_TXPos) |
					USBEPxR::STAT_TXMask);
		
		//set write 0 to clear bits
		epVal |= (1 << USBEPxR::CTR_RXPos) |
				 (1 << USBEPxR::CTR_TXPos);
		
		//clear ctr rx
		epVal &= ~(1 << USBEPxR::CTR_RXPos);
		USBEndpointRegisters->EPxR[endpoint] = epVal;
	}
	
	void ClearCorrectTransmissionTransfer(uint8_t endpoint)
	{
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		
		//clear toggle bits
		epVal &= ~( (1 << USBEPxR::DTOG_RXPos) |
					USBEPxR::STAT_RXMask       |
					(1 << USBEPxR::SETUPPos)   |
					(1 << USBEPxR::DTOG_TXPos) |
					USBEPxR::STAT_TXMask);
		
		//set write 0 to clear bits
		epVal |= (1 << USBEPxR::CTR_RXPos) |
				 (1 << USBEPxR::CTR_TXPos);
		
		//clear ctr rx
		epVal &= ~(1 << USBEPxR::CTR_TXPos);
		USBEndpointRegisters->EPxR[endpoint] = epVal;
	}
	
  }
    
  namespace Control
  {	  
	  namespace CNTR
	  {
		  constexpr uint32_t FRESPos = 0U;
		  constexpr uint32_t PDWNPos = 1U;
		  constexpr uint32_t LP_MODEPos = 2U;
		  constexpr uint32_t FSUSPPos = 3U;
		  constexpr uint32_t RESUMEPos = 4U;
		  constexpr uint32_t L1RESUMEPos = 5U;
		  constexpr uint32_t L1REQMPos = 7U;
		  constexpr uint32_t ESOFMPos = 8U;
		  constexpr uint32_t SOFMPos = 9U;
		  constexpr uint32_t RESETMPos = 10U;
		  constexpr uint32_t SUSPMPos = 11U;
		  constexpr uint32_t WKUPMPos = 12U;
		  constexpr uint32_t ERRMPos = 13U;
		  constexpr uint32_t PMAOVRMPos = 14U;
		  constexpr uint32_t CTRMPos = 15U;
	  }
	  
	  enum class Interrupt
	  {
		  CorrectTransfer,
		  PMAOverUnderRun,
		  Error,
		  Wakeup,
		  Suspend,
		  Reset,
		  StartOfFrame,
		  ExpectedStartOfFrame,
		  L1Request
	  };
	  
	  void PowerDown()
	  {
		  USBRegisters->CNTR |= (1 << CNTR::PDWNPos);
	  }
	  
	  void PowerUp()
	  {
		  USBRegisters->CNTR &= ~(1 << CNTR::PDWNPos);
	  }
	  
	  void ForceReset()
	  {
		  USBRegisters->CNTR |= (1 << CNTR::FRESPos);
	  }
	  
	  void ClearReset()
	  {
		  USBRegisters->CNTR &= ~(1 << CNTR::FRESPos);
	  }
	  
	  void EnableInterrupt(Interrupt type)
	  {
		  uint32_t registerValue = USBRegisters->CNTR;
		  switch(type)
		  {
			  case Interrupt::CorrectTransfer:
				registerValue |= (1 << CNTR::CTRMPos);
				break;
			  case Interrupt::PMAOverUnderRun:
				registerValue |= (1 << CNTR::PMAOVRMPos);
				break;
			  case Interrupt::Error:
				registerValue |= (1 << CNTR::ERRMPos);
				break;
			  case Interrupt::Wakeup:
				registerValue |= (1 << CNTR::WKUPMPos);
				break;
			  case Interrupt::Suspend:
				registerValue |= (1 << CNTR::SUSPMPos);
				break;
			  case Interrupt::Reset:
				registerValue |= (1 << CNTR::RESETMPos);
				break;
			  case Interrupt::StartOfFrame:
				registerValue |= (1 << CNTR::SOFMPos);
				break;
			  case Interrupt::ExpectedStartOfFrame:
				registerValue |= (1 << CNTR::ESOFMPos);
				break;
			  case Interrupt::L1Request:
				registerValue |= (1 << CNTR::L1REQMPos);
				break;
			  default:
				  break;
		  }
		  USBRegisters->CNTR = registerValue;
	  }
	  
	  void DisableInterrupt(Interrupt type)
	  {
		  uint32_t registerValue = USBRegisters->CNTR;
		  switch(type)
		  {
			  case Interrupt::CorrectTransfer:
				registerValue &= ~(1 << CNTR::CTRMPos);
				break;
			  case Interrupt::PMAOverUnderRun:
				registerValue &= ~(1 << CNTR::PMAOVRMPos);
				break;
			  case Interrupt::Error:
				registerValue &= ~(1 << CNTR::ERRMPos);
				break;
			  case Interrupt::Wakeup:
				registerValue &= ~(1 << CNTR::WKUPMPos);
				break;
			  case Interrupt::Suspend:
				registerValue &= ~(1 << CNTR::SUSPMPos);
				break;
			  case Interrupt::Reset:
				registerValue &= ~(1 << CNTR::RESETMPos);
				break;
			  case Interrupt::StartOfFrame:
				registerValue &= ~(1 << CNTR::SOFMPos);
				break;
			  case Interrupt::ExpectedStartOfFrame:
				registerValue &= ~(1 << CNTR::ESOFMPos);
				break;
			  case Interrupt::L1Request:
				registerValue &= ~(1 << CNTR::L1REQMPos);
				break;
			  default:
				  break;
		  }
		  USBRegisters->CNTR = registerValue;
	  }
  }
   
  namespace BufferDescriptor
  {
	  constexpr uint16_t TotalPacketMemoryLength = 1024U;
	  constexpr uint16_t EndpointDescriptorTableLength = 64U;
	  constexpr uint16_t Ep0DataLength = 64U;
	  
	  namespace COUNT_RX
	  {
		  constexpr uint16_t BL_SIZEPos = 15U;
		  constexpr uint16_t NUM_BLOCKPos = 10U;
		  constexpr uint16_t NUM_BLOCKMask = 0x1FU << NUM_BLOCKPos;
	  }
	  uint16_t NextFreeBufferOffset = EndpointDescriptorTableLength + (2 * Ep0DataLength);

	  __packed struct EndpointDescriptor
	  {
		  volatile uint16_t ADDR_TX;
		  volatile uint16_t COUNT_TX;
		  volatile uint16_t ADDR_RX;
		  volatile uint16_t COUNT_RX;
	  };
	  
	  __packed struct Table
	  {
		  volatile EndpointDescriptor Endpoint[8];
	  };
	  
	  static_assert(sizeof(EndpointDescriptor) == 8, "Endpoint descriptor size!");
	  
	  Table volatile * const DescriptorTable = reinterpret_cast<Table *>(0x40006000);

	  bool AllocateEndpointBuffer(uint8_t endpoint, uint16_t size)
	  {
		if(TotalPacketMemoryLength - NextFreeBufferOffset < size)
			return false;
		
		const bool isInEndpoint = 0x80 == (endpoint & 0x80);
		const uint8_t maskedEndpoint = (endpoint & 0x7F); //remove direction bit

		uint16_t address = NextFreeBufferOffset;
		NextFreeBufferOffset += size;

		if(isInEndpoint)
		{
			DescriptorTable->Endpoint[endpoint].ADDR_TX = address;
		}
		else
		{
			DescriptorTable->Endpoint[endpoint].ADDR_RX = address;
			if(size > 62U)
			{
				uint16_t numberOfBlocks = (size / 32) - 1;
				numberOfBlocks <<= COUNT_RX::NUM_BLOCKPos;
				numberOfBlocks &= COUNT_RX::NUM_BLOCKMask;
				numberOfBlocks |= (1 << COUNT_RX::BL_SIZEPos);
				DescriptorTable->Endpoint[endpoint].COUNT_RX = numberOfBlocks;
			}
			else
			{
				uint16_t numberOfBlocks = (size / 2);
				numberOfBlocks <<= COUNT_RX::NUM_BLOCKPos;
				numberOfBlocks &= COUNT_RX::NUM_BLOCKMask;
				DescriptorTable->Endpoint[endpoint].COUNT_RX = numberOfBlocks;
			}
		}
	  }
  }
}

namespace
{
	F0USB *instance;
	uint8_t usbRxBuffer[1024];
}

F0USB::F0USB()
{
	instance = this;
}

void F0USB::RegisterDevice(USBDevice *device)
{
	mDevice = device;
	InitialiseHardware();
}

void F0USB::InitialiseHardware()
{
	F072::Control::PowerUp();
	for(uint32_t i = 0; i < 200; i++)
	{
		asm("isb");
	}
	
	F072::Control::EnableInterrupt(F072::Control::Interrupt::CorrectTransfer);
	F072::Control::EnableInterrupt(F072::Control::Interrupt::Reset);
	
	F072::Control::ClearReset();
}

void F0USB::RegisterEndpoint(uint8_t endpointNumber, EPType type, uint16_t inSize, uint16_t outSize, uint8_t *outRxBuffer)
{
	const uint8_t maskedEndpoint = endpointNumber & 0x7FU; //remove direction bit
	
	mEndpoints[maskedEndpoint].inSize = inSize;
	mEndpoints[maskedEndpoint].outSize = outSize;
	mEndpoints[maskedEndpoint].outRxBuffer = outRxBuffer;
	
	if(EPType::Isochronous == type)
	{  //isoc endpoints are always double buffered
		if(inSize > 0U && outSize > 0U)
		{
			asm("BKPT 0"); //can't handle this configuration in hardware
		}
		
		const uint16_t size = std::max<uint16_t>(inSize, outSize);
		F072::BufferDescriptor::AllocateEndpointBuffer(maskedEndpoint | 0x80, size);
		F072::BufferDescriptor::AllocateEndpointBuffer(maskedEndpoint, size);
	}
	else
	{
		if(inSize > 0U)
		{
			F072::BufferDescriptor::AllocateEndpointBuffer(maskedEndpoint | 0x80, inSize);
		}
		
		if(outSize > 0U)
		{
			F072::BufferDescriptor::AllocateEndpointBuffer(maskedEndpoint, outSize);
		}	
	}
	
	F072::Endpoint::SetEPType(maskedEndpoint, type);
}

void F0USB::Interrupt()
{
	if((F072::USBRegisters->ISTR & (1 << F072::ISTR::RESETPos)) == (1 << F072::ISTR::RESETPos))
	{
		ResetIRQ();
		F072::USBRegisters->ISTR = 0xFFFF & ~(1 << F072::ISTR::RESETPos);
	}
	
	if((F072::USBRegisters->ISTR & (1 << F072::ISTR::CTRPos)) == (1 << F072::ISTR::CTRPos))
	{
		CorrectTransferIRQ();
	}
}

void F0USB::ResetIRQ()
{
	F072::USBRegisters->BTABLE = 0U;
	
	F072::BufferDescriptor::DescriptorTable->Endpoint[0].ADDR_RX = 0x40;
	F072::BufferDescriptor::DescriptorTable->Endpoint[0].ADDR_TX = 0x80;
	F072::BufferDescriptor::DescriptorTable->Endpoint[0].COUNT_RX = (1 << 15) | (1 << 10); //2 * 32 byte blocks
	
	F072::Endpoint::SetAddress(0, 0);
	F072::Endpoint::SetEPType(0, EPType::Control);
	F072::Endpoint::SetRxState(0, EPRxState::Valid);
	F072::Endpoint::SetTxState(0, EPTxState::NAK);
	
	F072::USBRegisters->DADDR = (1 << 7); //enable usb
	F072::USBRegisters->BCDR = (1 << 15); //enable pull up	
}

void F0USB::CorrectTransferIRQ()
{
	const uint8_t hardwareEndpoint = static_cast<uint8_t>(F072::USBRegisters->ISTR & F072::ISTR::EP_IDMask);
	if((F072::USBEndpointRegisters->EPxR[hardwareEndpoint] & (1 << F072::Endpoint::USBEPxR::CTR_RXPos)) == (1 << F072::Endpoint::USBEPxR::CTR_RXPos))
	{
		const uint16_t bytesReceived = F072::BufferDescriptor::DescriptorTable->Endpoint[hardwareEndpoint].COUNT_RX & 0x01FFU;
		uint8_t volatile * const packetMemory = reinterpret_cast<uint8_t *>(F072::BufferDescriptor::DescriptorTable->Endpoint[hardwareEndpoint].ADDR_RX + 0x40006000U);
		uint8_t *userMemory = mEndpoints[hardwareEndpoint].outRxBuffer;
		for(uint32_t i = 0; i < bytesReceived; i++)
		{
			userMemory[i] = packetMemory[i];
		}

		if(0U == hardwareEndpoint)
		{
			mDevice->EP0Out(bytesReceived);
		}
		else
		{
			mDevice->DataOut(hardwareEndpoint, bytesReceived);
		}
		
		F072::Endpoint::ClearCorrectReceptionTransfer(hardwareEndpoint);
		F072::Endpoint::SetRxState(hardwareEndpoint, EPRxState::Valid);
	}
}

void F0USB::TxData(uint8_t endpointNumber, const uint8_t *data, uint16_t length)
{
	uint16_t * volatile packetMemory = reinterpret_cast<uint16_t *>(F072::BufferDescriptor::DescriptorTable->Endpoint[endpointNumber].ADDR_TX + 0x40006000U);
	for(uint32_t i = 0; i < length; i += 2)
	{
		uint16_t halfWord = data[i] | (data[i + 1] << 8);
		packetMemory[i/2] = halfWord;
	}
	F072::BufferDescriptor::DescriptorTable->Endpoint[endpointNumber].COUNT_TX = length;
	F072::Endpoint::SetTxState(0, EPTxState::Valid);
}

extern "C" void USB_IRQHandler()
{
	instance->Interrupt();
}