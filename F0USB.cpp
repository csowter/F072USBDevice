#include "F0USB.h"
#include <stdint.h>

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

enum class EPType {
	Bulk,
	Control,
	Isochronous,
	Interrupt
};

namespace F072 
{
  constexpr uint32_t USBBaseAddress = 0x40005C00U;
  constexpr uint32_t CommonRegistersOffset = 0x40U;
  constexpr uint32_t ISTROffset = 0x44U;
  
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
	  constexpr uint32_t EP_ISMask = 0x0FU;
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
  }
}

namespace
{
	F0USB *instance;
}


F0USB::F0USB()
{
	InitialiseHardware();
	instance = this;
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
//		F072::USBRegisters->ISTR = 0xFFFF & ~(1 << F072::ISTR::CTRPos);
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

uint8_t rxData[64];

void F0USB::CorrectTransferIRQ()
{
	uint8_t *memory = reinterpret_cast<uint8_t *>(0x40U + 0x40006000);
	for(int i = 0; i < 64; i++)
	{
		rxData[i] = memory[i];
	}
	
}

extern "C" void USB_IRQHandler()
{
	instance->Interrupt();
}