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
	namespace Registers
	{
		namespace CNTR 
		{
			namespace Position
			{
				constexpr uint32_t CTRM     = 15UL;
				constexpr uint32_t PMAOVRM  = 14UL;
				constexpr uint32_t ERRM     = 13UL;
				constexpr uint32_t WKUPM    = 12UL;
				constexpr uint32_t SUSPM    = 11UL;
				constexpr uint32_t RESETM   = 10UL;
				constexpr uint32_t SOFM     = 9UL;
				constexpr uint32_t ESOFM    = 8UL;
				constexpr uint32_t L1REQM   = 7UL;
				constexpr uint32_t L1RESUME = 5UL;
				constexpr uint32_t RESUME   = 4UL;
				constexpr uint32_t FSUSP    = 3UL;
				constexpr uint32_t LP_MODE  = 2UL;
				constexpr uint32_t PDWN     = 1UL;
				constexpr uint32_t FRES     = 0UL;
			}

			namespace Mask
			{
				[[maybe_unused]] constexpr uint32_t CTRM     = (1UL << Position::CTRM);
				[[maybe_unused]] constexpr uint32_t PMAOVRM  = (1UL << Position::PMAOVRM);
				[[maybe_unused]] constexpr uint32_t ERRM     = (1UL << Position::ERRM);
				[[maybe_unused]] constexpr uint32_t WKUPM    = (1UL << Position::WKUPM);
				[[maybe_unused]] constexpr uint32_t SUSPM    = (1UL << Position::SUSPM);
				[[maybe_unused]] constexpr uint32_t RESETM   = (1UL << Position::RESETM);
				[[maybe_unused]] constexpr uint32_t SOFM     = (1UL << Position::SOFM);
				[[maybe_unused]] constexpr uint32_t ESOFM    = (1UL << Position::ESOFM);
				[[maybe_unused]] constexpr uint32_t L1REQM   = (1UL << Position::L1REQM);
				[[maybe_unused]] constexpr uint32_t L1RESUME = (1UL << Position::L1RESUME);
				[[maybe_unused]] constexpr uint32_t RESUME   = (1UL << Position::RESUME);
				[[maybe_unused]] constexpr uint32_t FSUSP    = (1UL << Position::FSUSP);
				[[maybe_unused]] constexpr uint32_t LP_MODE  = (1UL << Position::LP_MODE);
				[[maybe_unused]] constexpr uint32_t PDWN     = (1UL << Position::PDWN);
				[[maybe_unused]] constexpr uint32_t FRES     = (1UL << Position::FRES);
			}
		}

		namespace ISTR
		{
			namespace Position
			{
				constexpr uint32_t CTR    = 15UL;
				constexpr uint32_t PMAOVR = 14UL;
				constexpr uint32_t ERR    = 13UL;
				constexpr uint32_t WKUP   = 12UL;
				constexpr uint32_t SUSP   = 11UL;
				constexpr uint32_t RESET  = 10UL;
				constexpr uint32_t SOF    = 9UL;
				constexpr uint32_t ESOF   = 8UL;
				constexpr uint32_t L1REQ  = 7UL;
				constexpr uint32_t DIR    = 4UL;
				constexpr uint32_t EP_ID  = 0UL;
			}

			namespace Mask 
			{
				[[maybe_unused]] constexpr uint32_t CTR    = (1UL << Position::CTR);
				[[maybe_unused]] constexpr uint32_t PMAOVR = (1UL << Position::PMAOVR);
				[[maybe_unused]] constexpr uint32_t ERR    = (1UL << Position::ERR);
				[[maybe_unused]] constexpr uint32_t WKUP   = (1UL << Position::WKUP);
				[[maybe_unused]] constexpr uint32_t SUSP   = (1UL << Position::SUSP);
				[[maybe_unused]] constexpr uint32_t RESET  = (1UL << Position::RESET);
				[[maybe_unused]] constexpr uint32_t SOF    = (1UL << Position::SOF);
				[[maybe_unused]] constexpr uint32_t ESOF   = (1UL << Position::ESOF);
				[[maybe_unused]] constexpr uint32_t L1REQ  = (1UL << Position::L1REQ);
				[[maybe_unused]] constexpr uint32_t DIR    = (1UL << Position::DIR);
				[[maybe_unused]] constexpr uint32_t EP_ID  = (0x0FUL << Position::EP_ID);
			}
		}

		namespace FNR
		{
			namespace Position
			{
				constexpr uint32_t RXDP = 15UL;
				constexpr uint32_t RXDM = 14UL;
				constexpr uint32_t LCK  = 13UL;
				constexpr uint32_t LSOF = 11UL;
				constexpr uint32_t FN   = 0UL;
			}

			namespace Mask
			{
				[[maybe_unused]] constexpr uint32_t RXDP = (1UL << Position::RXDP);
				[[maybe_unused]] constexpr uint32_t RXDM = (1UL << Position::RXDM);
				[[maybe_unused]] constexpr uint32_t LCK  = (1UL << Position::LCK);
				[[maybe_unused]] constexpr uint32_t LSOF = (3UL << Position::LSOF);
				[[maybe_unused]] constexpr uint32_t FN   = (0x7FF << Position::FN);
			}
		}

		namespace DADDR
		{
			namespace Position
			{
				constexpr uint32_t EF  = 7UL;
				constexpr uint32_t ADD = 0UL;
			}

			namespace Mask
			{
				[[maybe_unused]] constexpr uint32_t EF  = (1UL << Position::EF);
				[[maybe_unused]] constexpr uint32_t ADD = (0x7FUL << Position::ADD);
			}
		}

		namespace BTABLE
		{
			namespace Position
			{
				constexpr uint32_t BTABLE = 3UL;
			}

			namespace Mask
			{
				[[maybe_unused]] constexpr uint32_t BTABLE = (0x1FFFUL << Position::BTABLE);
			}
		}

		namespace LPMCSR
		{
			namespace Position
			{
				constexpr uint32_t BESL    = 4UL;
				constexpr uint32_t REMWAKE = 3UL;
				constexpr uint32_t LPMACK  = 1UL;
				constexpr uint32_t LPMEN   = 0UL;
			}

			namespace Mask
			{
				[[maybe_unused]] constexpr uint32_t BESL    = (0x0FUL << Position::BESL);
				[[maybe_unused]] constexpr uint32_t REMWAKE = (1UL << Position::REMWAKE);
				[[maybe_unused]] constexpr uint32_t LPMACK  = (1UL << Position::LPMACK);
				[[maybe_unused]] constexpr uint32_t LPMEN   = (1UL << Position::LPMEN);
			}
		}

		namespace BCDR
		{
			namespace Position
			{
				constexpr uint32_t DPPU   = 15UL;
				constexpr uint32_t PS2DET = 7UL;
				constexpr uint32_t SDET   = 6UL;
				constexpr uint32_t PDET   = 5UL;
				constexpr uint32_t DCDET  = 4UL;
				constexpr uint32_t SDEN   = 3UL;
				constexpr uint32_t PDEN   = 2UL;
				constexpr uint32_t DCDEN  = 1UL;
				constexpr uint32_t BCDEN  = 0UL;
			}

			namespace Mask
			{
				[[maybe_unused]] constexpr uint32_t DPPU   = (1UL << Position::DPPU);
				[[maybe_unused]] constexpr uint32_t PS2DET = (1UL << Position::PS2DET);
				[[maybe_unused]] constexpr uint32_t SDET   = (1UL << Position::SDET);
				[[maybe_unused]] constexpr uint32_t PDET   = (1UL << Position::PDET);
				[[maybe_unused]] constexpr uint32_t DCDET  = (1UL << Position::DCDET);
				[[maybe_unused]] constexpr uint32_t SDEN   = (1UL << Position::SDEN);
				[[maybe_unused]] constexpr uint32_t PDEN   = (1UL << Position::PDEN);
				[[maybe_unused]] constexpr uint32_t DCDEN  = (1UL << Position::DCDEN);
				[[maybe_unused]] constexpr uint32_t BCDEN  = (1UL << Position::BCDEN);
			}
		}

		namespace EPnR
		{
			namespace Position
			{
				constexpr uint32_t CTR_RX  = 15UL;
				constexpr uint32_t DTOG_RX = 14UL;
				constexpr uint32_t STAT_RX = 12UL;
				constexpr uint32_t SETUP   = 11UL;
				constexpr uint32_t EP_TYPE = 9UL;
				constexpr uint32_t EP_KIND = 8UL;
				constexpr uint32_t CTR_TX  = 7UL;
				constexpr uint32_t DTOG_TX = 6UL;
				constexpr uint32_t STAT_TX = 4UL;
				constexpr uint32_t EA      = 0UL;
			}

			namespace Mask
			{
				[[maybe_unused]] constexpr uint32_t CTR_RX  = (1UL << Position::CTR_RX);
				[[maybe_unused]] constexpr uint32_t DTOG_RX = (1UL << Position::DTOG_RX);
				[[maybe_unused]] constexpr uint32_t STAT_RX = (3UL << Position::STAT_RX);
				[[maybe_unused]] constexpr uint32_t SETUP   = (1UL << Position::SETUP);
				[[maybe_unused]] constexpr uint32_t EP_TYPE = (3UL << Position::EP_TYPE);
				[[maybe_unused]] constexpr uint32_t EP_KIND = (1UL << Position::EP_KIND);
				[[maybe_unused]] constexpr uint32_t CTR_TX  = (1UL << Position::CTR_TX);
				[[maybe_unused]] constexpr uint32_t DTOG_TX = (1UL << Position::DTOG_TX);
				[[maybe_unused]] constexpr uint32_t STAT_TX = (3UL << Position::STAT_TX);
				[[maybe_unused]] constexpr uint32_t EA      = (0x0FUL << Position::EA);
			}
		}
	}

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

  
  namespace Endpoint 
  {
	void SetAddress(uint8_t endpoint, uint8_t address)
	{
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		//clear toggle bits
		constexpr uint32_t toggleBitsMask = F072::Registers::EPnR::Mask::DTOG_RX |
											F072::Registers::EPnR::Mask::STAT_RX |
											F072::Registers::EPnR::Mask::SETUP   |
											F072::Registers::EPnR::Mask::DTOG_TX |
											F072::Registers::EPnR::Mask::STAT_TX;
		epVal &= ~toggleBitsMask;

		//clear address bits
		epVal &= ~F072::Registers::EPnR::Mask::EA;
		//set write 0 to clear bits
		constexpr uint32_t write0ToClearMask = F072::Registers::EPnR::Mask::CTR_RX |
											   F072::Registers::EPnR::Mask::CTR_TX;
		epVal |= write0ToClearMask;
		//set address
		epVal |= (address << F072::Registers::EPnR::Position::EA);
		
		USBEndpointRegisters->EPxR[endpoint] = epVal;
	}
	
	void SetTxState(uint8_t endpoint, EPTxState state)
	{
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		const uint32_t currentState = 
				((epVal & F072::Registers::EPnR::Mask::STAT_TX) >> F072::Registers::EPnR::Position::STAT_TX);
		uint32_t newState = static_cast<uint32_t>(state) ^ currentState;
		newState <<= F072::Registers::EPnR::Position::STAT_TX;
		
		constexpr uint32_t toggleBitsMask = F072::Registers::EPnR::Mask::DTOG_RX |
											F072::Registers::EPnR::Mask::STAT_RX |
											F072::Registers::EPnR::Mask::SETUP   |
											F072::Registers::EPnR::Mask::DTOG_TX |
											F072::Registers::EPnR::Mask::STAT_TX;
		epVal &= ~toggleBitsMask;
		
		//set write 0 to clear bits
		constexpr uint32_t write0ToClearMask = F072::Registers::EPnR::Mask::CTR_RX |
											   F072::Registers::EPnR::Mask::CTR_TX;
		epVal |= write0ToClearMask;
		
		epVal |= newState; //todo this looks wrong ^= new state? or mask out state and or in
		
		USBEndpointRegisters->EPxR[endpoint] = epVal;
	}
	
	void SetRxState(uint8_t endpoint, EPRxState state)
	{
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		const uint32_t currentState = 
			((epVal & F072::Registers::EPnR::Mask::STAT_RX) >> F072::Registers::EPnR::Position::STAT_RX);
		uint32_t newState = static_cast<uint32_t>(state) ^ currentState;
		newState <<= F072::Registers::EPnR::Position::STAT_RX;
		
		constexpr uint32_t toggleBitsMask = F072::Registers::EPnR::Mask::DTOG_RX |
											F072::Registers::EPnR::Mask::STAT_RX |
											F072::Registers::EPnR::Mask::SETUP   |
											F072::Registers::EPnR::Mask::DTOG_TX |
											F072::Registers::EPnR::Mask::STAT_TX;
		epVal &= ~toggleBitsMask;
		
		//set write 0 to clear bits
		constexpr uint32_t write0ToClearMask = F072::Registers::EPnR::Mask::CTR_RX |
											   F072::Registers::EPnR::Mask::CTR_TX;
		epVal |= write0ToClearMask;
		
		epVal |= newState;
		
		USBEndpointRegisters->EPxR[endpoint] = epVal;		
	}
	
	void SetEPType(uint8_t endpoint, EPType type)
	{
		const uint32_t epTypeValue = static_cast<uint32_t>(type) << F072::Registers::EPnR::Position::EP_TYPE;
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		
		constexpr uint32_t toggleBitsMask = F072::Registers::EPnR::Mask::DTOG_RX |
											F072::Registers::EPnR::Mask::STAT_RX |
											F072::Registers::EPnR::Mask::SETUP   |
											F072::Registers::EPnR::Mask::DTOG_TX |
											F072::Registers::EPnR::Mask::STAT_TX;
		epVal &= ~toggleBitsMask;
		
		//set write 0 to clear bits
		constexpr uint32_t write0ToClearMask = F072::Registers::EPnR::Mask::CTR_RX |
											   F072::Registers::EPnR::Mask::CTR_TX;
		epVal |= write0ToClearMask;
		
		//clear current type bits
		epVal &= ~F072::Registers::EPnR::Mask::EP_TYPE;
		epVal |= epTypeValue;
		
		USBEndpointRegisters->EPxR[endpoint] = epVal;
	}
	
	bool CorrectReceptionTransfer(uint8_t endpoint)
	{
		return (F072::Registers::EPnR::Mask::CTR_RX == (USBEndpointRegisters->EPxR[endpoint] & F072::Registers::EPnR::Mask::CTR_RX));
	}
	
	bool CorrectTransmissionTransfer(uint8_t endpoint)
	{
		return (F072::Registers::EPnR::Mask::CTR_TX == (USBEndpointRegisters->EPxR[endpoint] & F072::Registers::EPnR::Mask::CTR_TX));
	}
	
	void ClearCorrectReceptionTransfer(uint8_t endpoint)
	{
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		
		constexpr uint32_t toggleBitsMask = F072::Registers::EPnR::Mask::DTOG_RX |
											F072::Registers::EPnR::Mask::STAT_RX |
											F072::Registers::EPnR::Mask::SETUP   |
											F072::Registers::EPnR::Mask::DTOG_TX |
											F072::Registers::EPnR::Mask::STAT_TX;
		epVal &= ~toggleBitsMask;
		
		//set write 0 to clear bits
		constexpr uint32_t write0ToClearMask = F072::Registers::EPnR::Mask::CTR_RX |
											   F072::Registers::EPnR::Mask::CTR_TX;
		epVal |= write0ToClearMask;
		
		//clear ctr rx
		epVal &= ~F072::Registers::EPnR::Mask::CTR_RX;
		USBEndpointRegisters->EPxR[endpoint] = epVal;
	}
	
	void ClearCorrectTransmissionTransfer(uint8_t endpoint)
	{
		uint32_t epVal = USBEndpointRegisters->EPxR[endpoint];
		
		constexpr uint32_t toggleBitsMask = F072::Registers::EPnR::Mask::DTOG_RX |
											F072::Registers::EPnR::Mask::STAT_RX |
											F072::Registers::EPnR::Mask::SETUP   |
											F072::Registers::EPnR::Mask::DTOG_TX |
											F072::Registers::EPnR::Mask::STAT_TX;
		epVal &= ~toggleBitsMask;
		
		//set write 0 to clear bits
		constexpr uint32_t write0ToClearMask = F072::Registers::EPnR::Mask::CTR_RX |
											   F072::Registers::EPnR::Mask::CTR_TX;
		epVal |= write0ToClearMask;
		
		//clear ctr rx
		epVal &= ~F072::Registers::EPnR::Mask::CTR_TX;
		USBEndpointRegisters->EPxR[endpoint] = epVal;
	}
	
  }
    
  namespace Control
  {	  
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
		  USBRegisters->CNTR |= F072::Registers::CNTR::Mask::PDWN;
	  }
	  
	  void PowerUp()
	  {
		  USBRegisters->CNTR &= ~F072::Registers::CNTR::Mask::PDWN;
	  }
	  
	  void ForceReset()
	  {
		  USBRegisters->CNTR |= F072::Registers::CNTR::Mask::FRES;
	  }
	  
	  void ClearReset()
	  {
		  USBRegisters->CNTR &= ~F072::Registers::CNTR::Mask::FRES;
	  }
	  
	  void EnableInterrupt(Interrupt type)
	  {
		  uint32_t registerValue = USBRegisters->CNTR;
		  switch(type)
		  {
			  case Interrupt::CorrectTransfer:
				registerValue |= F072::Registers::CNTR::Mask::CTRM;
				break;
			  case Interrupt::PMAOverUnderRun:
				registerValue |= F072::Registers::CNTR::Mask::PMAOVRM;
				break;
			  case Interrupt::Error:
				registerValue |= F072::Registers::CNTR::Mask::ERRM;
				break;
			  case Interrupt::Wakeup:
				registerValue |= F072::Registers::CNTR::Mask::WKUPM;
				break;
			  case Interrupt::Suspend:
				registerValue |= F072::Registers::CNTR::Mask::SUSPM;
				break;
			  case Interrupt::Reset:
				registerValue |= F072::Registers::CNTR::Mask::RESETM;
				break;
			  case Interrupt::StartOfFrame:
				registerValue |= F072::Registers::CNTR::Mask::SOFM;
				break;
			  case Interrupt::ExpectedStartOfFrame:
				registerValue |= F072::Registers::CNTR::Mask::ESOFM;
				break;
			  case Interrupt::L1Request:
				registerValue |= F072::Registers::CNTR::Mask::L1REQM;
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
				registerValue &= ~F072::Registers::CNTR::Mask::CTRM;
				break;
			  case Interrupt::PMAOverUnderRun:
				registerValue &= ~F072::Registers::CNTR::Mask::PMAOVRM;
				break;
			  case Interrupt::Error:
				registerValue &= ~F072::Registers::CNTR::Mask::ERRM;
				break;
			  case Interrupt::Wakeup:
				registerValue &= ~F072::Registers::CNTR::Mask::WKUPM;
				break;
			  case Interrupt::Suspend:
				registerValue &= ~F072::Registers::CNTR::Mask::SUSPM;
				break;
			  case Interrupt::Reset:
				registerValue &= ~F072::Registers::CNTR::Mask::RESETM;
				break;
			  case Interrupt::StartOfFrame:
				registerValue &= ~F072::Registers::CNTR::Mask::SOFM;
				break;
			  case Interrupt::ExpectedStartOfFrame:
				registerValue &= ~F072::Registers::CNTR::Mask::ESOFM;
				break;
			  case Interrupt::L1Request:
				registerValue &= ~F072::Registers::CNTR::Mask::L1REQM;
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
	if(F072::Registers::ISTR::Mask::RESET == (F072::USBRegisters->ISTR & F072::Registers::ISTR::Mask::RESET))
	{
		ResetIRQ();
		F072::USBRegisters->ISTR = 0xFFFF & ~F072::Registers::ISTR::Mask::RESET;
	}
	
	if(F072::Registers::ISTR::Mask::CTR == (F072::USBRegisters->ISTR & F072::Registers::ISTR::Mask::CTR))
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
	const uint8_t hardwareEndpoint = static_cast<uint8_t>(F072::USBRegisters->ISTR & F072::Registers::ISTR::Mask::EP_ID);
	if(F072::Registers::EPnR::Mask::CTR_RX == (F072::USBEndpointRegisters->EPxR[hardwareEndpoint] & F072::Registers::EPnR::Mask::CTR_RX))
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
	
	if(F072::Registers::EPnR::Mask::CTR_TX == (F072::USBEndpointRegisters->EPxR[hardwareEndpoint] & F072::Registers::EPnR::Mask::CTR_TX))
	{
		if(0U == hardwareEndpoint)
		{
			mDevice->EP0In();
		}
		else{
			mDevice->DataIn(hardwareEndpoint);
		}
		
		F072::Endpoint::ClearCorrectTransmissionTransfer(hardwareEndpoint);
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
	F072::Endpoint::SetTxState(endpointNumber, EPTxState::Valid);
}

void F0USB::SetDeviceAddress(uint8_t address)
{
	F072::USBRegisters->DADDR |= address;
}

extern "C" void USB_IRQHandler()
{
	instance->Interrupt();
}