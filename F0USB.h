#ifndef f0usb_h
#define f0usb_h

#include <stdint.h>

enum class EPType {
	Bulk,
	Control,
	Isochronous,
	Interrupt
};



class F0USB
{
	struct RegisteredEndpoint
	{
		uint16_t inSize;
		uint16_t outSize;
		uint8_t *outRxBuffer;
	};
	
public:
	F0USB();
	void Interrupt();
	void RegisterEndpoint(uint8_t endpointNumber, EPType type, uint16_t inSize, uint16_t outSize, uint8_t *outRxBuffer);
	void TxData(uint8_t endpointNumber, const uint8_t *data, uint16_t length);

private:
	void InitialiseHardware();
	void ResetIRQ();
	void CorrectTransferIRQ();

	

	RegisteredEndpoint mEndpoints[8];
};

#endif
