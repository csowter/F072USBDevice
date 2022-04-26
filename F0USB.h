#ifndef f0usb_h
#define f0usb_h
#include <stdint.h>

enum class EPType {
	Bulk,
	Control,
	Isochronous,
	Interrupt
};


class USBDevice;

class F0USB
{
	struct RegisteredEndpoint
	{
		uint16_t inSize{0};
		uint16_t outSize{0};
		uint8_t *outRxBuffer{nullptr};
	};
	
public:
	F0USB();
	void RegisterDevice(USBDevice *device);
	void Interrupt();
	void RegisterEndpoint(uint8_t endpointNumber, EPType type, uint16_t inSize, uint16_t outSize, uint8_t *outRxBuffer);
	void TxData(uint8_t endpointNumber, const uint8_t *data, uint16_t length);
	void SetDeviceAddress(uint8_t address);

private:
	void InitialiseHardware();
	void ResetIRQ();
	void CorrectTransferIRQ();

	USBDevice *mDevice{nullptr};
	RegisteredEndpoint mEndpoints[8];
};

#endif
