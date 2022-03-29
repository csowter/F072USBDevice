#ifndef f0usb_h
#define f0usb_h

class F0USB
{
public:
	F0USB();
	void Interrupt();

private:
	void InitialiseHardware();
	void ResetIRQ();
	void CorrectTransferIRQ();
};

#endif
