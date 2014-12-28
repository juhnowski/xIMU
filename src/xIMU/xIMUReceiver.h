//=====================================================================================================
// xIMUReceiver.h
// 06/08/2012
//=====================================================================================================
#ifndef xIMUReceiver_h
#define xIMUReceiver_h

//---------------------------------------------------------------------------------------------------
// Definitions

typedef enum {
	PktError,
	PktCommand,
	PktReadRegister,
	PktWriteRegister,
	PktReadDateTime,
	PktWriteDateTime,
	PktRawBatteryAndThermometerData,
	PktCalBatteryAndThermometerData,
	PktRawInertialAndMagneticData,
 	PktCalInertialAndMagneticData,
	PktQuaternionData,
	PktDigitalIOdata,
	PktRawAnalogueInputData,
	PktCalAnalogueInputData,
	PktPWMoutputData,
	PktRawADXL345busData,
	PktCalADXL345busData,
} PacketHeader;

typedef enum {
	ErrNoError,
	ErrFactoryResetFailed,
	ErrLowBattery,
	ErrUSBreceiveBufferOverrun,
	ErrUSBtransmitBufferOverrun,
	ErrBluetoothReceiveBufferOverrun,
	ErrBluetoothTransmitBufferOverrun,
	ErrSDcardWriteBufferOverrun,
	ErrTooFewBytesInPacket,
	ErrTooManyBytesInPacket,
	ErrInvalidChecksum,
	ErrUnknownHeader,
	ErrInvalidNumBytesForPacketHeader,
	ErrInvalidRegisterAddress,
	ErrRegisterReadOnly,
	ErrInvalidRegisterValue,
	ErrInvalidCommand,
	ErrGyroscopeAxisNotAt200dps,
	ErrGyroscopeNotStationary,
	ErrAcceleroemterAxisNotAt1g,
	ErrMagnetometerSaturation,
	ErrIncorrectAuxillaryPortMode,
	ErrUARTreceiveBufferOverrun,
	ErrUARTtransmitBufferOverrun
} ErrorCode;

//---------------------------------------------------------------------------------------------------
// Variables

extern float gyroscope[3];
extern float accelerometer[3];
extern float magnetometer[3];
extern float quaternion[4];
extern float eulerAngles[3];

//---------------------------------------------------------------------------------------------------
// Function declarations

int xIMUReceiverIsFramingChar(const unsigned char c);
ErrorCode xIMUReceiverProcess(const unsigned char* const rxBuf, const unsigned char rxBufSize, PacketHeader* const packetHeader);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
