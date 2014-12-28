//=====================================================================================================
// xIMUReceiver.c
// 06/08/2012
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Includes

#include <math.h>
#include "xIMUReceiver.h"

//---------------------------------------------------------------------------------------------------
// Definitions

typedef enum {
    QCalibratedBattery = 12,
    QCalibratedThermometer = 8,
    QCalibratedGyroscope = 4,
    QCalibratedAccelerometer = 11,
    QCalibratedMagnetometer = 11,
	QQuaternion = 15,
    QBatterySensitivity = 5,
    QBatteryBias = 8,
    QThermometerSensitivity = 6,
    QThermometerBias = 0,
    QGyroscopeSensitivity = 7,
	QGyroscopeSampled200dps = 0,
    QGyroscopeBiasAt25degC = 3,
	QGyroscopeBiasTempSensitivity = 11,
	QGyroscopeSampledBias = 3,
    QAccelerometerSensitivity = 4,
    QAccelerometerBias = 8,
	QAccelerometerSampled1g = 4,
    QMagnetometerSensitivity = 4,
    QMagnetometerBias = 8,
	QMagnetometerHardIronBias = 11,
    QAlgorithmKp = 11,
    QAlgorithmKi = 15,
	QAlgorithmInitKp = 11,
	QAlgorithmInitPeriod = 11,
	QCalibratedAnalogueInput = 12,
	QAnalogueInputSensitivity = 4,
	QAnalogueInputBias = 8,
    QCalibratedADXL345 = 10,
	QADXL345busSensitivity = 6,
	QADXL345busBias = 8,
} FixedQ;

//----------------------------------------------------------------------------------------------------
// Variables

float gyroscope[3] = { 0, 0, 0 };
float accelerometer[3] = { 0, 0, 0 };
float magnetometer[3] = { 0, 0, 0 };
float quaternion[4] = { 0, 0, 0 };
float eulerAngles[3] = { 0, 0, 0 };

unsigned char shiftReg[256];

//----------------------------------------------------------------------------------------------------
// Function declarations

static int DecodePacket(const unsigned char* const rxBuf, unsigned char* const decodedPacket, const int rxBufSize);
static void LeftShift(void);
float Fixed16ToFloat(const short fixed, const FixedQ q);
unsigned short Concat(const unsigned char msb, const unsigned char lsb);

//----------------------------------------------------------------------------------------------------
// Functions

int xIMUReceiverIsFramingChar(const unsigned char c) {
	return (c & 0x80);
}

ErrorCode xIMUReceiverProcess(const unsigned char* const rxBuf, const unsigned char rxBufSize, PacketHeader* const packetHeader) {
	unsigned char decodedPacket[256];
	int decodedPacketSize;
	unsigned char checksum;

	// Decode receive buffer
	decodedPacketSize = DecodePacket(rxBuf, decodedPacket, rxBufSize);

	// Verify checksum
	checksum = 0;
	for(int i = 0; i < decodedPacketSize - 1; i++) {
		checksum += decodedPacket[i];
	}
	if(checksum != decodedPacket[decodedPacketSize - 1]) {
		return ErrInvalidChecksum;
	}

	// Interpret packet according to packet header
	switch(decodedPacket[0]) {
		case(PktCalInertialAndMagneticData):
			*packetHeader = PktCalInertialAndMagneticData;
			if(decodedPacketSize != 20) {
				return ErrInvalidNumBytesForPacketHeader;
			}
			gyroscope[0] = Fixed16ToFloat(Concat(decodedPacket[1], decodedPacket[2]), QCalibratedGyroscope);
			gyroscope[1] = Fixed16ToFloat(Concat(decodedPacket[3], decodedPacket[4]), QCalibratedGyroscope);
			gyroscope[2] = Fixed16ToFloat(Concat(decodedPacket[5], decodedPacket[6]), QCalibratedGyroscope);
			accelerometer[0] = Fixed16ToFloat(Concat(decodedPacket[7], decodedPacket[8]), QCalibratedAccelerometer);
			accelerometer[1] = Fixed16ToFloat(Concat(decodedPacket[9], decodedPacket[10]), QCalibratedAccelerometer);
			accelerometer[2] = Fixed16ToFloat(Concat(decodedPacket[11], decodedPacket[12]), QCalibratedAccelerometer);
			magnetometer[0] = Fixed16ToFloat(Concat(decodedPacket[13], decodedPacket[14]), QCalibratedMagnetometer);
			magnetometer[1] = Fixed16ToFloat(Concat(decodedPacket[15], decodedPacket[16]), QCalibratedMagnetometer);
			magnetometer[2] = Fixed16ToFloat(Concat(decodedPacket[17], decodedPacket[18]), QCalibratedMagnetometer);
			break;
		case(PktQuaternionData):
			*packetHeader = PktQuaternionData;
			if(decodedPacketSize != 10) {
				return ErrInvalidNumBytesForPacketHeader;
			}
			quaternion[0] = Fixed16ToFloat(Concat(decodedPacket[1], decodedPacket[2]), QQuaternion);
			quaternion[1] = Fixed16ToFloat(Concat(decodedPacket[3], decodedPacket[4]), QQuaternion);
			quaternion[2] = Fixed16ToFloat(Concat(decodedPacket[5], decodedPacket[6]), QQuaternion);
			quaternion[3] = Fixed16ToFloat(Concat(decodedPacket[7], decodedPacket[8]), QQuaternion);
			eulerAngles[0] = 57.296f * atan2(2.0f * (quaternion[2] * quaternion[3] - quaternion[0] * quaternion[1]), 2.0f * quaternion[0] * quaternion[0] - 1.0f + 2.0f * quaternion[3] * quaternion[3]);
			eulerAngles[1] = 57.296f * atan((2.0f * (quaternion[1] * quaternion[3] + quaternion[0] * quaternion[2])) / sqrt(1.0f - pow((2.0 * quaternion[1] * quaternion[3] + 2.0f * quaternion[0] * quaternion[2]), 2.0f)));
			eulerAngles[2] = 57.296f * atan2(2.0f * quaternion[0] * quaternion[0] - 1.0f + 2.0f * quaternion[1] * quaternion[1], 2.0f * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]));
			break;
		default:
			return ErrUnknownHeader;
	}

	// Return successfully
	return ErrNoError;
}

static int DecodePacket(const unsigned char* const rxBuf, unsigned char* const decodedPacket, const int rxBufSize) {
	int decodedPacketSize = rxBufSize - 1 - ((rxBufSize - 1) >> 3);
    for (int i = rxBufSize - 1; i >= 0; i--) {
        shiftReg[i] = rxBuf[i];
        LeftShift();
    }
	for (int i = 0; i < decodedPacketSize; i++) {
        decodedPacket[i] = shiftReg[i];
    }
    return decodedPacketSize;
}

static void LeftShift(void) {
    shiftReg[0] <<= 1;
    for (int i = 1; i < sizeof(shiftReg); i++) {
        if (shiftReg[i] & 0x80) {
        	shiftReg[i - 1] |= 0x01;
        }
        shiftReg[i] <<= 1;
    }
}

float Fixed16ToFloat(const short fixed, const FixedQ q) {
	return (float)fixed / (float)(1 << q);
}

unsigned short Concat(const unsigned char msb, const unsigned char lsb) {
	return ((unsigned short)msb << 8) | (unsigned short)lsb;
}

//=====================================================================================================
// End of file
//=====================================================================================================
