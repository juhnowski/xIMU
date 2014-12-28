//=====================================================================================================
// Example.cpp
// 06/08/2012
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Includes

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "xIMUReceiver.h"

//----------------------------------------------------------------------------------------------------
// Functions

int main(void) {

	// Serial port variables
	int serialHandle;
	struct termios options;
	unsigned char numBytes;
	unsigned char serialBuf[256];

	// Packet decoding variables
	unsigned char rxBuf[256];
	unsigned char rxBufSize = 0;
	ErrorCode ErrorCode = ErrNoError;
	PacketHeader packetHeader;

	// Open serial port
	serialHandle = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY );
	if (serialHandle == -1) {
		printf("Could not open serial port\n");
	}
	else {
		fcntl(serialHandle, F_SETFL, 0);
		printf("Opened serial port successfully!\n");
	}
	bzero(&options, sizeof(options)); 	// set all bits to zero
	options.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;
	tcflush(serialHandle, TCIFLUSH); 				// clean the serial line
	tcsetattr(serialHandle, TCSANOW, &options); 	//set the new option for the port

	// Main loop
	while(1) {
		// Fetch data from serial port
		numBytes = read(serialHandle, &serialBuf, sizeof(serialBuf));

		// Process each byte
		for(int i = 0; i < numBytes; i++) {

			// Add data to receive buffer
			rxBuf[rxBufSize++] = serialBuf[i];

			// Process receive buffer if framing char received
			if(xIMUReceiverIsFramingChar(serialBuf[i]))	{
				ErrorCode = xIMUReceiverProcess(rxBuf, rxBufSize, &packetHeader);
				rxBufSize = 0;	// clear receive buffer

				// Print data if decoded successfully else print Error code
				if(ErrorCode == ErrNoError) {
					switch(packetHeader) {
						case(PktCalInertialAndMagneticData):
							printf("Gyr: %8.2f, %8.2f, %8.2f, Acc: %8.2f, %8.2f, %8.2f, Mag: %8.2f, %8.2f, %8.2f\n",gyroscope[0], gyroscope[1], gyroscope[2], accelerometer[0], accelerometer[1], accelerometer[2], magnetometer[0], magnetometer[1], magnetometer[2]);
							break;
						case(PktQuaternionData):
							printf("Pitch = %6.1f, \tRoll = %6.1f, \tYaw = %6.1f\n", eulerAngles[0], eulerAngles[1], eulerAngles[2]);
							break;
						default:
							break;
					}
				}
				else{
					printf("Error Code: %d\n", ErrorCode);
				}
			}
		}
	}
}

//=====================================================================================================
// End of file
//=====================================================================================================
