#include <stdlib.h>

#include "fsl_i2c_master_driver.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devTSL2591.h"


extern volatile WarpI2CDeviceState	deviceTSL2591State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

enum
{
	kTSL2591PinSCL		= GPIO_MAKE_PIN(HW_GPIOB, 3),
	kTSL2591PinSDA		= GPIO_MAKE_PIN(HW_GPIOB, 4),
};


void
initTSL2591(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	PORT_HAL_SetMuxMode(PORTB_BASE, 3u, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4u, kPortMuxAlt2);


	deviceStatePointer->i2cAddress	= i2cAddress;
//	deviceStatePointer->signalType	= ();
	return;
}

WarpStatus
writeSensorRegisterTSL2591(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		commandByte[1];
	uint8_t		payloadByte[1];
	i2c_status_t	status;
	i2c_device_t slave =
	{
		.address = deviceTSL2591State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};
	
	commandByte[0] = deviceRegister;
	
	payloadByte[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterTSL2591(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;

	USED(numberOfBytes);
	
	i2c_device_t slave =
	{
		.address = deviceTSL2591State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceTSL2591State.i2cBuffer,
							1,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void TSL2591Enable(){
				uint8_t		i2cAddress, payloadByte[1], commandByte[1];
				i2c_status_t	i2cStatus;
				WarpStatus	status;
					
				i2cAddress = 0x29;
				payloadByte[0] = 0x93;

				i2c_device_t slave =
				{
					.address = i2cAddress,
					.baudRate_kbps = gWarpI2cBaudRateKbps
				};

				enableSssupply(1800);		//equivalent of doing 'g'-->'1800' in warp
				enableI2Cpins(32768);		//hard coded the default value

				commandByte[0] = 0xA0;		//register that controls on/off sensor
				i2cStatus = I2C_DRV_MasterSendDataBlocking(
										0 /* I2C instance */,
										&slave,
										commandByte,
										1,
										payloadByte,
										1,
										gWarpI2cTimeoutMilliseconds);
					if (i2cStatus != kStatus_I2C_Success)
					{
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
						SEGGER_RTT_printf(0, "\r\n\tI2C write failed, error %d.\n\n", i2cStatus);
#endif
					}
					disableI2Cpins();
			
}

void TSL2591Disable(){
				uint8_t		i2cAddress, payloadByte[1], commandByte[1];
				i2c_status_t	i2cStatus;
				WarpStatus	status;
					
				i2cAddress = 0x29;
				payloadByte[0] = 0x00;

				i2c_device_t slave =
				{
					.address = i2cAddress,
					.baudRate_kbps = gWarpI2cBaudRateKbps
				};

				enableSssupply(1800);		//equivalent of doing 'g'-->'1800' in warp
				enableI2Cpins(32768);		//hard coded the default value

				commandByte[0] = 0xA0;		//register that controls on/off sensor
				i2cStatus = I2C_DRV_MasterSendDataBlocking(
										0 /* I2C instance */,
										&slave,
										commandByte,
										1,
										payloadByte,
										1,
										gWarpI2cTimeoutMilliseconds);
					if (i2cStatus != kStatus_I2C_Success)
					{
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
						SEGGER_RTT_printf(0, "\r\n\tI2C write failed, error %d.\n\n", i2cStatus);
#endif
					}
					disableI2Cpins();
			
}


