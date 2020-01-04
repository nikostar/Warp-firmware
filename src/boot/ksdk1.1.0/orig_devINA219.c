#include <stdlib.h>

#include "fsl_i2c_master_driver.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devINA219.h"


extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
	uint8_t		commandByte[1];
	uint16_t	payloadByte[1];
	i2c_status_t	status;

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							((payload>>8) & 0xFF),
							1,
							(payload & 0xFF),
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, uint16_t *value)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;
	
	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							gWarpI2cTimeoutMilliseconds,
							((*value<<8) | *value),
							1,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void setCalibration_32V_1A(){
	ina219_calValue=10240;
	ina219_currentDIvider_mA=25;
	ina219_powerMultiplier_mW=0.8f
	
	writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);

	uint16_t config= INA219_CONFIG_BVOLTAGERANGE_32V |
			 INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
			 INA219_CONFIG_SADCRES_12BIT_1S_532US |
			 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	writeSensorRegisterINA219(INA219_REG_CONFIG,config);

}

void setCalibration_32V_2A(){
	ina219_calValue=4096;
	ina219_currentDIvider_mA=10;
	ina219_powerMultiplier_mW=2
	
	writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);

	uint16_t config= INA219_CONFIG_BVOLTAGERANGE_32V |
			 INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
			 INA219_CONFIG_SADCRES_12BIT_1S_532US |
			 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	writeSensorRegisterINA219(INA219_REG_CONFIG,config);

}

void setCalibration_16V_400mA(){
	ina219_calValue=8192;
	ina219_currentDIvider_mA=20;
	ina219_powerMultiplier_mW=1.0f
	
	writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);

	uint16_t config= INA219_CONFIG_BVOLTAGERANGE_16V |
			 INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
			 INA219_CONFIG_SADCRES_12BIT_1S_532US |
			 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	writeSensorRegisterINA219(INA219_REG_CONFIG,config);

}

int16_t getCurrent_raw(){
	uint16_t value;
	writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);
	readSensorRegisterINA219(INA219_REG_CURRENT, &value);

	return (int16_t)value;
}

float getCurrent_mA(void)
{
	float valueDec=getCurrent_raw();
	valueDec/=ina_currentDivider_mA;
	return valueDec;

}
