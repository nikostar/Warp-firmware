#include <stdlib.h>

#include "fsl_i2c_master_driver.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

//to set appropriate pins

enum
{
	WaterSensorPinAmberLED		= GPIO_MAKE_PIN(HW_GPIOA, 6),
	WaterSensorPinSuperRedLED	= GPIO_MAKE_PIN(HW_GPIOA, 7),
	WaterSensorPinWhiteLED		= GPIO_MAKE_PIN(HW_GPIOB, 5),
	WaterSensorPinPhotodetector	= GPIO_MAKE_PIN(HW_GPIOB, 1),
};

//might not be needed
void
initWaterSensor()
{
	PORT_HAL_SetMuxMode(PORTA_BASE, 6u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 5u, kPortMuxAsGpio);
	//PORT_HAL_SetMuxMode(PORTB_BASE, 10u, kPortMuxAsGpio);
	return;
}


void ReadPhotodetector(){
	//GPIOB_PDIR & (1<<10)
	//SEGGER_RTT_printf(0, "\r\n\tPhotodetector reading=%d V",
	//				GPIOB_PDIR & (1<<10));
	//SEGGER_RTT_printf(0, "\r\n\tDetectorReading=%d", ADC16_DRV_GETCONVRAW()
					// GPIO_DRV_ReadPinInput(WaterSensorPinPhotodetector));
	
	return;
}

//might only output 1V. Might not be enough... Need to check

void TurnOnAmberLED(){
	GPIO_DRV_SetPinOutput(WaterSensorPinAmberLED);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(WaterSensorPinAmberLED);
	return;
}

void TurnOnSuperRedLED(){
	GPIO_DRV_SetPinOutput(WaterSensorPinSuperRedLED);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(WaterSensorPinSuperRedLED);
	return;
}

void TurnOnWhiteLED(){
	GPIO_DRV_SetPinOutput(WaterSensorPinWhiteLED);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(WaterSensorPinWhiteLED);
	return;
}
