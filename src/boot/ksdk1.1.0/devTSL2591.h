#ifndef WARP_BUILD_ENABLE_DEVTSL2591
#define WARP_BUILD_ENABLE_DEVTSL2591
#endif

typedef enum{
  TSL2591_REGISTER_ENABLE             = 0x00, // Enable register
  TSL2591_REGISTER_CONTROL            = 0x01, // Control register
  TSL2591_REGISTER_THRESHOLD_AILTL    = 0x04, // ALS low threshold lower byte
  TSL2591_REGISTER_THRESHOLD_AILTH    = 0x05, // ALS low threshold upper byte
  TSL2591_REGISTER_THRESHOLD_AIHTL    = 0x06, // ALS high threshold lower byte
  TSL2591_REGISTER_THRESHOLD_AIHTH    = 0x07, // ALS high threshold upper byte
  TSL2591_REGISTER_THRESHOLD_NPAILTL  = 0x08, // No Persist ALS low threshold lower byte
  TSL2591_REGISTER_THRESHOLD_NPAILTH  = 0x09, // No Persist ALS low threshold higher byte
  TSL2591_REGISTER_THRESHOLD_NPAIHTL  = 0x0A, // No Persist ALS high threshold lower byte
  TSL2591_REGISTER_THRESHOLD_NPAIHTH  = 0x0B, // No Persist ALS high threshold higher byte
  TSL2591_REGISTER_PERSIST_FILTER     = 0x0C, // Interrupt persistence filter
  TSL2591_REGISTER_PACKAGE_PID        = 0x11, // Package Identification
  TSL2591_REGISTER_DEVICE_ID          = 0x12, // Device Identification
  TSL2591_REGISTER_DEVICE_STATUS      = 0x13, // Internal Status
  TSL2591_REGISTER_CHAN0_LOW          = 0x14, // Channel 0 data, low byte
  TSL2591_REGISTER_CHAN0_HIGH         = 0x15, // Channel 0 data, high byte
  TSL2591_REGISTER_CHAN1_LOW          = 0x16, // Channel 1 data, low byte
  TSL2591_REGISTER_CHAN1_HIGH         = 0x17 // Channel 1 data, high byte
}TSL2591Constants;

void
initTSL2591(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);

WarpStatus
writeSensorRegisterTSL2591(uint8_t deviceRegister, uint8_t payload);

WarpStatus
readSensorRegisterTSL2591(uint8_t deviceRegister, int numberOfBytes);

void TSL2591Enable();

void TSL2591Disable();

void TurnOnAmberLED();

void TurnOffAmberLED();

void TurnOnSuperRedLED();

void TurnOffSuperRedLED();

void TurnOnWhiteLED();

void TurnOffWhiteLED();


