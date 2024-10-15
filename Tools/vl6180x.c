//#include "Arduino.h"

// Define some additional registers mentioned in application notes and we use
///! period between each measurement when in continuous mode

#include "vl6180x.h"

#define SYSRANGE__INTERMEASUREMENT_PERIOD 0x001b
// P19 application notes

/**************************************************************************/
/*!
    @brief  I2C low level interfacing
*/
/**************************************************************************/

// Read 1 byte from the VL6180X at 'address'
uint8_t VL6180X_Read8(VL6180X_Handler_t* device, uint16_t command) {
  device->txBuffer[0] = (uint8_t)(command >> 8);
  device->txBuffer[1] = (uint8_t)(command & 0xFF);
  uint8_t send_address = device->i2cAddress << 1;

  HAL_I2C_Master_Transmit(device->i2cHandler, send_address, device->txBuffer, 2, 1000);
  HAL_I2C_Master_Receive(device->i2cHandler, send_address, device->rxBuffer, 1, 1000);
  return device->rxBuffer[0];
}

// Read 2 byte from the VL6180X at 'address'
uint16_t VL6180X_Read16(VL6180X_Handler_t* device, uint16_t command) {
  device->txBuffer[0] = (uint8_t)(command >> 8);
  device->txBuffer[1] = (uint8_t)(command & 0xFF);
  uint8_t send_address = device->i2cAddress << 1;

  HAL_I2C_Master_Transmit(device->i2cHandler, send_address, device->txBuffer, 2, 1000);
  HAL_I2C_Master_Receive(device->i2cHandler, send_address, device->rxBuffer, 2, 1000);
  return ((uint16_t)(device->rxBuffer[0]) << 8) | ((uint16_t)(device->rxBuffer[1]));
}

// write 1 byte
void VL6180X_Write8(VL6180X_Handler_t* device, uint16_t command, uint8_t data) {
  device->txBuffer[0] = (uint8_t)(command >> 8);
  device->txBuffer[1] = (uint8_t)(command & 0xFF);
  uint8_t send_address = device->i2cAddress << 1;

  device->txBuffer[2] = data;
  HAL_I2C_Master_Transmit(device->i2cHandler, send_address, device->txBuffer, 3, 1000);
}

// write 2 bytes
void VL6180X_Write16(VL6180X_Handler_t* device, uint16_t command, uint16_t data) {
  device->txBuffer[0] = (uint8_t)(command >> 8);
  device->txBuffer[1] = (uint8_t)(command & 0xFF);
  device->txBuffer[2] = (uint8_t)(data >> 8);
  device->txBuffer[2] = (uint8_t)(data & 0xFF);
  uint8_t send_address = device->i2cAddress << 1;

  HAL_I2C_Master_Transmit(device->i2cHandler, send_address, device->txBuffer, 4, 1000);
}

bool VL6180X_Init(VL6180X_Handler_t *device, I2C_HandleTypeDef *i2cHandler, uint8_t i2cAddress) {
  device->i2cHandler = i2cHandler;
  device->i2cAddress = i2cAddress;

  if (VL6180X_Read8(device, VL6180X_REG_IDENTIFICATION_MODEL_ID) != 0xB4) {
    return false;
  }

  if (VL6180X_Read8(device, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET) & 0x01) {
    VL6180X_LoadSettings(device);
    VL6180X_Write8(device, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Load the settings for proximity/distance ranging
*/
/**************************************************************************/

void VL6180X_LoadSettings(VL6180X_Handler_t *device) {
  // load settings!

  // private settings from page 24 of app note
  VL6180X_Write8(device, 0x0207, 0x01);
  VL6180X_Write8(device, 0x0208, 0x01);
  VL6180X_Write8(device, 0x0096, 0x00);
  VL6180X_Write8(device, 0x0097, 0xfd);
  VL6180X_Write8(device, 0x00e3, 0x00);
  VL6180X_Write8(device, 0x00e4, 0x04);
  VL6180X_Write8(device, 0x00e5, 0x02);
  VL6180X_Write8(device, 0x00e6, 0x01);
  VL6180X_Write8(device, 0x00e7, 0x03);
  VL6180X_Write8(device, 0x00f5, 0x02);
  VL6180X_Write8(device, 0x00d9, 0x05);
  VL6180X_Write8(device, 0x00db, 0xce);
  VL6180X_Write8(device, 0x00dc, 0x03);
  VL6180X_Write8(device, 0x00dd, 0xf8);
  VL6180X_Write8(device, 0x009f, 0x00);
  VL6180X_Write8(device, 0x00a3, 0x3c);
  VL6180X_Write8(device, 0x00b7, 0x00);
  VL6180X_Write8(device, 0x00bb, 0x3c);
  VL6180X_Write8(device, 0x00b2, 0x09);
  VL6180X_Write8(device, 0x00ca, 0x09);
  VL6180X_Write8(device, 0x0198, 0x01);
  VL6180X_Write8(device, 0x01b0, 0x17);
  VL6180X_Write8(device, 0x01ad, 0x00);
  VL6180X_Write8(device, 0x00ff, 0x05);
  VL6180X_Write8(device, 0x0100, 0x05);
  VL6180X_Write8(device, 0x0199, 0x05);
  VL6180X_Write8(device, 0x01a6, 0x1b);
  VL6180X_Write8(device, 0x01ac, 0x3e);
  VL6180X_Write8(device, 0x01a7, 0x1f);
  VL6180X_Write8(device, 0x0030, 0x00);

  // Recommended : Public registers - See data sheet for more detail
  VL6180X_Write8(device, 0x0011, 0x10); // Enables polling for 'New Sample ready'
                        // when measurement completes
  VL6180X_Write8(device, 0x010a, 0x30); // Set the averaging sample period
                        // (compromise between lower noise and
                        // increased execution time)
  VL6180X_Write8(device, 0x003f, 0x46); // Sets the light and dark gain (upper
                        // nibble). Dark gain should not be
                        // changed.
  VL6180X_Write8(device, 0x0031, 0xFF); // sets the # of range measurements after
                        // which auto calibration of system is
                        // performed
  VL6180X_Write8(device, 0x0041, 0x63); // Set ALS integration time to 100ms
  VL6180X_Write8(device, 0x002e, 0x01); // perform a single temperature calibration
                        // of the ranging sensor

  // Optional: Public registers - See data sheet for more detail
  VL6180X_Write8(device, SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);
                        // Set default ranging inter-measurement
                        // period to 100ms
  VL6180X_Write8(device, 0x003e, 0x31); // Set default ALS inter-measurement period
                        // to 500ms
  VL6180X_Write8(device, 0x0014, 0x24); // Configures interrupt on 'New Sample
                        // Ready threshold event'
}

/**************************************************************************/
/*!
    @brief  Single shot ranging. Be sure to check the return of {@link
   readRangeStatus} to before using the return value!
    @return Distance in millimeters if valid
*/
/**************************************************************************/

uint8_t VL6180X_ReadRange(VL6180X_Handler_t *device) {
  // wait for device to be ready for range measurement
  while (!(VL6180X_Read8(device, VL6180X_REG_RESULT_RANGE_STATUS) & 0x01));

  // Start a range measurement
  VL6180X_Write8(device, VL6180X_REG_SYSRANGE_START, 0x01);

  // Poll until bit 2 is set
  while (!(VL6180X_Read8(device, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04));

  // read range in mm
  uint8_t range = VL6180X_Read8(device, VL6180X_REG_RESULT_RANGE_VAL);

  // clear interrupt
  VL6180X_Write8(device, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

  return range;
}

/**************************************************************************/
/*!
    @brief  start Single shot ranging. The caller of this should have code
    that waits until the read completes, by either calling
    {@link waitRangeComplete} or calling {@link isRangeComplete} until it
    returns true.  And then the code should call {@link readRangeResult}
    to retrieve the range value and clear out the internal status.
    @return true if range completed.
*/
/**************************************************************************/

bool VL6180X_StartRange(VL6180X_Handler_t *device) {
  // wait for device to be ready for range measurement
  while (!(VL6180X_Read8(device, VL6180X_REG_RESULT_RANGE_STATUS) & 0x01));

  // Start a range measurement
  VL6180X_Write8(device, VL6180X_REG_SYSRANGE_START, 0x01);

  return true;
}

/**************************************************************************/
/*!
    @brief  Check to see if the range command completed.
    @return true if range completed.
*/
/**************************************************************************/

bool VL6180X_IsRangeComplete(VL6180X_Handler_t *device) {
  // Poll until bit 2 is set
  if ((VL6180X_Read8(device, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04)) return true;
  return false;
}

/**************************************************************************/
/*!
    @brief  Wait until Range completed
    @return true if range completed.
*/
/**************************************************************************/

bool VL6180X_WaitRangeComplete(VL6180X_Handler_t *device) {
  // Poll until bit 2 is set
  while (!(VL6180X_Read8(device, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04));
  return true;
}

/**************************************************************************/
/*!
    @brief  Return results of read reqyest also clears out the interrupt
    Be sure to check the return of {@link readRangeStatus} to before using
    the return value!
    @return if range started.
*/
/**************************************************************************/

uint8_t VL6180X_ReadRangeResult(VL6180X_Handler_t *device) {
  // read range in mm
  uint8_t range = VL6180X_Read8(device, VL6180X_REG_RESULT_RANGE_VAL);
  // clear interrupt
  VL6180X_Write8(device, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);
  return range;
}

/**************************************************************************/
/*!
    @brief  Start continuous ranging
    @param  period_ms Optional Period between ranges in ms.  Values will
    be rounded down to 10ms units with minimum of 10ms.  Default is 50
*/
/**************************************************************************/

void VL6180X_StartRangeContinuous(VL6180X_Handler_t *device, uint16_t period_ms) {
  uint8_t period_reg = 0;
  if (period_ms > 10) {
    if (period_ms < 2550)
      period_reg = (period_ms / 10) - 1;
    else
      period_reg = 254;
  }
  // Set  ranging inter-measurement
  VL6180X_Write8(device, SYSRANGE__INTERMEASUREMENT_PERIOD, period_reg);
  // Start a continuous range measurement
  VL6180X_Write8(device, VL6180X_REG_SYSRANGE_START, 0x03);
}

/**************************************************************************/
/*!
    @brief stop continuous range operation.
*/
/**************************************************************************/

void VL6180X_StopRangeContinuous(VL6180X_Handler_t *device) {
  // stop the continuous range operation, by setting the range register
  // back to 1, Page 7 of appication notes
  VL6180X_Write8(device, VL6180X_REG_SYSRANGE_START, 0x01);
}

/**************************************************************************/
/*!
    @brief  Request ranging success/error message (retreive after ranging)
    @returns One of possible VL6180X_ERROR_* values
*/
/**************************************************************************/

uint8_t VL6180X_ReadRangeStatus(VL6180X_Handler_t *device) {
  return (VL6180X_Read8(device, VL6180X_REG_RESULT_RANGE_STATUS) >> 4);
}

/**************************************************************************/
/*!
    @brief  Single shot lux measurement
    @param  gain Gain setting, one of VL6180X_ALS_GAIN_*
    @returns Lux reading
*/
/**************************************************************************/

float VL6180X_ReadLux(VL6180X_Handler_t *device, uint8_t gain) {
  uint8_t reg;

  reg = VL6180X_Read8(device, VL6180X_REG_SYSTEM_INTERRUPT_CONFIG);
  reg &= ~0x38;
  reg |= (0x4 << 3); // IRQ on ALS ready
  VL6180X_Write8(device, VL6180X_REG_SYSTEM_INTERRUPT_CONFIG, reg);

  // 100 ms integration period
  VL6180X_Write8(device, VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI, 0);
  VL6180X_Write8(device, VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO, 100);

  // analog gain
  if (gain > VL6180X_ALS_GAIN_40) {
    gain = VL6180X_ALS_GAIN_40;
  }
  VL6180X_Write8(device, VL6180X_REG_SYSALS_ANALOGUE_GAIN, 0x40 | gain);

  // start ALS
  VL6180X_Write8(device, VL6180X_REG_SYSALS_START, 0x1);

  // Poll until "New Sample Ready threshold event" is set
  while (4 != ((VL6180X_Read8(device, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) >> 3) & 0x7));

  // read lux!
  float lux = VL6180X_Read16(device, VL6180X_REG_RESULT_ALS_VAL);

  // clear interrupt
  VL6180X_Write8(device, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

  lux *= 0.32; // calibrated count/lux
  switch (gain) {
  case VL6180X_ALS_GAIN_1:
    break;
  case VL6180X_ALS_GAIN_1_25:
    lux /= 1.25;
    break;
  case VL6180X_ALS_GAIN_1_67:
    lux /= 1.67;
    break;
  case VL6180X_ALS_GAIN_2_5:
    lux /= 2.5;
    break;
  case VL6180X_ALS_GAIN_5:
    lux /= 5;
    break;
  case VL6180X_ALS_GAIN_10:
    lux /= 10;
    break;
  case VL6180X_ALS_GAIN_20:
    lux /= 20;
    break;
  case VL6180X_ALS_GAIN_40:
    lux /= 40;
    break;
  }
  lux *= 100;
  lux /= 100; // integration time in ms

  return lux;
}

/**************************************************************************/
/*!
    @brief  Set the offset
    @param  offset Offset setting
*/
/**************************************************************************/

void VL6180X_SetOffset(VL6180X_Handler_t *device, uint8_t offset) {
  // write the offset
  VL6180X_Write8(device, VL6180X_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset);
}

/**************************************************************************/
/*!
    @brief  Get the 7 bytes of id
    @param  id_ptr Pointer to array of id bytes
*/
/**************************************************************************/

void VL6180X_GetID(VL6180X_Handler_t *device, uint8_t *id_ptr) {
  id_ptr[0] = VL6180X_Read8(device, VL6180X_REG_IDENTIFICATION_MODEL_ID + 0);
  id_ptr[1] = VL6180X_Read8(device, VL6180X_REG_IDENTIFICATION_MODEL_ID + 1);
  id_ptr[2] = VL6180X_Read8(device, VL6180X_REG_IDENTIFICATION_MODEL_ID + 2);
  id_ptr[3] = VL6180X_Read8(device, VL6180X_REG_IDENTIFICATION_MODEL_ID + 3);
  id_ptr[4] = VL6180X_Read8(device, VL6180X_REG_IDENTIFICATION_MODEL_ID + 4);
  id_ptr[6] = VL6180X_Read8(device, VL6180X_REG_IDENTIFICATION_MODEL_ID + 6);
  id_ptr[7] = VL6180X_Read8(device, VL6180X_REG_IDENTIFICATION_MODEL_ID + 7);
}

