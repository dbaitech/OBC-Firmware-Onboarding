#include "lm75bd.h"
#include "i2c_io.h"
#include "errors.h"
#include "logging.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

/* LM75BD Registers (p.8) */
#define LM75BD_REG_TEMP 0x00U  /* Temperature Register (R/W) */
#define LM75BD_REG_CONF 0x01U  /* Configuration Register (R/W) */

error_code_t lm75bdInit(lm75bd_config_t *config) {
  error_code_t errCode;

  if (config == NULL) return ERR_CODE_INVALID_ARG;

  RETURN_IF_ERROR_CODE(writeConfigLM75BD(config->devAddr, config->osFaultQueueSize, config->osPolarity,
                                         config->osOperationMode, config->devOperationMode));

  // Assume that the overtemperature and hysteresis thresholds are already set
  // Hysteresis: 75 degrees Celsius
  // Overtemperature: 80 degrees Celsius

  return ERR_CODE_SUCCESS;
}

#define TEMP_REG_WRITE_BUFF_SIZE 1U
#define TEMP_READ_BUFF_SIZE 2U
error_code_t readTempLM75BD(uint8_t devAddr, float *temp) {
  /* Implement this driver function */
  if (temp == NULL) {
    return ERR_CODE_INVALID_ARG;
  }

  error_code_t errCode;

  // Set up the buffer to store the temperature register address
  // 0: Register address
  uint8_t writeBuff[TEMP_REG_WRITE_BUFF_SIZE] = { LM75BD_REG_TEMP };

  // Send the pointer to set up reading from the temperature register
  errCode = i2cSendTo(devAddr, writeBuff, TEMP_REG_WRITE_BUFF_SIZE);
  RETURN_IF_ERROR_CODE(errCode);

  // Set up the buffer to store the temperature data received
  uint8_t readBuff[TEMP_READ_BUFF_SIZE] = { 0, 0 };

  // Read from the temperature register
  errCode = i2cReceiveFrom(devAddr, readBuff, TEMP_READ_BUFF_SIZE);
  RETURN_IF_ERROR_CODE(errCode);

  // Convert temperature register value to degrees Celsius
  // 1. Extract the temperature data (D10 to D0)
  // | D10 D9 D8 D7 D6 D5 D4 D3 | D2 D1 D0 X X X X X |
  int16_t temp_data = (readBuff[0] << 3) & 0x7F8;
  temp_data |= (readBuff[1] >> 5) & 0x07;

  // 2. Checking if D10 is 0 (positive) or 1 (negative)
  // and sign extending to allow for two's complement if D10 is 1
  if (temp_data & (1 << 10)) {
    temp_data |= ~0x7FF;
  }

  // 3. Rescaling the temperature data
  *temp = temp_data * 0.125;

  return ERR_CODE_SUCCESS;
}

#define CONF_WRITE_BUFF_SIZE 2U
error_code_t writeConfigLM75BD(uint8_t devAddr, uint8_t osFaultQueueSize, uint8_t osPolarity,
                                   uint8_t osOperationMode, uint8_t devOperationMode) {
  error_code_t errCode;

  // Stores the register address and data to be written
  // 0: Register address
  // 1: Data
  uint8_t buff[CONF_WRITE_BUFF_SIZE] = {0};

  buff[0] = LM75BD_REG_CONF;

  uint8_t osFaltQueueRegData = 0;
  switch (osFaultQueueSize) {
    case 1:
      osFaltQueueRegData = 0;
      break;
    case 2:
      osFaltQueueRegData = 1;
      break;
    case 4:
      osFaltQueueRegData = 2;
      break;
    case 6:
      osFaltQueueRegData = 3;
      break;
    default:
      return ERR_CODE_INVALID_ARG;
  }

  buff[1] |= (osFaltQueueRegData << 3);
  buff[1] |= (osPolarity << 2);
  buff[1] |= (osOperationMode << 1);
  buff[1] |= devOperationMode;

  errCode = i2cSendTo(LM75BD_OBC_I2C_ADDR, buff, CONF_WRITE_BUFF_SIZE);
  if (errCode != ERR_CODE_SUCCESS) return errCode;

  return ERR_CODE_SUCCESS;
}
