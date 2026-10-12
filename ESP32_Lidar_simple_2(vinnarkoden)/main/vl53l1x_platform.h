/*
  Platform (I2C) layer for ST's VL53L1X ULD API, targeting ESP-IDF's
  "new" i2c_master driver (driver/i2c_master.h), and supporting several
  VL53L1X sensors at different I2C addresses on one shared bus.

  This header declares exactly the functions VL53L1X_api.c expects from
  its platform layer (the official ULD API's own vl53l1x_platform.h has
  the same shape) - drop the official VL53L1X_api.c/.h next to these files
  and it will link straight against this implementation.

  "dev" throughout is the sensor's current 7-bit I2C address. Since only
  one sensor is ever awake on the bus with the factory-default address
  (0x29) during the XSHUT bring-up sequence in main.c, and every sensor
  gets a unique address before its neighbor wakes up, "dev" is enough to
  uniquely pick out the right physical sensor - no extra handle/context
  needed at the call site.
*/

#ifndef VL53L1X_PLATFORM_H
#define VL53L1X_PLATFORM_H

#include <stdint.h>
#include "driver/i2c_master.h"

typedef uint8_t VL53L1X_ERROR;

// Call once at startup with an already-initialized I2C bus (see
// i2c_new_master_bus() in main.c) before touching any VL53L1X function.
void vl53l1x_platform_init_bus(i2c_master_bus_handle_t bus);

// Directly usable helper for the one-off "assign a new I2C address"
// register write during XSHUT bring-up (register 0x0001 on the VL53L1X
// holds its own 7-bit I2C address) - see main.c's sensor bring-up sequence.
VL53L1X_ERROR vl53l1x_platform_set_address(uint16_t dev_old_addr, uint8_t new_addr);

// ---- Functions VL53L1X_api.c calls directly ----
VL53L1X_ERROR VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1X_ERROR VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1X_ERROR VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data);
VL53L1X_ERROR VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data);
VL53L1X_ERROR VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data);
VL53L1X_ERROR VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata);
VL53L1X_ERROR VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata);
VL53L1X_ERROR VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata);
VL53L1X_ERROR VL53L1_WaitMs(uint16_t dev, int32_t wait_ms);

#endif // VL53L1X_PLATFORM_H
