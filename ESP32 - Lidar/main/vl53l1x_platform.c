#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l1x_platform.h"

#define VL53L1X_MAX_DEVICES 4    // 3 sensors + a little headroom
#define I2C_XFER_TIMEOUT_MS 100
#define I2C_DEVICE_SPEED_HZ 100000 // dropped from 400kHz while debugging the timeout - raise back once it's reliable

typedef struct {
    uint16_t addr;
    i2c_master_dev_handle_t handle;
} device_entry_t;

static i2c_master_bus_handle_t s_bus = NULL;
static device_entry_t s_devices[VL53L1X_MAX_DEVICES];
static uint8_t s_device_count = 0;

void vl53l1x_platform_init_bus(i2c_master_bus_handle_t bus) {
    s_bus = bus;
    s_device_count = 0;
}

// Finds (or lazily creates) the i2c_master device handle for a given
// 7-bit address. Handles are cached and reused for the lifetime of the
// program - safe even though the same default address (0x29) is reused
// in turn by each sensor during bring-up, since only one physical chip
// is ever awake at that address at a time.
#define VL53L1X_FACTORY_DEFAULT_ADDR 0x29

static i2c_master_dev_handle_t get_handle(uint16_t addr) {
    if (addr == VL53L1X_FACTORY_DEFAULT_ADDR) {
        // A different physical sensor answers at the factory-default
        // address each time during bring-up, so any handle cached from a
        // PREVIOUS sensor's turn at this address must never be reused -
        // tear it down and let a fresh one get created below.
        for (uint8_t i = 0; i < s_device_count; i++) {
            if (s_devices[i].addr == VL53L1X_FACTORY_DEFAULT_ADDR) {
                i2c_master_bus_rm_device(s_devices[i].handle);
                for (uint8_t j = i; j < s_device_count - 1; j++) {
                    s_devices[j] = s_devices[j + 1];
                }
                s_device_count--;
                break;
            }
        }
    }

    for (uint8_t i = 0; i < s_device_count; i++) {
        if (s_devices[i].addr == addr) return s_devices[i].handle;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
    };
    i2c_master_dev_handle_t handle = NULL;
    i2c_master_bus_add_device(s_bus, &dev_cfg, &handle);

    if (s_device_count < VL53L1X_MAX_DEVICES) {
        s_devices[s_device_count].addr = addr;
        s_devices[s_device_count].handle = handle;
        s_device_count++;
    }
    return handle;
}

VL53L1X_ERROR vl53l1x_platform_set_address(uint16_t dev_old_addr, uint8_t new_addr) {
    // Register 0x0001 (I2C_SLAVE__DEVICE_ADDRESS) holds the sensor's own
    // 7-bit I2C address directly - writing it here is what actually moves
    // the sensor off the factory-default 0x29 during XSHUT bring-up.
    return VL53L1_WrByte(dev_old_addr, 0x0001, new_addr);
}

VL53L1X_ERROR VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    i2c_master_dev_handle_t handle = get_handle(dev);
    uint8_t buf[2 + 256];
    if (count > 256) return 1; // guard against anything larger than we've sized for
    buf[0] = (uint8_t)(index >> 8);
    buf[1] = (uint8_t)(index & 0xFF);
    memcpy(&buf[2], pdata, count);
    esp_err_t err = i2c_master_transmit(handle, buf, 2 + count, I2C_XFER_TIMEOUT_MS);
    return (err == ESP_OK) ? 0 : 1;
}

VL53L1X_ERROR VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    i2c_master_dev_handle_t handle = get_handle(dev);
    uint8_t idx_buf[2] = { (uint8_t)(index >> 8), (uint8_t)(index & 0xFF) };
    esp_err_t err = i2c_master_transmit_receive(handle, idx_buf, 2, pdata, count, I2C_XFER_TIMEOUT_MS);
    return (err == ESP_OK) ? 0 : 1;
}

VL53L1X_ERROR VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

VL53L1X_ERROR VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return VL53L1_WriteMulti(dev, index, buf, 2);
}

VL53L1X_ERROR VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t buf[4] = {
        (uint8_t)(data >> 24), (uint8_t)(data >> 16),
        (uint8_t)(data >> 8), (uint8_t)(data & 0xFF)
    };
    return VL53L1_WriteMulti(dev, index, buf, 4);
}

VL53L1X_ERROR VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata) {
    return VL53L1_ReadMulti(dev, index, pdata, 1);
}

VL53L1X_ERROR VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata) {
    uint8_t buf[2];
    VL53L1X_ERROR status = VL53L1_ReadMulti(dev, index, buf, 2);
    *pdata = ((uint16_t)buf[0] << 8) | buf[1];
    return status;
}

VL53L1X_ERROR VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata) {
    uint8_t buf[4];
    VL53L1X_ERROR status = VL53L1_ReadMulti(dev, index, buf, 4);
    *pdata = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
             ((uint32_t)buf[2] << 8) | buf[3];
    return status;
}

VL53L1X_ERROR VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    (void)dev;
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    return 0;
}
