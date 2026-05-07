/*
 * SCCB (I2C-like) driver with the ESP-IDF 5.x new I2C API.
 * Patched for B0462 / GC2145 bring-up on ESP32-S3.
 */
#include <stdbool.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sccb.h"
#include "sensor.h"
#include <stdio.h>
#include "sdkconfig.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "sccb-ng";
#endif

#define LITTLETOBIG(x) ((x << 8) | (x >> 8))

#if (ESP_IDF_VERSION_MAJOR <= 5)
#include "esp_private/i2c_platform.h"
#endif

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define TIMEOUT_MS 1000
#define SCCB_FREQ CONFIG_SCCB_CLK_FREQ

#if CONFIG_SCCB_HARDWARE_I2C_PORT1
const int SCCB_I2C_PORT_DEFAULT = 1;
#else
const int SCCB_I2C_PORT_DEFAULT = 0;
#endif

#define MAX_DEVICES (UINT8_MAX - 1)
#define SCCB_RETRY_COUNT 5
#define SCCB_RETRY_DELAY_MS 5

typedef struct {
    i2c_master_dev_handle_t dev_handle;
    uint16_t address;
} device_t;

static device_t devices[MAX_DEVICES];
static uint8_t device_count = 0;
static int sccb_i2c_port;
static bool sccb_owns_i2c_port;

static i2c_master_dev_handle_t get_handle_from_address(uint8_t slv_addr)
{
    for (uint8_t i = 0; i < device_count; i++) {
        if (slv_addr == devices[i].address) {
            return devices[i].dev_handle;
        }
    }

    ESP_LOGE(TAG, "Device with address 0x%02x not found", slv_addr);
    return NULL;
}

static bool device_already_installed(uint8_t slv_addr)
{
    for (uint8_t i = 0; i < device_count; i++) {
        if (slv_addr == devices[i].address && devices[i].dev_handle != NULL) {
            return true;
        }
    }
    return false;
}

int SCCB_Install_Device(uint8_t slv_addr)
{
    esp_err_t ret;
    i2c_master_bus_handle_t bus_handle;

    if (device_already_installed(slv_addr)) {
        return 0;
    }

    if (device_count >= MAX_DEVICES) {
        ESP_LOGE(TAG, "cannot add more than %d devices", MAX_DEVICES);
        return ESP_FAIL;
    }

    ret = i2c_master_get_bus_handle(sccb_i2c_port, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to get SCCB I2C bus handle for port %d: %s",
                 sccb_i2c_port, esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = slv_addr,
        .scl_speed_hz = SCCB_FREQ,
    };

    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &devices[device_count].dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to install SCCB I2C device 0x%02x: %s",
                 slv_addr, esp_err_to_name(ret));
        return -1;
    }

    devices[device_count].address = slv_addr;
    device_count++;
    return 0;
}

int SCCB_Init(int pin_sda, int pin_scl)
{
    ESP_LOGI(TAG, "pin_sda %d pin_scl %d", pin_sda, pin_scl);

    esp_err_t ret;
    memset(devices, 0, sizeof(devices));
    device_count = 0;

    sccb_i2c_port = SCCB_I2C_PORT_DEFAULT;
    sccb_owns_i2c_port = true;
    ESP_LOGI(TAG, "sccb_i2c_port=%d freq=%d", sccb_i2c_port, SCCB_FREQ);

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = SCCB_I2C_PORT_DEFAULT,
        .scl_io_num = pin_scl,
        .sda_io_num = pin_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = 1,
    };

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to install SCCB I2C master bus on port %d: %s",
                 sccb_i2c_port, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

int SCCB_Use_Port(int i2c_num)
{
    if (sccb_owns_i2c_port) {
        SCCB_Deinit();
    }
    if (i2c_num < 0 || i2c_num > I2C_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    sccb_i2c_port = i2c_num;
    sccb_owns_i2c_port = false;
    return ESP_OK;
}

int SCCB_Deinit(void)
{
    esp_err_t ret;

    for (uint8_t i = 0; i < device_count; i++) {
        if (devices[i].dev_handle != NULL) {
            ret = i2c_master_bus_rm_device(devices[i].dev_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "failed to remove SCCB I2C device 0x%02x: %s",
                         devices[i].address, esp_err_to_name(ret));
                return ret;
            }
        }
        devices[i].dev_handle = NULL;
        devices[i].address = 0;
    }
    device_count = 0;

    if (!sccb_owns_i2c_port) {
        return ESP_OK;
    }
    sccb_owns_i2c_port = false;

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_master_get_bus_handle(sccb_i2c_port, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to get SCCB I2C bus handle for port %d: %s",
                 sccb_i2c_port, esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_del_master_bus(bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to delete SCCB I2C master bus at port %d: %s",
                 sccb_i2c_port, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

int SCCB_Probe(uint8_t slv_addr)
{
    esp_err_t ret;
    i2c_master_bus_handle_t bus_handle;

    ret = i2c_master_get_bus_handle(sccb_i2c_port, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to get SCCB I2C bus handle for port %d: %s",
                 sccb_i2c_port, esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_master_probe(bus_handle, slv_addr, TIMEOUT_MS);
    if (ret == ESP_OK) {
        return SCCB_Install_Device(slv_addr);
    }

    return ret;
}

int SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    i2c_master_dev_handle_t dev_handle = get_handle_from_address(slv_addr);
    if (dev_handle == NULL) {
        return -1;
    }

    uint8_t rx_buffer[1] = {0};
    esp_err_t ret = ESP_FAIL;

    for (int i = 0; i < SCCB_RETRY_COUNT; i++) {
        ret = i2c_master_transmit_receive(dev_handle, &reg, 1, rx_buffer, 1, TIMEOUT_MS);
        if (ret == ESP_OK) {
            return rx_buffer[0];
        }
        vTaskDelay(pdMS_TO_TICKS(SCCB_RETRY_DELAY_MS));
    }

    ESP_LOGE(TAG, "SCCB_Read failed addr:0x%02x reg:0x%02x ret:%d(%s)",
             slv_addr, reg, ret, esp_err_to_name(ret));
    return -1;
}

int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    i2c_master_dev_handle_t dev_handle = get_handle_from_address(slv_addr);
    if (dev_handle == NULL) {
        return -1;
    }

    uint8_t tx_buffer[2] = { reg, data };
    esp_err_t ret = ESP_FAIL;

    for (int i = 0; i < SCCB_RETRY_COUNT; i++) {
        ret = i2c_master_transmit(dev_handle, tx_buffer, sizeof(tx_buffer), TIMEOUT_MS);
        if (ret == ESP_OK) {
            return 0;
        }
        vTaskDelay(pdMS_TO_TICKS(SCCB_RETRY_DELAY_MS));
    }

    ESP_LOGE(TAG, "SCCB_Write failed addr:0x%02x reg:0x%02x data:0x%02x ret:%d(%s)",
             slv_addr, reg, data, ret, esp_err_to_name(ret));
    return -1;
}

int SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{
    i2c_master_dev_handle_t dev_handle = get_handle_from_address(slv_addr);
    if (dev_handle == NULL) {
        return -1;
    }

    uint8_t rx_buffer[1] = {0};
    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    esp_err_t ret = ESP_FAIL;

    for (int i = 0; i < SCCB_RETRY_COUNT; i++) {
        ret = i2c_master_transmit_receive(dev_handle, reg_u8, 2, rx_buffer, 1, TIMEOUT_MS);
        if (ret == ESP_OK) {
            return rx_buffer[0];
        }
        vTaskDelay(pdMS_TO_TICKS(SCCB_RETRY_DELAY_MS));
    }

    ESP_LOGE(TAG, "SCCB_Read16 failed addr:0x%02x reg:0x%04x ret:%d(%s)",
             slv_addr, reg, ret, esp_err_to_name(ret));
    return -1;
}

int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    i2c_master_dev_handle_t dev_handle = get_handle_from_address(slv_addr);
    if (dev_handle == NULL) {
        return -1;
    }

    uint8_t tx_buffer[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0x00ff), data };
    esp_err_t ret = ESP_FAIL;

    for (int i = 0; i < SCCB_RETRY_COUNT; i++) {
        ret = i2c_master_transmit(dev_handle, tx_buffer, sizeof(tx_buffer), TIMEOUT_MS);
        if (ret == ESP_OK) {
            return 0;
        }
        vTaskDelay(pdMS_TO_TICKS(SCCB_RETRY_DELAY_MS));
    }

    ESP_LOGE(TAG, "SCCB_Write16 failed addr:0x%02x reg:0x%04x data:0x%02x ret:%d(%s)",
             slv_addr, reg, data, ret, esp_err_to_name(ret));
    return -1;
}

uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg)
{
    i2c_master_dev_handle_t dev_handle = get_handle_from_address(slv_addr);
    if (dev_handle == NULL) {
        return 0xffff;
    }

    uint8_t rx_buffer[2] = {0};
    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    esp_err_t ret = ESP_FAIL;

    for (int i = 0; i < SCCB_RETRY_COUNT; i++) {
        ret = i2c_master_transmit_receive(dev_handle, reg_u8, 2, rx_buffer, 2, TIMEOUT_MS);
        if (ret == ESP_OK) {
            return ((uint16_t)rx_buffer[0] << 8) | (uint16_t)rx_buffer[1];
        }
        vTaskDelay(pdMS_TO_TICKS(SCCB_RETRY_DELAY_MS));
    }

    ESP_LOGE(TAG, "SCCB_Read_Addr16_Val16 failed addr:0x%02x reg:0x%04x ret:%d(%s)",
             slv_addr, reg, ret, esp_err_to_name(ret));
    return 0xffff;
}

int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data)
{
    i2c_master_dev_handle_t dev_handle = get_handle_from_address(slv_addr);
    if (dev_handle == NULL) {
        return -1;
    }

    uint8_t tx_buffer[4] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0x00ff),
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0x00ff),
    };
    esp_err_t ret = ESP_FAIL;

    for (int i = 0; i < SCCB_RETRY_COUNT; i++) {
        ret = i2c_master_transmit(dev_handle, tx_buffer, sizeof(tx_buffer), TIMEOUT_MS);
        if (ret == ESP_OK) {
            return 0;
        }
        vTaskDelay(pdMS_TO_TICKS(SCCB_RETRY_DELAY_MS));
    }

    ESP_LOGE(TAG, "SCCB_Write_Addr16_Val16 failed addr:0x%02x reg:0x%04x data:0x%04x ret:%d(%s)",
             slv_addr, reg, data, ret, esp_err_to_name(ret));
    return -1;
}
