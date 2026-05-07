#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "driver/pulse_cnt.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"

#include "img_converters.h"

static const char *TAG = "B0462_ONE_TEST";

/* Wi-Fi */
#define WIFI_SSID      "espwifi"
#define WIFI_PASS      "12345678"

/* B0462 GC2145 pin mapping */
#define CAM_PIN_SIOD      GPIO_NUM_4
#define CAM_PIN_SIOC      GPIO_NUM_5

#define CAM_PIN_D0        GPIO_NUM_6
#define CAM_PIN_D1        GPIO_NUM_7
#define CAM_PIN_D2        GPIO_NUM_8
#define CAM_PIN_D3        GPIO_NUM_9
#define CAM_PIN_D4        GPIO_NUM_10
#define CAM_PIN_D5        GPIO_NUM_11
#define CAM_PIN_D6        GPIO_NUM_12
#define CAM_PIN_D7        GPIO_NUM_13

#define CAM_PIN_PWDN      GPIO_NUM_14
#define CAM_PIN_HREF      GPIO_NUM_15
#define CAM_PIN_VSYNC     GPIO_NUM_16
#define CAM_PIN_POWER_EN  GPIO_NUM_17
#define CAM_PIN_PCLK      GPIO_NUM_18
#define CAM_PIN_XCLK      GPIO_NUM_21
#define CAM_PIN_RESET     -1

static int measure_pulse_count(gpio_num_t pin, int duration_us);

/* GC2145 7-bit SCCB/I2C address */
#define GC2145_ADDR       0x3C

/* Test mode */
#define TEST_MODE_NAME    "SVGA_10MHz_RGB565_REVERSED_D_BUS_COLORBAR"
#define TEST_XCLK_HZ      10000000
#define TEST_FRAME_SIZE   FRAMESIZE_SVGA

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/* ---------------------------------------------------------
   Power control
   --------------------------------------------------------- */

static void scan_camera_signal_pins(void)
{
    gpio_num_t pins[] = {
        CAM_PIN_D0,
        CAM_PIN_D1,
        CAM_PIN_D2,
        CAM_PIN_D3,
        CAM_PIN_D4,
        CAM_PIN_D5,
        CAM_PIN_D6,
        CAM_PIN_D7,
        CAM_PIN_HREF,
        CAM_PIN_VSYNC,
        CAM_PIN_PCLK,
        CAM_PIN_XCLK
    };

    const char *names[] = {
        "D0",
        "D1",
        "D2",
        "D3",
        "D4",
        "D5",
        "D6",
        "D7",
        "HREF",
        "VSYNC",
        "PCLK",
        "XCLK"
    };

    ESP_LOGI(TAG, "========== CAMERA PIN PULSE SCAN ==========");

    for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        int count = measure_pulse_count(pins[i], 1000);

        ESP_LOGI(TAG, "%s GPIO%d count in 1 ms = %d",
                 names[i],
                 pins[i],
                 count);
    }

    ESP_LOGI(TAG, "========== CAMERA PIN PULSE SCAN END ==========");
}

static void camera_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CAM_PIN_POWER_EN) | (1ULL << CAM_PIN_PWDN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void camera_power_off(void)
{
    gpio_set_level(CAM_PIN_POWER_EN, 0);
    gpio_set_level(CAM_PIN_PWDN, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
}

static void camera_power_on(void)
{
    gpio_set_level(CAM_PIN_POWER_EN, 1);
    gpio_set_level(CAM_PIN_PWDN, 0);
    vTaskDelay(pdMS_TO_TICKS(1500));
}

static void camera_power_cycle(void)
{
    camera_power_off();
    camera_power_on();
}

/* ---------------------------------------------------------
   Manual XCLK for direct SCCB test
   --------------------------------------------------------- */

static esp_err_t xclk_start_manual(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .freq_hz = TEST_XCLK_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Manual XCLK timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t ledc_channel = {
        .gpio_num = CAM_PIN_XCLK,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1,
        .hpoint = 0,
    };

    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Manual XCLK channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Manual XCLK started on GPIO%d at %d Hz",
             CAM_PIN_XCLK,
             TEST_XCLK_HZ);

    return ESP_OK;
}

static void xclk_stop_manual(void)
{
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Manual XCLK stopped");
}

/* ---------------------------------------------------------
   Direct SCCB/I2C register access
   --------------------------------------------------------- */

static esp_err_t direct_gc2145_read_reg(i2c_master_dev_handle_t dev,
                                        uint8_t reg,
                                        uint8_t *value)
{
    return i2c_master_transmit_receive(
        dev,
        &reg,
        1,
        value,
        1,
        100
    );
}

static esp_err_t direct_gc2145_write_reg(i2c_master_dev_handle_t dev,
                                         uint8_t reg,
                                         uint8_t value)
{
    uint8_t data[2] = { reg, value };

    return i2c_master_transmit(
        dev,
        data,
        sizeof(data),
        100
    );
}

static esp_err_t direct_sccb_precheck(void)
{
    ESP_LOGI(TAG, "========== DIRECT SCCB PRECHECK START ==========");

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CAM_PIN_SIOD,
        .scl_io_num = CAM_PIN_SIOC,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle = NULL;

    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GC2145_ADDR,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle = NULL;

    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
        i2c_del_master_bus(bus_handle);
        return ret;
    }

    ret = i2c_master_probe(bus_handle, GC2145_ADDR, 100);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DIRECT SCCB: Device ACK at 0x%02X", GC2145_ADDR);
    } else {
        ESP_LOGE(TAG,
                 "DIRECT SCCB: Device probe failed at 0x%02X: %s",
                 GC2145_ADDR,
                 esp_err_to_name(ret));
    }

    uint8_t pid_h = 0;
    uint8_t pid_l = 0;

    ret = direct_gc2145_read_reg(dev_handle, 0xF0, &pid_h);
    ESP_LOGI(TAG,
             "DIRECT SCCB: read 0xF0 ret=%s value=0x%02X",
             esp_err_to_name(ret),
             pid_h);

    ret = direct_gc2145_read_reg(dev_handle, 0xF1, &pid_l);
    ESP_LOGI(TAG,
             "DIRECT SCCB: read 0xF1 ret=%s value=0x%02X",
             esp_err_to_name(ret),
             pid_l);

    uint8_t reg06_before = 0;
    ret = direct_gc2145_read_reg(dev_handle, 0x06, &reg06_before);
    ESP_LOGI(TAG,
             "DIRECT SCCB: read 0x06 before ret=%s value=0x%02X",
             esp_err_to_name(ret),
             reg06_before);

    ret = direct_gc2145_write_reg(dev_handle, 0x06, 0x3B);
    ESP_LOGI(TAG,
             "DIRECT SCCB: write 0x06=0x3B ret=%s",
             esp_err_to_name(ret));

    uint8_t reg06_after = 0;
    ret = direct_gc2145_read_reg(dev_handle, 0x06, &reg06_after);
    ESP_LOGI(TAG,
             "DIRECT SCCB: read 0x06 after ret=%s value=0x%02X",
             esp_err_to_name(ret),
             reg06_after);

    i2c_master_bus_rm_device(dev_handle);
    i2c_del_master_bus(bus_handle);

    ESP_LOGI(TAG, "========== DIRECT SCCB PRECHECK END ==========");
    return ESP_OK;
}

/* ---------------------------------------------------------
   esp32-camera init
   --------------------------------------------------------- */

static esp_err_t camera_init_esp32_camera(void)
{
    camera_config_t config = {
        .pin_pwdn  = -1,
        .pin_reset = -1,

        .pin_xclk = CAM_PIN_XCLK,

        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        /*
         * IMPORTANT:
         * Reversed D0-D7 software mapping.
         * Keep physical wiring same.
         */
        .pin_d7 = CAM_PIN_D0,
        .pin_d6 = CAM_PIN_D1,
        .pin_d5 = CAM_PIN_D2,
        .pin_d4 = CAM_PIN_D3,
        .pin_d3 = CAM_PIN_D4,
        .pin_d2 = CAM_PIN_D5,
        .pin_d1 = CAM_PIN_D6,
        .pin_d0 = CAM_PIN_D7,

        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href  = CAM_PIN_HREF,
        .pin_pclk  = CAM_PIN_PCLK,

        .xclk_freq_hz = TEST_XCLK_HZ,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_RGB565,
        .frame_size   = TEST_FRAME_SIZE,

        .jpeg_quality = 12,
        .fb_count     = 1,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    };

    ESP_LOGI(TAG, "========== ESP32-CAMERA INIT START ==========");
    ESP_LOGI(TAG, "Mode: %s", TEST_MODE_NAME);
    ESP_LOGI(TAG, "XCLK: %d Hz", TEST_XCLK_HZ);

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "esp_camera_init failed: %s / 0x%x",
                 esp_err_to_name(err),
                 err);

        ESP_LOGI(TAG, "========== ESP32-CAMERA INIT END FAILED ==========");
        return err;
    }

    ESP_LOGI(TAG, "esp_camera_init returned ESP_OK");

    sensor_t *s = esp_camera_sensor_get();

    if (s) {
        ESP_LOGI(TAG, "Sensor PID from driver: 0x%04X", s->id.PID);

        int pid_h = s->get_reg(s, 0xF0, 0xFF);
        int pid_l = s->get_reg(s, 0xF1, 0xFF);
        int reg06 = s->get_reg(s, 0x06, 0xFF);
        int reg0b = s->get_reg(s, 0x0B, 0xFF);

        ESP_LOGI(TAG, "POST INIT: reg 0xF0 = 0x%02X", pid_h);
        ESP_LOGI(TAG, "POST INIT: reg 0xF1 = 0x%02X", pid_l);
        ESP_LOGI(TAG, "POST INIT: reg 0x06 = 0x%02X", reg06);
        ESP_LOGI(TAG, "POST INIT: reg 0x0B = 0x%02X", reg0b);

        /*
         * COLORBAR TEST:
         * If this shows clean color bars, D0-D7 bus is okay.
         * Later set this to 0 or remove it for normal camera image.
         */
        if (s->set_colorbar) {
            int cb_ret = s->set_colorbar(s, 1);
            ESP_LOGI(TAG, "GC2145 colorbar enable result = %d", cb_ret);
        } else {
            ESP_LOGW(TAG, "set_colorbar function not available");
        }

    } else {
        ESP_LOGE(TAG, "esp_camera_sensor_get returned NULL");
    }

    ESP_LOGI(TAG, "========== ESP32-CAMERA INIT END OK ==========");
    return ESP_OK;
}

/* ---------------------------------------------------------
   Real frame test
   --------------------------------------------------------- */

static esp_err_t test_capture_once(void)
{
    ESP_LOGI(TAG, "========== REAL FRAME CAPTURE TEST START ==========");

    camera_fb_t *fb = NULL;

    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "Test capture attempt %d", i + 1);

        fb = esp_camera_fb_get();

        if (fb) {
            ESP_LOGI(TAG, "TEST FRAME OK");
            ESP_LOGI(TAG, "Frame length = %u bytes", fb->len);
            ESP_LOGI(TAG, "Frame width  = %u", fb->width);
            ESP_LOGI(TAG, "Frame height = %u", fb->height);
            ESP_LOGI(TAG, "Frame format = %d", fb->format);

            esp_camera_fb_return(fb);

            ESP_LOGI(TAG, "========== REAL FRAME CAPTURE TEST END OK ==========");
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Test frame failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGE(TAG, "REAL FRAME CAPTURE FAILED");
    ESP_LOGI(TAG, "========== REAL FRAME CAPTURE TEST END FAILED ==========");
    return ESP_FAIL;
}

/* ---------------------------------------------------------
   Web server
   --------------------------------------------------------- */

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ESP_LOGE(TAG, "Web capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;

    bool jpeg_ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);

    esp_camera_fb_return(fb);

    if (!jpeg_ok || jpg_buf == NULL) {
        ESP_LOGE(TAG, "RGB565 to JPEG conversion failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req,
                       "Content-Disposition",
                       "inline; filename=capture.jpg");

    esp_err_t res = httpd_resp_send(req,
                                    (const char *)jpg_buf,
                                    jpg_len);

    free(jpg_buf);

    ESP_LOGI(TAG, "JPEG sent: %u bytes", jpg_len);
    return res;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    char html[1000];

    snprintf(html, sizeof(html),
        "<!DOCTYPE html>"
        "<html>"
        "<head><title>B0462 GC2145</title></head>"
        "<body style='font-family:Arial;text-align:center;'>"
        "<h2>ESP32-S3 + B0462 GC2145</h2>"
        "<p><b>Mode:</b> %s</p>"
        "<img src='/capture?ts=%lu' style='width:320px;height:240px;border:1px solid #333;'>"
        "<br><br>"
        "<button onclick='location.reload()'>Refresh</button>"
        "<br><br>"
        "<a href='/capture'>Open raw capture</a>"
        "</body>"
        "</html>",
        TEST_MODE_NAME,
        (unsigned long)xTaskGetTickCount()
    );

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = index_handler,
            .user_ctx = NULL
        };

        httpd_uri_t capture_uri = {
            .uri = "/capture",
            .method = HTTP_GET,
            .handler = capture_handler,
            .user_ctx = NULL
        };

        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &capture_uri);

        ESP_LOGI(TAG, "Web server started");
    } else {
        ESP_LOGE(TAG, "Web server start failed");
    }
}

/* ---------------------------------------------------------
   Wi-Fi
   --------------------------------------------------------- */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, reconnecting...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        ESP_LOGI(TAG,
                 "Got IP: " IPSTR,
                 IP2STR(&event->ip_info.ip));

        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL
    ));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        NULL
    ));

    wifi_config_t wifi_config = {0};

    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi...");

    xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );
}

/* ---------------------------------------------------------
   Pulse counter
   --------------------------------------------------------- */

static int measure_pulse_count(gpio_num_t pin, int duration_us)
{
    pcnt_unit_handle_t unit = NULL;
    pcnt_channel_handle_t chan = NULL;

    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };

    esp_err_t ret = pcnt_new_unit(&unit_config, &unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "PCNT unit create failed for GPIO%d: %s",
                 pin,
                 esp_err_to_name(ret));
        return -1;
    }

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = pin,
        .level_gpio_num = -1,
    };

    ret = pcnt_new_channel(unit, &chan_config, &chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "PCNT channel create failed for GPIO%d: %s",
                 pin,
                 esp_err_to_name(ret));

        pcnt_del_unit(unit);
        return -1;
    }

    pcnt_channel_set_edge_action(
        chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD
    );

    pcnt_channel_set_level_action(
        chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP
    );

    pcnt_unit_enable(unit);
    pcnt_unit_clear_count(unit);
    pcnt_unit_start(unit);

    esp_rom_delay_us(duration_us);

    pcnt_unit_stop(unit);

    int count = 0;
    pcnt_unit_get_count(unit, &count);

    pcnt_unit_disable(unit);
    pcnt_del_channel(chan);
    pcnt_del_unit(unit);

    return count;
}

static void software_signal_check(void)
{
    ESP_LOGI(TAG, "========== SOFTWARE SIGNAL CHECK ==========");

    int pclk_count  = measure_pulse_count(CAM_PIN_PCLK, 1000);
    int href_count  = measure_pulse_count(CAM_PIN_HREF, 1000000);
    int vsync_count = measure_pulse_count(CAM_PIN_VSYNC, 1000000);

    ESP_LOGI(TAG,
             "PCLK  GPIO%d count in 1 ms  = %d",
             CAM_PIN_PCLK,
             pclk_count);

    ESP_LOGI(TAG,
             "HREF  GPIO%d count in 1 sec = %d",
             CAM_PIN_HREF,
             href_count);

    ESP_LOGI(TAG,
             "VSYNC GPIO%d count in 1 sec = %d",
             CAM_PIN_VSYNC,
             vsync_count);

    if (pclk_count <= 0) {
        ESP_LOGE(TAG, "PCLK missing. Camera is not streaming pixel clock.");
    } else if (href_count <= 0 || vsync_count <= 0) {
        ESP_LOGE(TAG,
                 "PCLK exists, but HREF/VSYNC missing. Check HREF/VSYNC wiring or sensor timing.");
    } else {
        ESP_LOGI(TAG,
                 "PCLK/HREF/VSYNC are present. If capture still fails, suspect DVP pin mapping/DMA timing.");
    }

    ESP_LOGI(TAG, "========== SOFTWARE SIGNAL CHECK END ==========");
}

/* ---------------------------------------------------------
   Main
   --------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "B0462 GC2145 one-mode debug start");

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    camera_gpio_init();

    camera_power_cycle();

    if (xclk_start_manual() == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(300));
        direct_sccb_precheck();
        xclk_stop_manual();
    }

    camera_power_cycle();

    if (camera_init_esp32_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Driver init failed. Compare DIRECT SCCB result above.");
        ESP_LOGE(TAG, "If direct write 0x06=0x3B worked but driver write failed, patch esp32-camera gc2145.c sequence.");

        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    software_signal_check();
    scan_camera_signal_pins();

    if (test_capture_once() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init may be OK, but no frame reached ESP32.");
        ESP_LOGE(TAG, "Check PCLK GPIO18, VSYNC GPIO16, HREF GPIO15 with DSO.");

        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    wifi_init_sta();

    start_webserver();

    ESP_LOGI(TAG, "Open the shown IP address in browser");
    ESP_LOGI(TAG, "Mode: %s", TEST_MODE_NAME);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}