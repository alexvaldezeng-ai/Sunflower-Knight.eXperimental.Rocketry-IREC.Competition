#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "hal/adc_types.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

//SPI Pins
#define PIN_MOSI    GPIO_NUM_11
#define PIN_MISO    GPIO_NUM_13
#define PIN_SCLK    GPIO_NUM_12
#define PIN_CS_SD   GPIO_NUM_10
#define PIN_CS_IMU  GPIO_NUM_9

//Motor Pins
#define PIN_STEP1
#define PIN_STEP1_DIR
#define PIN_STEP1_ENABLE
#define PIN_STEP2
#define PIN_STEP2_DIR
#define PIN_STEP2_ENABLE
#define PIN_MOTOR_1A
#define PIN_MOTOR_1B
#define PIN_ESC GPIO_NUM_36

typedef enum {
  STATE_INIT, //Hardware bring up
  STATE_IDLE, //Wait for G force trigger
  STATE_DEPLOY, //Being deployed wait
  STATE_TRACKING, // Final tracking state
  STATE_FAULT
} State_t;

typedef struct {
  float accel_x, accel_y, accel_z; //MAX accel 90 m/s^2 9.35 m/s touch down 25.4s apogee
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
} IMUData_t;

typedef struct { 
  uint16_t north, south, east, west;
  bool charging; //Solar panel charge pin
} SunPos_t;

typedef struct {
  State_t state;
  IMUData_t imu;
  SunPos_t sun;
  float panel_angle_deg;
  uint32_t state_entry_ms;
  bool sd_healthy;
  bool imu_healthy;
  esp_err_t ret;
} PayloadHandle_t;

typedef struct {
  spi_device_handle_t imu;
  adc_oneshot_unit_handle_t adc1;
  sdmmc_card_t *sd_card;
} PayloadHardware_t;

static const char *TAG = "MAIN";

static void init_spi();
static void init_spi_sd(PayloadHardware_t *hardware);
static void init_spi_imu(PayloadHardware_t *hardware);
static void init_adc(PayloadHardware_t *hardware);
static void init_pwm();
static void esc_set_pulse_us(uint32_t us);
static void stepper_step(gpio_num_t pin_step, gpio_num_t pin_dir, bool dir, uint32_t steps);

void app_main(void) { 
  PayloadHandle_t payload = {0};
  PayloadHardware_t hardware = {0};
  init_spi();
  init_spi_sd(&hardware);
  payload.sd_healthy = true;
  payload.imu_healthy = true;
  init_spi_imu(&hardware);
  init_adc(&hardware);
  init_pwm();
  esc_set_pulse_us(1000);
  payload.state = STATE_IDLE;
  ESP_LOGI(TAG, "Payload Initialized");
  while(1){
     
  }
}

static void init_spi(){
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI Bus Active");
}

static void init_spi_sd(PayloadHardware_t *hardware){
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = PIN_CS_SD;
  slot_config.host_id = SPI2_HOST;
  esp_vfs_fat_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 2,
  };
  esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &hardware->sd_card);
  ESP_LOGI(TAG, "SD Card Mounted");
} 

static void init_spi_imu(PayloadHardware_t *hardware){
  spi_device_interface_config_t dev_cfg = {
    .clock_speed_hz = 500000,
    .mode = 0,                  
    .spics_io_num = PIN_CS_IMU,
    .queue_size = 1,
  };
  spi_bus_add_device(SPI2_HOST, &dev_cfg, &hardware->imu);
  ESP_LOGI(TAG, "IMU Initialized");
}

static void init_adc(PayloadHardware_t *hardware){
  adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &hardware->adc1));

  adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(hardware->adc1, ADC_CHANNEL_4, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(hardware->adc1, ADC_CHANNEL_5, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(hardware->adc1, ADC_CHANNEL_6, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(hardware->adc1, ADC_CHANNEL_7, &config));
}

static void init_pwm(){
  ledc_timer_config_t timer_cfg = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0,
    .duty_resolution = 16,
    .freq_hz = 50,
    .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&timer_cfg);

  ledc_channel_config_t channel_cfg = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = PIN_ESC,
    .duty = 0,
   .hpoint = 0,
  };
  ledc_channel_config(&channel_cfg);
}

static void esc_set_pulse_us(uint32_t us) {
    uint32_t duty = (us * 65536) / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void stepper_step(gpio_num_t pin_step, gpio_num_t pin_dir, bool dir, uint32_t steps) {
    gpio_set_level(pin_dir, dir);
    for (uint32_t i = 0; i < steps; i++) {
        gpio_set_level(pin_step, 1);
        vTaskDelay(pdMS_TO_TICKS(1));  // adjust to taste
        gpio_set_level(pin_step, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
