#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "portmacro.h"
#include "sdmmc_cmd.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

// Wiring notes:
// IMU:     PS1 -> 3.3V, 
// SD Card: VCC -> 5V from esc ubec 
// Steppers: VMOT -> battery, SLP+RST -> 3.3V or 5V (keep driver awake)
// LDRs:    LDR to 3.3V, 100k to GND, midpoint to ADC (N=GPIO32 S=GPIO33 E=GPIO34 W=GPIO35)
// ESC:     signal wire -> GPIO 27
// SPI Pins (shared bus: IMU + SD card)
#define PIN_MOSI GPIO_NUM_23
#define PIN_MISO GPIO_NUM_19
#define PIN_SCLK GPIO_NUM_18
#define PIN_CS_SD GPIO_NUM_22
#define PIN_CS_IMU GPIO_NUM_21
#define PIN_INT GPIO_NUM_4
#define PIN_RESET GPIO_NUM_5
#define PIN_WAKE GPIO_NUM_2

// Motor Pins
#define PIN_STEP1 GPIO_NUM_13
#define PIN_STEP1_DIR GPIO_NUM_16
#define PIN_STEP1_ENABLE GPIO_NUM_14
#define PIN_STEP2 GPIO_NUM_17
#define PIN_STEP2_DIR GPIO_NUM_26
#define PIN_STEP2_ENABLE GPIO_NUM_25
#define PIN_ESC GPIO_NUM_27

// IMU RID
#define RID_ACCELEROMETER 0x01
#define RID_GYROSCOPE 0x02
#define RID_MAGNETOMETER 0x03
#define RID_LINEAR_ACCEL 0x04
#define RID_ROTATION_VECTOR 0x05
#define RID_GAME_ROT_VECTOR 0x08
#define RID_BASE_TIMESTAMP 0xFB
#define RID_TIMESTAMP_REBASE 0xFA

// Launch Conditions
#define SM_PERIOD_MS 20        // 50 Hz control loop
#define LAUNCH_ACCEL_MS2 40.0f // ~4 g: far above handling, half of boost
#define LAUNCH_DEBOUNCE_N 5    // 100 ms sustained
#define LANDED_G_LO 8.3f       // |a| window meaning "resting at 1 g"
#define LANDED_G_HI 11.3f
#define LANDED_STILL_MS 10000     // must be still this long
#define LANDED_LOCKOUT_MS 120000  // ignore "landed" until T+60 s
#define DEPLOY_FALLBACK_MS 300000 // T+5 min: deploy NO MATTER WHAT
#define IMU_STALE_US 200000       // sample older than 200 ms = stale
#define LIFT_STEPS 2000           // TODO: measure on the real mechanism
#define LIFT_PERIOD_US 1000       // 1 kHz step rate (1/8 microstep)

// Tracking defines
#define LDR_BRIGHTER_IS_HIGHER                                                 \
  1 // 1: LDR on top of divider (more light = more counts)
    // 0: LDR on bottom (more light = fewer counts)
#define ADC_CH_NORTH ADC_CHANNEL_4
#define ADC_CH_SOUTH ADC_CHANNEL_5
#define ADC_CH_EAST ADC_CHANNEL_6
#define ADC_CH_WEST ADC_CHANNEL_7
#define ADC_SAMPLES 8        // average per channel per update
#define DARK_SUM_COUNTS 400  // pair sum below this = night/deep shade: hold
#define ERR_DEADBAND 0.08f   // |normalized error| below this: aligned, hold
#define STEPS_PER_NUDGE 8    // small fixed correction per update
#define NUDGE_PERIOD_US 2000 // gentle 500 steps/s for fine moves
#define PITCH_CLEARANCE 200    // extra steps after lift to clear chassis
#define PITCH_STEP_MIN 0       // never go below clearance height
#define PITCH_STEP_MAX (800)
#define YAW_STEP_MIN (-1600)
#define YAW_STEP_MAX (1600)
#define ESC_NEUTRAL_US 1500
#define ESC_BURST_OFFSET 30 // 1560/1440 us: gentle blip (tune on bench!)
#define ESC_BURST_MS 150
static int32_t s_pitch_pos = 0; // cumulative steps since deployment

typedef enum {
  STATE_INIT,     // Hardware bring up
  STATE_IDLE,     // Wait for G force trigger
  STATE_DEPLOY,   // Being deployed wait
  STATE_TRACKING, // Final tracking state
  STATE_FAULT
} State_t;

typedef struct {
  // MAX accel 90 m/s^2 9.35 m/s touch down 25.4s apogee
  int64_t t_us;  // esp_timer_get_time() at packet read
  float quat[4]; // w, x, y, z  (rotation vector)
  float quat_accuracy_rad;
  float accel[3]; // m/s^2, body frame
  bool quat_valid;
  bool accel_valid;
  float lin_accel[3];
  bool lin_accel_valid;
} IMUData_t;

typedef struct {
  uint16_t north, south, east, west;
  bool charging; // Solar panel charge pin
} SunPos_t;

typedef struct {
  State_t state;
  IMUData_t imu;
  SunPos_t sun;
  float panel_angle_deg;
  uint32_t state_entry_ms;
  bool sd_healthy;
  bool imu_healthy;
  bool lift_deployed;
  esp_err_t ret;
} PayloadHandle_t;

typedef struct {
  spi_device_handle_t imu;
  adc_oneshot_unit_handle_t adc1;
  sdmmc_card_t *sd_card;
} PayloadHardware_t;

static SemaphoreHandle_t s_int_sem;
static void IRAM_ATTR bno085_int_isr(void *arg) {
  BaseType_t hp_woken = pdFALSE;
  xSemaphoreGiveFromISR(s_int_sem, &hp_woken);
  if (hp_woken) {
    portYIELD_FROM_ISR();
  }
}

static const char *TAG = "payload";

static QueueHandle_t s_latest_q;
static QueueHandle_t s_log_q;
static PayloadHardware_t hardware;
static PayloadHandle_t payload;

static inline float accel_mag(const IMUData_t *s);
static void enter_state(State_t next);
static inline uint32_t ms_in_state(void);
static void init_spi();
static void init_spi_sd(PayloadHardware_t *hardware);
static void init_spi_imu(PayloadHardware_t *hardware);
static esp_err_t imu_wait_int(TickType_t timeout);
static esp_err_t imu_write(const uint8_t *pkt, size_t len);
static esp_err_t imu_read(uint8_t *buf, size_t buf_len, size_t *out_len,
                          TickType_t timeout, spi_device_handle_t imu);
static esp_err_t imu_enable_report(uint8_t report_id, uint32_t interval_us);
static esp_err_t imu_reset();
static esp_err_t imu_start();
static int report_len(uint8_t id);
static inline int16_t le16(const uint8_t *p);
static inline float q_to_float(int16_t v, int q);
static bool imu_parse_packet(const uint8_t *buf, size_t len);
static void init_adc(PayloadHardware_t *hardware);
static int read_ldr(adc_channel_t ch);
static void pitch_nudge(bool dir);
static void roll_nudge(bool dir);
static void roll_align_panel_up(QueueHandle_t latest_q);
static void sun_track_update();
static void init_pwm();
static void esc_set_pulse_us(uint32_t us);
static void stepper_step(gpio_num_t pin_step, gpio_num_t pin_dir, bool dir,
                         uint32_t steps, uint32_t step_period_us);

void imu_task(void *pvParameters) {
  static uint8_t buf[2048];
  size_t len = {0};
  payload.imu = (IMUData_t){0};
  uint8_t dropped_samples = 0;
  int err_streak = 0;

  while (1) {
    esp_err_t rd = imu_read(buf, sizeof buf, &len, pdMS_TO_TICKS(100), hardware.imu);
    if (rd == ESP_ERR_NOT_FOUND) {
      continue;                     // empty SHTP read ("no data") — normal
    }
    if (rd == ESP_ERR_TIMEOUT) {
      static int tmo_n = 0;
      if (++tmo_n <= 5)
        ESP_LOGW(TAG, "IMU read timeout (%d)", tmo_n);
      continue;                     // no reset on plain timeouts
    }
    if (rd != ESP_OK) {
      err_streak++;
      if (err_streak < 5) {
        // Transient bus contention (e.g. FF FF FF FF after SD card
        // transaction) — retry without resetting the sensor.
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      ESP_LOGW(TAG, "IMU read error (%s) x%d — resetting sensor",
               esp_err_to_name(rd), err_streak);
      err_streak = 0;
      imu_start();
      continue;
    }
    err_streak = 0;

    // Log ch0 error reports (should be rare now) and ch2 control packets.
    if (buf[2] == 0) {
      static int ch0_count = 0;
      ch0_count++;
      char hex[64] = {0};
      int hlen = (len - 4 > 16) ? 16 : (int)(len - 4);
      for (int j = 0; j < hlen; j++)
        snprintf(hex + j * 3, sizeof(hex) - j * 3, "%02X ", buf[4 + j]);
      ESP_LOGW(TAG, "ch0 #%d (len=%u): %s", ch0_count, (unsigned)len, hex);
    } else if (buf[2] == 2) {
      ESP_LOGI(TAG, "ch2 pkt: len=%u id=0x%02X",
               (unsigned)len, (len > 4) ? buf[4] : 0xFF);
    }

    if (imu_parse_packet(buf, len)) {
      payload.imu.t_us = esp_timer_get_time();
      xQueueOverwrite(s_latest_q, &payload.imu);

      static int sd_div = 0;
      if (++sd_div >= 10) { // log at ~10 Hz, not 100 Hz
        sd_div = 0;
        if (xQueueSend(s_log_q, &payload.imu, 0) != pdTRUE) {
          dropped_samples++;
        }
      }

      // Throttled debug log: print every ~1 s (every 100th sample at 100 Hz)
      static int log_div = 0;
      if (++log_div >= 100) {
        log_div = 0;
        ESP_LOGI(TAG, "IMU  a=[%.2f %.2f %.2f] |a|=%.2f  q=[%.4f %.4f %.4f %.4f]  la=[%.2f %.2f %.2f]",
                 payload.imu.accel[0], payload.imu.accel[1], payload.imu.accel[2],
                 accel_mag(&payload.imu),
                 payload.imu.quat[0], payload.imu.quat[1], payload.imu.quat[2], payload.imu.quat[3],
                 payload.imu.lin_accel[0], payload.imu.lin_accel[1], payload.imu.lin_accel[2]);
      }
    }
  }
}

void log_task(void *pvParameters) {
  // Find next available log file number
  char path[32];
  int file_num = 0;
  struct stat st;
  while (file_num < 9999) {
    snprintf(path, sizeof path, "/sdcard/log_%04d.csv", file_num);
    if (stat(path, &st) != 0)
      break;
    file_num++;
  }

  FILE *f = fopen(path, "w");
  if (!f) {
    ESP_LOGE(TAG, "failed to open %s", path);
    payload.sd_healthy = false;
    vTaskDelete(NULL);
    return;
  }

  fprintf(f, "t_us,ax,ay,az,qw,qx,qy,qz,lax,lay,laz\n");
  ESP_LOGI(TAG, "logging to %s", path);

  IMUData_t s;
  int unflushed = 0;

  while (1) {
    if (xQueueReceive(s_log_q, &s, pdMS_TO_TICKS(1000)) == pdTRUE) {
      fprintf(f, "%lld,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n",
              s.t_us,
              s.accel[0], s.accel[1], s.accel[2],
              s.quat[0], s.quat[1], s.quat[2], s.quat[3],
              s.lin_accel[0], s.lin_accel[1], s.lin_accel[2]);
      unflushed++;

      if (unflushed >= 50) {
        fflush(f);
        unflushed = 0;
      }
    } else {
      // Queue empty for 1 second — flush whatever we have
      if (unflushed > 0) {
        fflush(f);
        unflushed = 0;
      }
    }
  }
}

void state_task(void *pvParameters) {
  IMUData_t s;
  TickType_t wake = xTaskGetTickCount();

  // debounce/track counters — all reset on every state change
  int launch_count = 0;
  uint32_t still_since_ms = 0; // 0 = "not currently still"
  int64_t launch_t_ms = 0;

  enter_state(STATE_TRACKING); // init already done in app_main

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SM_PERIOD_MS));

    // --- gather inputs once per cycle ---
    bool fresh = (xQueuePeek(s_latest_q, &s, 0) == pdTRUE) &&
                 (esp_timer_get_time() - s.t_us < IMU_STALE_US);
    float amag = fresh ? accel_mag(&s) : -1.0f;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    payload.imu_healthy = fresh;

    switch (payload.state) {

    case STATE_IDLE:
      // Launch detect: sustained |a| above threshold.
      // Stale IMU on the pad is not a fault — just keep waiting;
      // the IMU task is responsible for recovering the sensor.
      if (fresh && amag > LAUNCH_ACCEL_MS2) {
        if (++launch_count >= LAUNCH_DEBOUNCE_N) {
          launch_t_ms = now_ms;
          launch_count = 0;
          enter_state(STATE_DEPLOY);
        }
      } else {
        launch_count = 0; // any miss resets the debounce
      }
      break;

    case STATE_DEPLOY: { // in flight, waiting for landing
      uint32_t since_launch = now_ms - (uint32_t)launch_t_ms;

      // Hard fallback: if sensors lie or logic misses the landing,
      // deploy anyway at T+5 min. A payload that never deploys
      // scores zero; one that deploys on the ground a bit late is fine.
      if (since_launch > DEPLOY_FALLBACK_MS) {
        ESP_LOGW(TAG, "fallback timer deploy");
        enter_state(STATE_TRACKING);
        break;
      }

      // Landing detect: lockout elapsed AND |a| pinned near 1 g
      // continuously for LANDED_STILL_MS.
      if (since_launch > LANDED_LOCKOUT_MS && fresh && amag > LANDED_G_LO &&
          amag < LANDED_G_HI) {
        if (still_since_ms == 0)
          still_since_ms = now_ms;
        if (now_ms - still_since_ms >= LANDED_STILL_MS) {
          still_since_ms = 0;
          enter_state(STATE_TRACKING);
        }
      } else {
        still_since_ms = 0; // motion (or stale data) resets stillness
      }
      break;
    }

    case STATE_TRACKING:
      // One-time deployment on entry, then track.
      if (!payload.lift_deployed) { // add bool to PayloadHandle_t
        ESP_LOGI(TAG, "deploying lift: %d steps on STEP1", LIFT_STEPS / 4);
        gpio_set_level(PIN_STEP1_ENABLE, 0); // A4988/DRV8825: EN active low
        stepper_step(PIN_STEP1, PIN_STEP1_DIR, true, LIFT_STEPS / 4,
                     LIFT_PERIOD_US);
        gpio_set_level(PIN_STEP1_ENABLE, 1); // de-energize: save battery,
                                             // lift should be self-holding
        // Pitch up for chassis clearance
        ESP_LOGI(TAG, "pitch clearance: %d steps", PITCH_CLEARANCE);
        gpio_set_level(PIN_STEP1_ENABLE, 0);
        gpio_set_level(PIN_STEP2_ENABLE, 0);
        stepper_step(PIN_STEP1, PIN_STEP1_DIR, true, PITCH_CLEARANCE,
                     NUDGE_PERIOD_US);
        stepper_step(PIN_STEP2, PIN_STEP2_DIR, false, PITCH_CLEARANCE,
                     NUDGE_PERIOD_US);
        gpio_set_level(PIN_STEP1_ENABLE, 1);
        gpio_set_level(PIN_STEP2_ENABLE, 1);
        s_pitch_pos = PITCH_CLEARANCE; // tracking starts from here
        ESP_LOGI(TAG, "lift deploy done, aligning roll");
        roll_align_panel_up(s_latest_q);
        payload.lift_deployed = true;
        ESP_LOGI(TAG, "deploy sequence complete");
      }

      sun_track_update();
      break;

    case STATE_FAULT:
      // Safe state: motors de-energized, log and idle.
      gpio_set_level(PIN_STEP1_ENABLE, 1);
      gpio_set_level(PIN_STEP2_ENABLE, 1);
      vTaskDelay(pdMS_TO_TICKS(1000));
      break;

    case STATE_INIT:
    default:
      enter_state(STATE_FAULT); // should never be here at runtime
      break;
    }
  }
}

void app_main(void) {
  payload = (PayloadHandle_t){0};
  hardware = (PayloadHardware_t){0};
  // Hold SD card CS high BEFORE initializing the SPI bus so the
  // card never sees clocks and can't drive MISO.
  gpio_config_t sd_cs_pin = {
      .pin_bit_mask = 1ULL << PIN_CS_SD,
      .mode = GPIO_MODE_OUTPUT,
  };
  gpio_config(&sd_cs_pin);
  gpio_set_level(PIN_CS_SD, 1);

  init_spi();

  // Hold IMU CS high and NRST low before IMU init.
  gpio_config_t pre_imu = {
      .pin_bit_mask = (1ULL << PIN_CS_IMU) | (1ULL << PIN_RESET),
      .mode = GPIO_MODE_OUTPUT,
  };
  gpio_config(&pre_imu);
  gpio_set_level(PIN_CS_IMU, 1); // deselect IMU
  gpio_set_level(PIN_RESET, 0);  // hold IMU in reset

  vTaskDelay(pdMS_TO_TICKS(500)); // let SD card power stabilize
  init_spi_sd(&hardware);

  payload.imu_healthy = true;
  s_int_sem = xSemaphoreCreateBinary();
  gpio_install_isr_service(0);
  init_spi_imu(&hardware);

  imu_start();
  init_adc(&hardware);
  gpio_config_t stepper_pins = {
      .pin_bit_mask = (1ULL << PIN_STEP1) | (1ULL << PIN_STEP1_DIR) |
                      (1ULL << PIN_STEP1_ENABLE) | (1ULL << PIN_STEP2) |
                      (1ULL << PIN_STEP2_DIR) | (1ULL << PIN_STEP2_ENABLE),
      .mode = GPIO_MODE_OUTPUT,
  };
  gpio_config(&stepper_pins);
  gpio_set_level(PIN_STEP1_ENABLE, 1); // active low: start de-energized
  gpio_set_level(PIN_STEP2_ENABLE, 1);
  init_pwm();
  esc_set_pulse_us(1500);
  payload.state = STATE_IDLE;
  s_latest_q = xQueueCreate(1, sizeof(IMUData_t));
  s_log_q = xQueueCreate(64, sizeof(IMUData_t));
  if (s_latest_q == NULL || s_log_q == NULL) {
    ESP_LOGE(TAG, "queue creation failed"); // out of heap — don't continue
    payload.state = STATE_FAULT;
    return;
  }
  ESP_LOGI(TAG, "Payload Initialized");
  xTaskCreate(imu_task, "IMU", 4096, NULL, 3, NULL);
  if (payload.sd_healthy)
    xTaskCreate(log_task, "LOG", 4096, NULL, 1, NULL);
  xTaskCreate(state_task, "STATE", 4096, NULL, 2, NULL);
  return;
}

static inline float accel_mag(const IMUData_t *s) {
  if (!s->accel_valid)
    return -1.0f;
  return sqrtf(s->accel[0] * s->accel[0] + s->accel[1] * s->accel[1] +
               s->accel[2] * s->accel[2]);
}

static void enter_state(State_t next) {
  ESP_LOGI(TAG, "state %d -> %d", payload.state, next);
  payload.state = next;
  payload.state_entry_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

static inline uint32_t ms_in_state(void) {
  return (uint32_t)(esp_timer_get_time() / 1000) - payload.state_entry_ms;
}

static void init_spi() {
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

static void init_spi_sd(PayloadHardware_t *hardware) {
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = SPI2_HOST;
  host.max_freq_khz = 400; // keep at probing speed — series resistor on MISO
  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = PIN_CS_SD;
  slot_config.host_id = SPI2_HOST;
  esp_vfs_fat_mount_config_t mount_config = {
      .format_if_mount_failed = true,
      .max_files = 2,
  };
  esp_err_t err = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config,
                                          &mount_config, &hardware->sd_card);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "SD card mount failed: %s", esp_err_to_name(err));
    payload.sd_healthy = false;
    // The failed SDSPI driver may leave the SD card in a partial init
    // state where it still drives MISO.  Reclaim CS as a plain GPIO
    // and hold it high so the card tristates MISO for the IMU.
    gpio_config_t sd_cs_pin = {
        .pin_bit_mask = 1ULL << PIN_CS_SD,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&sd_cs_pin);
    gpio_set_level(PIN_CS_SD, 1);
    return;
  }
  payload.sd_healthy = true;
  ESP_LOGI(TAG, "SD Card Mounted");
}

static void init_spi_imu(PayloadHardware_t *hardware) {
  spi_device_interface_config_t dev_cfg = {
      .clock_speed_hz = 3000000,
      .mode = 3,
      .spics_io_num = -1, // manual CS — required for BNO085 SHTP-SPI
      .queue_size = 1,
  };
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &hardware->imu));
  gpio_config_t io = {
      .pin_bit_mask = 1ULL << PIN_INT,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };
  gpio_config(&io);
  gpio_config_t out = {
      .pin_bit_mask = (1ULL << PIN_CS_IMU) | (1ULL << PIN_WAKE) | (1ULL << PIN_RESET),
      .mode = GPIO_MODE_OUTPUT,
  };
  gpio_config(&out);
  gpio_set_level(PIN_CS_IMU, 1); // deselect
  gpio_set_level(PIN_WAKE, 1);
  gpio_set_level(PIN_RESET, 0);  // keep in reset — imu_reset() will release
                                  // with proper PS0 setup time

  gpio_isr_handler_add(PIN_INT, bno085_int_isr, NULL);
  ESP_LOGI(TAG, "IMU Initialized");
}

static esp_err_t imu_wait_int(TickType_t timeout) {
  xSemaphoreTake(s_int_sem, 0);

  if (gpio_get_level(PIN_INT) == 0) {
    return ESP_OK;
  }

  if (xSemaphoreTake(s_int_sem, timeout) == pdTRUE) {
    return ESP_OK;
  }
  return (gpio_get_level(PIN_INT) == 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t imu_write(const uint8_t *pkt, size_t len) {
  gpio_set_level(PIN_WAKE, 0);

  esp_err_t err = imu_wait_int(pdMS_TO_TICKS(200));
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "imu_write: chip did not acknowledge WAKE");
    gpio_set_level(PIN_WAKE, 1);
    return err;
  }

  spi_device_acquire_bus(hardware.imu, portMAX_DELAY);
  gpio_set_level(PIN_CS_IMU, 0);

  spi_transaction_t t = {
      .length = len * 8,
      .tx_buffer = pkt,
  };
  err = spi_device_polling_transmit(hardware.imu, &t);

  gpio_set_level(PIN_CS_IMU, 1);
  spi_device_release_bus(hardware.imu);
  gpio_set_level(PIN_WAKE, 1);
  return err;
}
static esp_err_t imu_read(uint8_t *buf, size_t buf_len, size_t *out_len,
                          TickType_t timeout, spi_device_handle_t imu) {
  if (buf_len < 4)
    return ESP_ERR_INVALID_ARG;

  esp_err_t err = imu_wait_int(timeout);
  if (err != ESP_OK)
    return err;

  spi_device_acquire_bus(imu, portMAX_DELAY);
  gpio_set_level(PIN_CS_IMU, 0);

  // --- Phase 1: read 4-byte SHTP header (inline buffer, no DMA) ---
  spi_transaction_t hdr_t = {
      .length = 4 * 8,
      .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
  };
  memset(hdr_t.tx_data, 0, 4);            // zeros on MOSI (like SparkFun)
  err = spi_device_polling_transmit(imu, &hdr_t);
  if (err != ESP_OK) {
    gpio_set_level(PIN_CS_IMU, 1);
    spi_device_release_bus(imu);
    return err;
  }

  // Copy header into caller's buffer
  memcpy(buf, hdr_t.rx_data, 4);

  uint16_t pkt_len = ((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) & 0x7FFF;

  if (pkt_len == 0) {
    gpio_set_level(PIN_CS_IMU, 1);
    spi_device_release_bus(imu);
    return ESP_ERR_NOT_FOUND;
  }
  if (pkt_len < 4 || pkt_len == 0x7FFF) {
    gpio_set_level(PIN_CS_IMU, 1);
    spi_device_release_bus(imu);
    ESP_LOGW(TAG, "bad SHTP header: %02X %02X %02X %02X (len=%u)",
             buf[0], buf[1], buf[2], buf[3], (unsigned)pkt_len);
    return ESP_ERR_INVALID_RESPONSE;
  }

  // --- Phase 2: read exactly (pkt_len - 4) body bytes, CS still low ---
  size_t body = pkt_len - 4;
  if (body > buf_len - 4)
    body = buf_len - 4;

  if (body > 0) {
    spi_transaction_t body_t = {
        .length = body * 8,
        .rx_buffer = buf + 4,
    };
    err = spi_device_polling_transmit(imu, &body_t);
  }

  gpio_set_level(PIN_CS_IMU, 1);
  spi_device_release_bus(imu);

  if (err != ESP_OK)
    return err;

  *out_len = 4 + body;
  return ESP_OK;
}

static int report_len(uint8_t id) {
  switch (id) {
  case RID_BASE_TIMESTAMP:
    return 5;
  case RID_TIMESTAMP_REBASE:
    return 5;
  case RID_ACCELEROMETER:
    return 10;
  case RID_GYROSCOPE:
    return 10;
  case RID_MAGNETOMETER:
    return 10;
  case RID_LINEAR_ACCEL:
    return 10;
  case RID_GAME_ROT_VECTOR:
    return 12; // i,j,k,real — no accuracy field
  case RID_ROTATION_VECTOR:
    return 14; // i,j,k,real + accuracy estimate
  default:
    return -1;
  }
}

static inline int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline float q_to_float(int16_t v, int q) {
  return (float)v / (float)(1 << q);
}

static esp_err_t imu_enable_report(uint8_t report_id, uint32_t interval_us) {
  static uint8_t s_seq_ch2 = 0; // each channel keeps its own TX sequence

  uint8_t pkt[4 + 17] = {0};
  pkt[0] = sizeof pkt; // length LSB (21)
  pkt[1] = 0;          // length MSB
  pkt[2] = 2;          // channel 2: control
  pkt[3] = s_seq_ch2++;

  pkt[4] = 0xFD; // Set Feature command
  pkt[5] = report_id;
  // pkt[6..8] flags + change sensitivity = 0 -> plain periodic reporting
  pkt[9] = (uint8_t)(interval_us & 0xFF); // interval, LE u32, µs
  pkt[10] = (uint8_t)((interval_us >> 8) & 0xFF);
  pkt[11] = (uint8_t)((interval_us >> 16) & 0xFF);
  pkt[12] = (uint8_t)((interval_us >> 24) & 0xFF);
  // pkt[13..20] batch interval + sensor-specific = 0

  return imu_write(pkt, sizeof pkt);
}

static esp_err_t imu_reset() {

  // PS0 (WAKE) and PS1 must be high when NRST rises -> SPI mode latched.
  gpio_set_level(PIN_WAKE, 1);
  gpio_set_level(PIN_RESET, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(PIN_RESET, 1);

  // Boot takes 100-300 ms; then the chip raises INT and starts talking.
  // Read packets until it goes quiet. Cap the loop so a babbling or
  // dead chip can't hang us.
  vTaskDelay(pdMS_TO_TICKS(300));
  ESP_LOGI(TAG, "IMU NRST released — INT pin level: %d (expect 0 if chip is alive)",
           gpio_get_level(PIN_INT));

  if (gpio_get_level(PIN_INT) != 0) {
    ESP_LOGE(TAG, "IMU never responded after reset — check wiring / MISO contention");
    return ESP_ERR_NOT_FOUND;
  }

  // Drain ALL startup packets before returning.  The chip sends 50+
  // advertisement/init packets; commands sent before it finishes are
  // ignored.  Use a short per-read timeout (50 ms) so we detect
  // "quiet" quickly, with a wall-clock cap to avoid hanging forever.
  static uint8_t buf[2048];
  size_t len;
  int n = 0;
  int64_t deadline = esp_timer_get_time() + 3000000; // 3 s max

  int errs = 0;
  while (esp_timer_get_time() < deadline) {
    esp_err_t rd = imu_read(buf, sizeof buf, &len, pdMS_TO_TICKS(50), hardware.imu);
    if (rd == ESP_ERR_NOT_FOUND) {
      continue;                     // empty SHTP — try again
    }
    if (rd == ESP_ERR_TIMEOUT) {
      break;                        // chip went quiet — done
    }
    if (rd != ESP_OK) {
      // Transient glitches (e.g. FF FF FF FF) are common right after
      // reset — skip them instead of aborting the whole drain.
      if (++errs <= 5) {
        ESP_LOGW(TAG, "IMU drain: transient error (%s), skipping", esp_err_to_name(rd));
        continue;
      }
      ESP_LOGE(TAG, "IMU drain: too many errors, aborting");
      return rd;
    }
    n++;
  }

  ESP_LOGI(TAG, "IMU reset OK — drained %d startup packets", n);
  return ESP_OK;
}

static esp_err_t imu_start() {
  esp_err_t err = imu_reset();
  if (err != ESP_OK)
    return err;

  // 0x05 rotation vector, 0x01 accelerometer, 0x04 linear accel @ 100 Hz
  err = imu_enable_report(0x05, 10000);
  ESP_LOGI(TAG, "enable rot_vec: %s", esp_err_to_name(err));
  if (err != ESP_OK)
    return err;
  err = imu_enable_report(0x01, 10000);
  ESP_LOGI(TAG, "enable accel:   %s", esp_err_to_name(err));
  if (err != ESP_OK)
    return err;
  err = imu_enable_report(0x04, 10000);
  ESP_LOGI(TAG, "enable lin_acc: %s", esp_err_to_name(err));
  return err;
}

static bool imu_parse_packet(const uint8_t *buf, size_t len) {
  if (len < 4 + 1)
    return false;
  if (buf[2] != 3)
    return false;

  bool got_any = false;
  size_t i = 4;

  while (i < len) {
    uint8_t id = buf[i];
    int rlen = report_len(id);

    if (rlen < 0 || i + (size_t)rlen > len) {
      break;
    }

    const uint8_t *d = &buf[i + 4];

    switch (id) {
    case RID_ROTATION_VECTOR:
      payload.imu.quat[0] = q_to_float(le16(d + 6), 14); // w
      payload.imu.quat[1] = q_to_float(le16(d + 0), 14); // x (i)
      payload.imu.quat[2] = q_to_float(le16(d + 2), 14); // y (j)
      payload.imu.quat[3] = q_to_float(le16(d + 4), 14); // z (k)
      // d+8: heading accuracy estimate, Q12 radians — useful for
      // gating sun-tracking decisions on attitude quality.
      // s->quat_accuracy_rad = q_to_float(le16(d + 8), 12);
      payload.imu.quat_valid = true;
      got_any = true;
      break;

    case RID_ACCELEROMETER:
      payload.imu.accel[0] = q_to_float(le16(d + 0), 8);
      payload.imu.accel[1] = q_to_float(le16(d + 2), 8);
      payload.imu.accel[2] = q_to_float(le16(d + 4), 8);
      payload.imu.accel_valid = true;
      got_any = true;
      break;

    case RID_LINEAR_ACCEL:
      payload.imu.lin_accel[0] = q_to_float(le16(d + 0), 8);
      payload.imu.lin_accel[1] = q_to_float(le16(d + 2), 8);
      payload.imu.lin_accel[2] = q_to_float(le16(d + 4), 8);
      payload.imu.lin_accel_valid = true;
      got_any = true;
      break;

    case RID_BASE_TIMESTAMP:
    case RID_TIMESTAMP_REBASE:
      break;
    default:
      break;
    }
    i += (size_t)rlen;
  }
  return got_any;
}

static void init_adc(PayloadHardware_t *hardware) {
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &hardware->adc1));

  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_12,
  };
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(hardware->adc1, ADC_CHANNEL_4, &config));
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(hardware->adc1, ADC_CHANNEL_5, &config));
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(hardware->adc1, ADC_CHANNEL_6, &config));
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(hardware->adc1, ADC_CHANNEL_7, &config));
}

static int read_ldr(adc_channel_t ch) {
  int sum = 0, raw = 0, ok = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    if (adc_oneshot_read(hardware.adc1, ch, &raw) == ESP_OK) {
      sum += raw;
      ok++;
    }
  }
  int avg = ok > 0 ? sum / ok : 0;
#if LDR_BRIGHTER_IS_HIGHER
  return avg;
#else
  return 4095 - avg;
#endif
}

static void pitch_nudge(bool dir)
{
    int32_t next = s_pitch_pos + (dir ? STEPS_PER_NUDGE : -STEPS_PER_NUDGE);
    if (next < PITCH_STEP_MIN || next > PITCH_STEP_MAX) return;  // soft limit
 
    gpio_set_level(PIN_STEP1_ENABLE, 0);             // EN active low
    gpio_set_level(PIN_STEP2_ENABLE, 0);
    stepper_step(PIN_STEP1, PIN_STEP1_DIR,  dir, STEPS_PER_NUDGE, NUDGE_PERIOD_US);
    stepper_step(PIN_STEP2, PIN_STEP2_DIR, !dir, STEPS_PER_NUDGE, NUDGE_PERIOD_US);
    gpio_set_level(PIN_STEP1_ENABLE, 1);             // de-energize between nudges
    gpio_set_level(PIN_STEP2_ENABLE, 1);
    s_pitch_pos = next;
}

static void roll_nudge(bool dir)
{
    esc_set_pulse_us(dir ? ESC_NEUTRAL_US + ESC_BURST_OFFSET
                         : ESC_NEUTRAL_US - ESC_BURST_OFFSET);
    vTaskDelay(pdMS_TO_TICKS(ESC_BURST_MS));
    esc_set_pulse_us(ESC_NEUTRAL_US);
}


static void roll_align_panel_up(QueueHandle_t latest_q)
{
    IMUData_t s;
    for (int i = 0; i < 60; i++) {                  // cap: ~60 blips max
        if (xQueuePeek(latest_q, &s, pdMS_TO_TICKS(200)) != pdTRUE ||
            !s.accel_valid) {
            return;                                  // no IMU: skip coarse align,
        }                                            // LDRs alone must do the job
 
        float roll = atan2f(s.accel[1], s.accel[2]); // rad, 0 = panel up
        if (fabsf(roll) < 0.15f) return;             // within ~9 deg: good enough
 
        roll_nudge(roll > 0 ? false : true);         // TODO: verify sign on bench
        vTaskDelay(pdMS_TO_TICKS(500));              // settle before re-measuring
    }
}

static void init_pwm() {
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

static void sun_track_update(void) {
  payload.sun.north = (uint16_t)read_ldr(ADC_CH_NORTH);
  payload.sun.south = (uint16_t)read_ldr(ADC_CH_SOUTH);
  payload.sun.east = (uint16_t)read_ldr(ADC_CH_EAST);
  payload.sun.west = (uint16_t)read_ldr(ADC_CH_WEST);

  int ew_sum = payload.sun.east + payload.sun.west;
  int ns_sum = payload.sun.north + payload.sun.south;

  // Periodic LDR status log (~1 Hz at 50 Hz loop rate)
  static int sun_div = 0;
  if (++sun_div >= 50) {
    sun_div = 0;
    ESP_LOGI(TAG, "LDR N=%u S=%u E=%u W=%u  ew_sum=%d ns_sum=%d",
             payload.sun.north, payload.sun.south,
             payload.sun.east, payload.sun.west, ew_sum, ns_sum);
  }

  // Night / heavy shade: don't chase noise in the dark.
  if (ew_sum < DARK_SUM_COUNTS || ns_sum < DARK_SUM_COUNTS) {
    return;
  }

  float roll_err = (float)(payload.sun.east - payload.sun.west) / (float)ew_sum;
  float pitch_err =
      (float)(payload.sun.north - payload.sun.south) / (float)ns_sum;

  // Roll and pitch are orthogonal body axes here, so the errors decouple:
  // correct both independently each cycle. Deadband = hysteresis.
  // Sign convention TODO: verify both axes on hardware; flip the bool if
  // the panel runs away from the light instead of toward it.
  // ("east/west" LDR pair = across the panel width -> roll;
  //  "north/south" pair = along the body axis -> pitch jacks.)
  if (roll_err > ERR_DEADBAND) {
    roll_nudge(true);
  } else if (roll_err < -ERR_DEADBAND) {
    roll_nudge(false);
  }

  if (pitch_err > ERR_DEADBAND) {
    pitch_nudge(true);
  } else if (pitch_err < -ERR_DEADBAND) {
    pitch_nudge(false);
  }
}

static void esc_set_pulse_us(uint32_t us) {
  uint32_t duty = (us * 65536) / 20000;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void stepper_step(gpio_num_t pin_step, gpio_num_t pin_dir, bool dir,
                         uint32_t steps, uint32_t step_period_us) {
  gpio_set_level(pin_dir, dir);
  esp_rom_delay_us(2);

  for (uint32_t i = 0; i < steps; i++) {
    gpio_set_level(pin_step, 1);
    esp_rom_delay_us(3);
    gpio_set_level(pin_step, 0);
    if (step_period_us - 3 >= 1000) {
      vTaskDelay(1); // yield to RTOS every step to avoid WDT
    } else {
      esp_rom_delay_us(step_period_us - 3);
    }
  }
}
