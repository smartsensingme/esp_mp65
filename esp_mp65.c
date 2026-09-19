#include "esp_mp65.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#if CONFIG_ESP_MP65_THREAD_SAFE
#include "freertos/semphr.h"
#endif

#define MP65_REG_SMPLRT_DIV 0x19
#define MP65_REG_CONFIG 0x1A
#define MP65_REG_GYRO_CONFIG 0x1B
#define MP65_REG_ACCEL_CONFIG 0x1C
#define MP65_REG_ACCEL_CONFIG2 0x1D
#define MP65_REG_INT_PIN_CFG 0x37
#define MP65_REG_INT_ENABLE 0x38
#define MP65_REG_INT_STATUS 0x3A
#define MP65_REG_ACCEL_XOUT_H 0x3B
#define MP65_REG_USER_CTRL 0x6A
#define MP65_REG_PWR_MGMT_1 0x6B
#define MP65_REG_PWR_MGMT_2 0x6C
#define MP65_REG_WHO_AM_I 0x75

#define MP65_SPI_READ_BIT 0x80
#define MP65_WHO_AM_I_VALUE 0x70
#define MP65_PWR_DEVICE_RESET 0x80
#define MP65_PWR_CLKSEL_PLL_X 0x01
#define MP65_USER_I2C_IF_DIS 0x10
#define MP65_MAX_TRANSFER_BYTES 15
#define MP65_GRAVITY_MPS2 9.80665f
#define MP65_DEG_TO_RAD 0.01745329251994329577f

static const char *TAG = "esp_mp65";

struct esp_mp65_dev_t {
  spi_host_device_t spi_host;
  spi_device_handle_t spi_device;
  esp_mp65_accel_range_t accel_range;
  esp_mp65_gyro_range_t gyro_range;
  bool owns_bus;
  bool bus_acquired;
  int spi_pins[4];
  QueueHandle_t ready_queue;
  int ready_gpio, ready_core;
  bool owns_isr_service, ready_handler;
  uint32_t ready_sequence;
  int64_t ready_previous_us;
#if CONFIG_ESP_MP65_THREAD_SAFE
  SemaphoreHandle_t mutex;
#endif
};

/* Internal: read, read_who_am_i, acquire_bus, release_bus and the internal INT
 * setter serialize SPI. Lifecycle/queue waits are not protected as a whole. */
static esp_err_t mp65_lock(esp_mp65_handle_t handle) {
#if CONFIG_ESP_MP65_THREAD_SAFE
  if (xSemaphoreTake(handle->mutex,
                     pdMS_TO_TICKS(CONFIG_ESP_MP65_LOCK_TIMEOUT_MS)) !=
      pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#else
  (void)handle;
#endif
  return ESP_OK;
}

/* Internal: the same SPI callers as mp65_lock release their acquired mutex. */
static void mp65_unlock(esp_mp65_handle_t handle) {
#if CONFIG_ESP_MP65_THREAD_SAFE
  xSemaphoreGive(handle->mutex);
#else
  (void)handle;
#endif
}

/* Internal: init, read, read_who_am_i and the internal INT setter. Fixed-size,
 * not bounded-time, polling burst. Runtime callers hold the mutex; init owns
 * the unpublished handle exclusively. Output is copied only on success. */
static esp_err_t mp65_read_registers_unlocked(esp_mp65_handle_t handle,
                                              uint8_t reg, uint8_t *data,
                                              size_t length) {
  if (length == 0 || length + 1 > MP65_MAX_TRANSFER_BYTES) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t tx[MP65_MAX_TRANSFER_BYTES] = {0};
  uint8_t rx[MP65_MAX_TRANSFER_BYTES] = {0};
  tx[0] = reg | MP65_SPI_READ_BIT;

  spi_transaction_t transaction = {
      .length = (length + 1) * 8,
      .tx_buffer = tx,
      .rx_buffer = rx,
  };
  /* Propagate read errors without formatted logging in the acquisition path. */
  esp_err_t err = spi_device_polling_transmit(handle->spi_device, &transaction);
  if (err != ESP_OK)
    return err;
  memcpy(data, &rx[1], length);
  return ESP_OK;
}

/* Internal: init and mp65_set_data_ready_interrupt write registers. No SPI
 * transaction timeout is supplied by the polling API. */
static esp_err_t mp65_write_register_unlocked(esp_mp65_handle_t handle,
                                              uint8_t reg, uint8_t value) {
  spi_transaction_t transaction = {
      .flags = SPI_TRANS_USE_TXDATA,
      .length = 16,
  };
  transaction.tx_data[0] = reg & ~MP65_SPI_READ_BIT;
  transaction.tx_data[1] = value;
  return spi_device_polling_transmit(handle->spi_device, &transaction);
}

/* Internal, called by esp_mp65_read: signed big-endian register conversion. */
static int16_t mp65_be16(const uint8_t *bytes) {
  return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

/* Internal, called by esp_mp65_init before resource/hardware changes. */
static esp_err_t mp65_validate_config(const esp_mp65_config_t *config) {
  ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "config is NULL");
  ESP_RETURN_ON_FALSE(
      config->spi_host == SPI2_HOST || config->spi_host == SPI3_HOST,
      ESP_ERR_INVALID_ARG, TAG, "SPI host must be SPI2 or SPI3");
  if (config->initialize_spi_bus) {
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(config->pin_sclk),
                        ESP_ERR_INVALID_ARG, TAG,
                        "SCLK is not an output-capable GPIO");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(config->pin_mosi),
                        ESP_ERR_INVALID_ARG, TAG,
                        "MOSI is not an output-capable GPIO");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(config->pin_miso),
                        ESP_ERR_INVALID_ARG, TAG, "MISO is not a valid GPIO");
  }
  ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(config->pin_cs),
                      ESP_ERR_INVALID_ARG, TAG,
                      "CS is not an output-capable GPIO");
  ESP_RETURN_ON_FALSE(
      config->clock_speed_hz >= 100000 && config->clock_speed_hz <= 1000000,
      ESP_ERR_INVALID_ARG, TAG, "SPI clock must be between 100 kHz and 1 MHz");
  ESP_RETURN_ON_FALSE(config->accel_range <= ESP_MP65_ACCEL_RANGE_16G,
                      ESP_ERR_INVALID_ARG, TAG, "invalid accelerometer range");
  ESP_RETURN_ON_FALSE(config->gyro_range <= ESP_MP65_GYRO_RANGE_2000DPS,
                      ESP_ERR_INVALID_ARG, TAG, "invalid gyroscope range");
  ESP_RETURN_ON_FALSE(config->dlpf_cfg <= 7, ESP_ERR_INVALID_ARG, TAG,
                      "invalid DLPF setting");
  return ESP_OK;
}

/* External only: validate, allocate, attach SPI, configure sensor and publish.
 * Failure labels unwind resources; cleanup errors do not replace setup error.
 */
esp_err_t esp_mp65_init(const esp_mp65_config_t *config,
                        esp_mp65_handle_t *out_handle) {
  /* Validate first so failure cannot leave a partially created instance. */
  ESP_RETURN_ON_ERROR(mp65_validate_config(config), TAG,
                      "invalid configuration");
  ESP_RETURN_ON_FALSE(out_handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "out_handle is NULL");
  *out_handle = NULL;

  esp_mp65_handle_t handle = calloc(1, sizeof(*handle));
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG,
                      "cannot allocate handle");
  handle->spi_host = config->spi_host;
  handle->accel_range = config->accel_range;
  handle->gyro_range = config->gyro_range;
  handle->spi_pins[0] = config->pin_cs;
  handle->spi_pins[1] = config->pin_sclk;
  handle->spi_pins[2] = config->pin_mosi;
  handle->spi_pins[3] = config->pin_miso;

  /* Allocate synchronization once; reads use it without allocating it. */
#if CONFIG_ESP_MP65_THREAD_SAFE
  handle->mutex = xSemaphoreCreateMutex();
  if (handle->mutex == NULL) {
    free(handle);
    return ESP_ERR_NO_MEM;
  }
#endif

  spi_bus_config_t bus_config = {
      .mosi_io_num = config->pin_mosi,
      .miso_io_num = config->pin_miso,
      .sclk_io_num = config->pin_sclk,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = MP65_MAX_TRANSFER_BYTES,
  };
  esp_err_t err = ESP_OK;
  /* Bus lifetime and host exclusivity are deliberately separate policies. */
  if (config->initialize_spi_bus) {
    err = spi_bus_initialize(config->spi_host, &bus_config, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "failed to initialize SPI bus: %s", esp_err_to_name(err));
      goto fail_handle;
    }
    handle->owns_bus = true;
  }

  spi_device_interface_config_t device_config = {
      .clock_speed_hz = config->clock_speed_hz,
      .mode = 0,
      .spics_io_num = config->pin_cs,
      .queue_size = 1,
  };
  err =
      spi_bus_add_device(config->spi_host, &device_config, &handle->spi_device);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to add SPI device: %s", esp_err_to_name(err));
    goto fail_bus;
  }

  if (config->acquire_bus_on_init) {
    err = spi_device_acquire_bus(handle->spi_device, portMAX_DELAY);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "failed to acquire SPI bus: %s", esp_err_to_name(err));
      goto fail_device;
    }
    handle->bus_acquired = true;
  }

  /* Reset, select SPI/PLL operation, and allow the sensor to settle. */
  err = mp65_write_register_unlocked(handle, MP65_REG_PWR_MGMT_1,
                                     MP65_PWR_DEVICE_RESET);
  if (err != ESP_OK) {
    goto fail_device;
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  err = mp65_write_register_unlocked(handle, MP65_REG_USER_CTRL,
                                     MP65_USER_I2C_IF_DIS);
  if (err != ESP_OK) {
    goto fail_device;
  }
  err = mp65_write_register_unlocked(handle, MP65_REG_PWR_MGMT_1,
                                     MP65_PWR_CLKSEL_PLL_X);
  if (err != ESP_OK) {
    goto fail_device;
  }
  err = mp65_write_register_unlocked(handle, MP65_REG_PWR_MGMT_2, 0);
  if (err != ESP_OK) {
    goto fail_device;
  }
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Reject a responsive but incompatible part before applying configuration. */
  uint8_t who_am_i = 0;
  err = mp65_read_registers_unlocked(handle, MP65_REG_WHO_AM_I, &who_am_i, 1);
  if (err != ESP_OK) {
    goto fail_device;
  }
  if (who_am_i != MP65_WHO_AM_I_VALUE) {
    ESP_LOGE(TAG, "unexpected WHO_AM_I 0x%02x (expected 0x%02x)", who_am_i,
             MP65_WHO_AM_I_VALUE);
    err = ESP_ERR_NOT_FOUND;
    goto fail_device;
  }

  /* Program filter, full-scale ranges, and output-rate divider. */
  err = mp65_write_register_unlocked(handle, MP65_REG_CONFIG, config->dlpf_cfg);
  if (err != ESP_OK) {
    goto fail_device;
  }
  err = mp65_write_register_unlocked(handle, MP65_REG_GYRO_CONFIG,
                                     (uint8_t)config->gyro_range << 3);
  if (err != ESP_OK) {
    goto fail_device;
  }
  err = mp65_write_register_unlocked(handle, MP65_REG_ACCEL_CONFIG,
                                     (uint8_t)config->accel_range << 3);
  if (err != ESP_OK) {
    goto fail_device;
  }
  err = mp65_write_register_unlocked(handle, MP65_REG_ACCEL_CONFIG2,
                                     config->dlpf_cfg);
  if (err != ESP_OK) {
    goto fail_device;
  }
  err = mp65_write_register_unlocked(handle, MP65_REG_SMPLRT_DIV,
                                     config->sample_rate_divider);
  if (err != ESP_OK) {
    goto fail_device;
  }

  *out_handle = handle;
  ESP_LOGI(TAG, "MPU-6500 ready (WHO_AM_I=0x%02x)", who_am_i);
  return ESP_OK;

/* Unwind in strict reverse acquisition order; also used after sensor errors. */
fail_device:
  if (handle->bus_acquired) {
    spi_device_release_bus(handle->spi_device);
  }
  spi_bus_remove_device(handle->spi_device);
fail_bus:
  if (handle->owns_bus) {
    spi_bus_free(config->spi_host);
  }
fail_handle:
#if CONFIG_ESP_MP65_THREAD_SAFE
  vSemaphoreDelete(handle->mutex);
#endif
  free(handle);
  return err;
}

/* External only: stop managed INT then release SPI/free state. Later cleanup
 * errors can leave partial teardown; application serializes every user. */
esp_err_t esp_mp65_deinit(esp_mp65_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "handle is NULL");
  esp_err_t stop_error = esp_mp65_data_ready_stop(handle);
  if (stop_error != ESP_OK)
    return stop_error;
  if (handle->bus_acquired) {
    spi_device_release_bus(handle->spi_device);
  }
  esp_err_t err = spi_bus_remove_device(handle->spi_device);
  if (err != ESP_OK) {
    return err;
  }
  if (handle->owns_bus) {
    err = spi_bus_free(handle->spi_host);
    if (err != ESP_OK) {
      return err;
    }
  }
#if CONFIG_ESP_MP65_THREAD_SAFE
  vSemaphoreDelete(handle->mutex);
#endif
  free(handle);
  return ESP_OK;
}

/* External only: serialized identity read via mp65_read_registers_unlocked. */
esp_err_t esp_mp65_read_who_am_i(esp_mp65_handle_t handle, uint8_t *who_am_i) {
  ESP_RETURN_ON_FALSE(handle != NULL && who_am_i != NULL, ESP_ERR_INVALID_ARG,
                      TAG, "invalid argument");
  ESP_RETURN_ON_ERROR(mp65_lock(handle), TAG, "sensor lock timed out");
  esp_err_t err =
      mp65_read_registers_unlocked(handle, MP65_REG_WHO_AM_I, who_am_i, 1);
  mp65_unlock(handle);
  return err;
}

/* External only: coherent burst and nominal SI conversion. No calibration,
 * remapping, new-sample detection or historical sensor buffering. */
esp_err_t esp_mp65_read(esp_mp65_handle_t handle, esp_mp65_sample_t *sample) {
  ESP_RETURN_ON_FALSE(handle != NULL && sample != NULL, ESP_ERR_INVALID_ARG,
                      TAG, "invalid argument");

  uint8_t data[14];
  esp_err_t err = mp65_lock(handle);
  if (err != ESP_OK)
    return err;
  err = mp65_read_registers_unlocked(handle, MP65_REG_ACCEL_XOUT_H, data,
                                     sizeof(data));
  mp65_unlock(handle);
  if (err != ESP_OK) {
    return err;
  }

  /* Decode only after a complete successful transfer, preserving output on
   * communication failure. */
  sample->accel_raw[0] = mp65_be16(&data[0]);
  sample->accel_raw[1] = mp65_be16(&data[2]);
  sample->accel_raw[2] = mp65_be16(&data[4]);
  sample->temperature_raw = mp65_be16(&data[6]);
  sample->gyro_raw[0] = mp65_be16(&data[8]);
  sample->gyro_raw[1] = mp65_be16(&data[10]);
  sample->gyro_raw[2] = mp65_be16(&data[12]);

  const float accel_lsb_per_g[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
  const float gyro_lsb_per_dps[] = {131.0f, 65.5f, 32.8f, 16.4f};
  const float accel_scale =
      MP65_GRAVITY_MPS2 / accel_lsb_per_g[handle->accel_range];
  const float gyro_scale =
      MP65_DEG_TO_RAD / gyro_lsb_per_dps[handle->gyro_range];

  for (int axis = 0; axis < 3; ++axis) {
    sample->accel_mps2[axis] = sample->accel_raw[axis] * accel_scale;
    sample->gyro_rads[axis] = sample->gyro_raw[axis] * gyro_scale;
  }
  sample->temperature_c = sample->temperature_raw / 333.87f + 21.0f;
  return ESP_OK;
}

/* Internal: low-level public setter and managed start/stop configure sensor
 * INT. Disable first, clear pending status, then enable only RAW_RDY. */
static esp_err_t mp65_set_data_ready_interrupt(esp_mp65_handle_t handle,
                                               bool enabled) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;
  esp_err_t err = mp65_lock(handle);
  if (err != ESP_OK)
    return err;
  err = mp65_write_register_unlocked(handle, MP65_REG_INT_ENABLE, 0);
  if (err == ESP_OK && enabled) {
    /* Active high, push-pull, non-latched pulses; clear status on status read.
     */
    err = mp65_write_register_unlocked(handle, MP65_REG_INT_PIN_CFG, 0);
    uint8_t status;
    if (err == ESP_OK)
      err =
          mp65_read_registers_unlocked(handle, MP65_REG_INT_STATUS, &status, 1);
    if (err == ESP_OK)
      err = mp65_write_register_unlocked(handle, MP65_REG_INT_ENABLE, 1);
  }
  mp65_unlock(handle);
  return err;
}

/* External only: disallow manual INT changes while managed mode owns it. */
esp_err_t esp_mp65_set_data_ready_interrupt(esp_mp65_handle_t handle,
                                            bool enabled) {
  if (!handle)
    return ESP_ERR_INVALID_ARG;
  if (handle->ready_queue)
    return ESP_ERR_INVALID_STATE;
  return mp65_set_data_ready_interrupt(handle, enabled);
}

/* Internal callback registered by data_ready_start, invoked by GPIO service.
 * Integer-only timestamp/count/overwrite; queue holds events, not samples.
 * Non-IRAM service: no cache-off guarantee. */
static void mp65_on_data_ready(void *arg) {
  esp_mp65_handle_t h = arg;
  int64_t now = esp_timer_get_time();
  esp_mp65_data_ready_event_t event = {
      .sequence = ++h->ready_sequence,
      .isr_us = now,
      .gap_us = h->ready_previous_us ? now - h->ready_previous_us : 0,
  };
  h->ready_previous_us = now;
  BaseType_t wake = pdFALSE;
  xQueueOverwriteFromISR(h->ready_queue, &event, &wake);
  if (wake)
    portYIELD_FROM_ISR();
}

/* External and internal (deinit): disable source before freeing ISR state.
 * Join consumer first; same-core removal ensures no ISR remains in flight. */
esp_err_t esp_mp65_data_ready_stop(esp_mp65_handle_t h) {
  if (!h)
    return ESP_ERR_INVALID_ARG;
  if (!h->ready_queue)
    return ESP_OK;
  if (xTaskGetCoreID(NULL) == tskNO_AFFINITY)
    return ESP_ERR_INVALID_STATE;
  if (xPortGetCoreID() != h->ready_core)
    return ESP_ERR_INVALID_STATE;
  esp_err_t err = mp65_set_data_ready_interrupt(h, false);
  if (err != ESP_OK)
    return err;
  /* Same-core pinned lifecycle means no ISR can still execute after removal. */
  if (h->ready_handler) {
    err = gpio_isr_handler_remove(h->ready_gpio);
    if (err != ESP_OK)
      return err;
    h->ready_handler = false;
  }
  if (h->owns_isr_service) {
    err = gpio_uninstall_isr_service();
    if (err != ESP_OK)
      return err;
    h->owns_isr_service = false;
  }
  vQueueDelete(h->ready_queue);
  h->ready_queue = NULL;
  return ESP_OK;
}

/* External only: validate core/ownership, disable source, allocate queue,
 * attach service/handler, then enable sensor. Failure detaches before free. */
esp_err_t esp_mp65_data_ready_start(esp_mp65_handle_t h,
                                    const esp_mp65_data_ready_config_t *c) {
  if (!h || !c || !GPIO_IS_VALID_GPIO(c->gpio_num))
    return ESP_ERR_INVALID_ARG;
  if (h->ready_queue)
    return ESP_ERR_INVALID_STATE;
  if (xTaskGetCoreID(NULL) == tskNO_AFFINITY)
    return ESP_ERR_INVALID_STATE;
  for (unsigned i = 0; i < 4; ++i)
    if (c->gpio_num == h->spi_pins[i])
      return ESP_ERR_INVALID_ARG;
  esp_err_t err = mp65_set_data_ready_interrupt(h, false);
  if (err != ESP_OK)
    return err;
  h->ready_queue = xQueueCreate(1, sizeof(esp_mp65_data_ready_event_t));
  if (!h->ready_queue)
    return ESP_ERR_NO_MEM;
  h->ready_gpio = c->gpio_num;
  h->ready_core = xPortGetCoreID();
  h->ready_sequence = 0;
  h->ready_previous_us = 0;
  if (c->install_isr_service) {
    err = gpio_install_isr_service(0);
    if (err != ESP_OK)
      goto fail;
    h->owns_isr_service = true;
  }
  gpio_config_t gpio = {
      .pin_bit_mask = UINT64_C(1) << c->gpio_num,
      .mode = GPIO_MODE_INPUT,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_POSEDGE,
  };
  err = gpio_config(&gpio);
  if (err != ESP_OK)
    goto fail;
  err = gpio_isr_handler_add(c->gpio_num, mp65_on_data_ready, h);
  if (err != ESP_OK)
    goto fail;
  h->ready_handler = true;
  err = mp65_set_data_ready_interrupt(h, true);
  if (err == ESP_OK)
    return ESP_OK;
fail:
  /* Detach before freeing ISR state even if sensor communication failed.
   * Valid pin/service lifetime is guaranteed by the lifecycle contract. */
  if (h->ready_handler)
    gpio_isr_handler_remove(h->ready_gpio);
  h->ready_handler = false;
  if (h->owns_isr_service)
    gpio_uninstall_isr_service();
  h->owns_isr_service = false;
  vQueueDelete(h->ready_queue);
  h->ready_queue = NULL;
  return err;
}

/* External only: one consumer waits without SPI or held mutex. Finite timeout
 * rounds upward and saturates below portMAX_DELAY. */
esp_err_t esp_mp65_wait_data_ready(esp_mp65_handle_t h,
                                   esp_mp65_data_ready_event_t *event,
                                   uint32_t timeout_ms) {
  if (!h || !event)
    return ESP_ERR_INVALID_ARG;
  if (!h->ready_queue)
    return ESP_ERR_INVALID_STATE;
  uint64_t ticks = ((uint64_t)timeout_ms * configTICK_RATE_HZ + 999) / 1000;
  TickType_t wait =
      timeout_ms == UINT32_MAX
          ? portMAX_DELAY
          : (TickType_t)(ticks >= portMAX_DELAY ? portMAX_DELAY - 1 : ticks);
  return xQueueReceive(h->ready_queue, event, wait) == pdTRUE ? ESP_OK
                                                              : ESP_ERR_TIMEOUT;
}

/* External only: phase-level reservation; acquisition itself may wait forever.
 */
esp_err_t esp_mp65_acquire_bus(esp_mp65_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "handle is NULL");
  ESP_RETURN_ON_ERROR(mp65_lock(handle), TAG, "sensor lock timed out");

  if (handle->bus_acquired) {
    mp65_unlock(handle);
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = spi_device_acquire_bus(handle->spi_device, portMAX_DELAY);
  if (err == ESP_OK) {
    handle->bus_acquired = true;
  }
  mp65_unlock(handle);
  return err;
}

/* External only: release paired with explicit or initialization acquisition. */
esp_err_t esp_mp65_release_bus(esp_mp65_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "handle is NULL");
  ESP_RETURN_ON_ERROR(mp65_lock(handle), TAG, "sensor lock timed out");

  if (!handle->bus_acquired) {
    mp65_unlock(handle);
    return ESP_ERR_INVALID_STATE;
  }

  spi_device_release_bus(handle->spi_device);
  handle->bus_acquired = false;
  mp65_unlock(handle);
  return ESP_OK;
}
