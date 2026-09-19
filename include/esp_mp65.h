#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque MPU-6500 device handle.
 *
 * A successful esp_mp65_init() creates one handle. The component owns the
 * handle and its SPI device until esp_mp65_deinit(); the application must not
 * copy, inspect, or free the pointed-to object.
 */
typedef struct esp_mp65_dev_t *esp_mp65_handle_t;

/** Accelerometer full-scale selection; values map to ACCEL_CONFIG.AFS_SEL. */
typedef enum {
  ESP_MP65_ACCEL_RANGE_2G = 0, /**< +/-2 g; nominal sensitivity 16384 LSB/g. */
  ESP_MP65_ACCEL_RANGE_4G,     /**< +/-4 g; nominal sensitivity 8192 LSB/g. */
  ESP_MP65_ACCEL_RANGE_8G,     /**< +/-8 g; nominal sensitivity 4096 LSB/g. */
  ESP_MP65_ACCEL_RANGE_16G,    /**< +/-16 g; nominal sensitivity 2048 LSB/g. */
} esp_mp65_accel_range_t;

/** Gyroscope full-scale selection; values map to GYRO_CONFIG.FS_SEL. */
typedef enum {
  ESP_MP65_GYRO_RANGE_250DPS = 0, /**< +/-250 deg/s; 131 LSB/(deg/s). */
  ESP_MP65_GYRO_RANGE_500DPS,     /**< +/-500 deg/s; 65.5 LSB/(deg/s). */
  ESP_MP65_GYRO_RANGE_1000DPS,    /**< +/-1000 deg/s; 32.8 LSB/(deg/s). */
  ESP_MP65_GYRO_RANGE_2000DPS,    /**< +/-2000 deg/s; 16.4 LSB/(deg/s). */
} esp_mp65_gyro_range_t;

/** Per-instance configuration; values are copied by esp_mp65_init(). */
typedef struct {
  spi_host_device_t spi_host; /**< SPI2_HOST or SPI3_HOST. */
  int pin_sclk; /**< SCLK GPIO; checked when initialize_spi_bus is true. */
  int pin_mosi; /**< Controller-output/SDI GPIO; checked when creating bus. */
  int pin_miso; /**< Controller-input/SDO GPIO; checked when creating bus. */
  int pin_cs;   /**< Active-low chip-select; must be output-capable. */
  int clock_speed_hz;       /**< SPI clock in [100000, 1000000] Hz. */
  bool initialize_spi_bus;  /**< If true, initialize and later free spi_host. */
  bool acquire_bus_on_init; /**< If true, reserve spi_host until release/deinit.
                             */
  esp_mp65_accel_range_t accel_range; /**< Full scale for setup/conversion. */
  esp_mp65_gyro_range_t gyro_range;   /**< Full scale for setup/conversion. */
  uint8_t dlpf_cfg; /**< CONFIG and ACCEL_CONFIG2 filter value in [0, 7]. */
  uint8_t sample_rate_divider; /**< SMPLRT_DIV register value in [0, 255]. */
} esp_mp65_config_t;

/** One coherent 14-byte sensor burst. Axis order is X, Y, Z. */
typedef struct {
  int16_t accel_raw[3];    /**< Acceleration register values in sensor LSB. */
  int16_t temperature_raw; /**< Temperature register value in sensor LSB. */
  int16_t gyro_raw[3];     /**< Gyroscope register values in sensor LSB. */
  float accel_mps2[3];     /**< Acceleration in m/s^2, using g=9.80665. */
  float temperature_c;     /**< Temperature in degrees Celsius. */
  float gyro_rads[3];      /**< Angular velocity in radians per second. */
} esp_mp65_sample_t;

/** SPI host selected by the ESP_MP65_SPI_HOST Kconfig choice. */
#if CONFIG_ESP_MP65_SPI_HOST_3
#define ESP_MP65_DEFAULT_SPI_HOST SPI3_HOST
#else
#define ESP_MP65_DEFAULT_SPI_HOST SPI2_HOST
#endif

/** Default host GPIO for managed MPU-6500 data-ready INT. */
#define ESP_MP65_DEFAULT_INT_GPIO CONFIG_ESP_MP65_PIN_INT

/** Accelerometer range selected by the Kconfig choice. */
#if CONFIG_ESP_MP65_ACCEL_RANGE_4G
#define ESP_MP65_DEFAULT_ACCEL_RANGE ESP_MP65_ACCEL_RANGE_4G
#elif CONFIG_ESP_MP65_ACCEL_RANGE_8G
#define ESP_MP65_DEFAULT_ACCEL_RANGE ESP_MP65_ACCEL_RANGE_8G
#elif CONFIG_ESP_MP65_ACCEL_RANGE_16G
#define ESP_MP65_DEFAULT_ACCEL_RANGE ESP_MP65_ACCEL_RANGE_16G
#else
#define ESP_MP65_DEFAULT_ACCEL_RANGE ESP_MP65_ACCEL_RANGE_2G
#endif

/** Gyroscope range selected by the Kconfig choice. */
#if CONFIG_ESP_MP65_GYRO_RANGE_500DPS
#define ESP_MP65_DEFAULT_GYRO_RANGE ESP_MP65_GYRO_RANGE_500DPS
#elif CONFIG_ESP_MP65_GYRO_RANGE_1000DPS
#define ESP_MP65_DEFAULT_GYRO_RANGE ESP_MP65_GYRO_RANGE_1000DPS
#elif CONFIG_ESP_MP65_GYRO_RANGE_2000DPS
#define ESP_MP65_DEFAULT_GYRO_RANGE ESP_MP65_GYRO_RANGE_2000DPS
#else
#define ESP_MP65_DEFAULT_GYRO_RANGE ESP_MP65_GYRO_RANGE_250DPS
#endif

/**
 * @brief Initializer using Kconfig defaults.
 *
 * It creates/owns the SPI host but does not reserve it exclusively. Override
 * either boolean per instance for an application-owned or exclusive bus.
 */
#define ESP_MP65_CONFIG_DEFAULT()                                              \
  {                                                                            \
      .spi_host = ESP_MP65_DEFAULT_SPI_HOST,                                   \
      .pin_sclk = CONFIG_ESP_MP65_PIN_SCLK,                                    \
      .pin_mosi = CONFIG_ESP_MP65_PIN_MOSI,                                    \
      .pin_miso = CONFIG_ESP_MP65_PIN_MISO,                                    \
      .pin_cs = CONFIG_ESP_MP65_PIN_CS,                                        \
      .clock_speed_hz = CONFIG_ESP_MP65_SPI_CLOCK_HZ,                          \
      .initialize_spi_bus = true,                                              \
      .acquire_bus_on_init = false,                                            \
      .accel_range = ESP_MP65_DEFAULT_ACCEL_RANGE,                             \
      .gyro_range = ESP_MP65_DEFAULT_GYRO_RANGE,                               \
      .dlpf_cfg = CONFIG_ESP_MP65_DLPF_CFG,                                    \
      .sample_rate_divider = CONFIG_ESP_MP65_SAMPLE_RATE_DIV,                  \
  }

/**
 * @brief Create, reset, identify, and configure one MPU-6500.
 *
 * If initialize_spi_bus is true, the host must be unused and becomes owned by
 * the handle. Otherwise the caller must initialize and later free it. If
 * acquire_bus_on_init is true, other devices on that host remain blocked until
 * release or deinit. This function allocates, may block, and requests 100 ms
 * and 10 ms settling delays quantized to RTOS ticks; never call it from a
 * periodic real-time iteration. Failure releases resources acquired by this
 * call on normal cleanup paths. Invalid config is
 * rejected before writing out_handle; initialize your handle to NULL. After
 * config/output validation, later failures leave *out_handle NULL.
 *
 * @param config Valid configuration; not retained after the call.
 * @param out_handle Receives the new component-owned handle.
 * @return ESP_OK; ESP_ERR_INVALID_ARG; ESP_ERR_NO_MEM; ESP_ERR_NOT_FOUND if
 *         WHO_AM_I is not 0x70; or an ESP-IDF SPI error.
 */
esp_err_t esp_mp65_init(const esp_mp65_config_t *config,
                        esp_mp65_handle_t *out_handle);

/**
 * @brief Release a reservation, remove the device, free an owned host and
 * handle.
 *
 * Serialize this call with every other operation and never use the handle after
 * complete success. A cleanup error is returned before the handle is freed, so
 * the caller can diagnose it. Cleanup is not transactional: device removal can
 * precede host-free failure. Blind retry/resumed reads are unsafe after partial
 * SPI teardown. Stop other devices on an owned host first. Active managed INT
 * also requires cleanup on its start core.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG, or managed-INT/GPIO/SPI cleanup error.
 */
esp_err_t esp_mp65_deinit(esp_mp65_handle_t handle);

/**
 * @brief Read WHO_AM_I into caller-owned storage (expected MPU-6500 value
 * 0x70).
 * @return ESP_OK, ESP_ERR_INVALID_ARG, optional-mutex ESP_ERR_TIMEOUT, or an
 *         ESP-IDF SPI transaction error.
 */
esp_err_t esp_mp65_read_who_am_i(esp_mp65_handle_t handle, uint8_t *who_am_i);

/**
 * @brief Read and convert acceleration, temperature, and angular velocity.
 *
 * The call performs one synchronous 15-byte polling SPI transaction, allocates
 * no memory, and writes sample only after that transaction succeeds. The SPI
 * polling API has no transaction timeout; validate failure recovery when a
 * bounded application deadline is required.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG, optional-mutex ESP_ERR_TIMEOUT, or an
 *         ESP-IDF SPI transaction error.
 */
esp_err_t esp_mp65_read(esp_mp65_handle_t handle, esp_mp65_sample_t *sample);

/** Configure the sensor INT output for active-high, push-pull, 50 us data-ready
 * pulses, or disable it. Called by the application in task context, outside
 * the acquisition loop; never from an ISR. No ESP32 GPIO is configured here.
 * Init leaves interrupts disabled. Enabling clears pending sensor status first
 * and disables other interrupt sources and FSYNC routing. Configure the host
 * GPIO/ISR before enabling. Disable before removing that ISR or deinitializing.
 * Returns INVALID_STATE while the managed data-ready mode is active; use
 * esp_mp65_data_ready_stop() then. Serialized by the optional handle mutex.
 * No allocation. On SPI failure the
 * hardware state is uncertain; caller must handle the returned error.
 * @param handle Initialized sensor handle.
 * @param enabled True to enable RAW_RDY_EN; false disables sensor interrupts.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE in managed mode,
 *         mutex ESP_ERR_TIMEOUT, or SPI error.
 */
esp_err_t esp_mp65_set_data_ready_interrupt(esp_mp65_handle_t handle,
                                            bool enabled);

/** Optional host-side INT configuration. Start/stop must run in a task pinned
 * to the ISR service's core. Lifecycle calls must be serialized with all users.
 */
typedef struct {
  int gpio_num; /**< Dedicated free input GPIO wired to sensor INT. */
  bool install_isr_service; /**< true: install a new, exclusive non-IRAM GPIO
                             * service on the calling core; an existing service
                             * is an error. No other driver may attach until
                             * stop. false: borrow an application-owned service
                             * installed on the calling core without
                             * ESP_INTR_FLAG_IRAM. The application guarantees
                             * that core/flags and keeps the service alive until
                             * stop. */
} esp_mp65_data_ready_config_t;

/** Latest observed ISR event, not a buffered sensor sample. */
typedef struct {
  uint32_t sequence; /**< Starts at 1 on each start; wraps modulo 2^32. */
  int64_t isr_us;    /**< esp_timer timestamp at ISR entry, microseconds. */
  int64_t gap_us;    /**< Interval from previous observed ISR, 0 for first. */
} esp_mp65_data_ready_event_t;

/** Configure GPIO and one-slot overwrite queue, then enable sensor RAW_RDY.
 * Task-context setup only; allocates but creates no worker task. No SPI/FPU in
 * ISR. GPIO must not be used by another driver/handler or by the SPI signals.
 * Existing GPIO configuration is not restored at stop. Requires a pinned task;
 * start, stop and deinit with active INT must use that same core.
 * Returns INVALID_ARG, INVALID_STATE (already started/service conflict),
 * NO_MEM, or GPIO/SPI error. On failure no host handler/queue remains; if SPI
 * failed, sensor INT state can be uncertain. Direct reads remain available.
 * @param handle Initialized handle, not already in managed mode.
 * @param config Required configuration, consumed synchronously, not retained.
 * @return ESP_OK, errors above, or optional mutex ESP_ERR_TIMEOUT.
 */
esp_err_t esp_mp65_data_ready_start(esp_mp65_handle_t handle,
                                    const esp_mp65_data_ready_config_t *config);
/** Wait for newest event from one consumer task. No SPI, allocation or mutex
 * held while waiting. timeout_ms=0 polls; UINT32_MAX waits indefinitely; other
 * timeouts round up to RTOS ticks. TIMEOUT/INVALID_STATE leaves output intact.
 * Caller computes sequence gaps to detect overwritten events. This does not
 * detect every lost hardware edge or preserve overwritten sensor register data.
 * Stop/deinit must not race a waiter; finish/join the consumer first.
 * @param handle Initialized handle with managed mode started.
 * @param event Required caller-owned output, copied on success only.
 * @param timeout_ms Poll, finite timeout, or UINT32_MAX as described above.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE when stopped,
 *         or ESP_ERR_TIMEOUT when no event arrives.
 */
esp_err_t esp_mp65_wait_data_ready(esp_mp65_handle_t handle,
                                   esp_mp65_data_ready_event_t *event,
                                   uint32_t timeout_ms);
/** Disable sensor INT, remove host handler and queue, uninstall only a service
 * exclusively installed by start. Idempotent when stopped. Caller must stop
 * all waiters first and run on the start core. Errors preserve resources for
 * retry; no further read/wait/start until cleanup succeeds after stop error.
 * @param handle Required initialized handle; NULL returns INVALID_ARG.
 * @return ESP_OK (also already stopped), INVALID_STATE for wrong/unpinned core,
 *         or GPIO/SPI/optional-mutex error (ESP_ERR_ prefixes).
 */
esp_err_t esp_mp65_data_ready_stop(esp_mp65_handle_t handle);

/**
 * @brief Reserve the SPI host for this device during a high-rate phase.
 *
 * Call once before, never inside, a deadline-sensitive loop. ESP-IDF permits
 * only an indefinite acquisition wait. Pair success with release or deinit.
 * Reservation prevents transactions from other devices; it neither prevents
 * task preemption nor replaces same-handle serialization.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE if already held,
 *         optional-mutex ESP_ERR_TIMEOUT, or an ESP-IDF SPI error.
 */
esp_err_t esp_mp65_acquire_bus(esp_mp65_handle_t handle);

/**
 * @brief Release a bus reservation made explicitly or during initialization.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE if not held, or
 *         optional-mutex ESP_ERR_TIMEOUT.
 */
esp_err_t esp_mp65_release_bus(esp_mp65_handle_t handle);

/** @name Common ownership and call graph
 * All APIs are external task-context entry points, never ISR calls. Only
 * esp_mp65_data_ready_stop() is also called internally (by esp_mp65_deinit()).
 * Handles must remain alive through the full call; NULL is invalid. Read
 * outputs are caller-owned, required, copied on success and never retained.
 * start/stop/deinit and manual INT reconfiguration require lifecycle
 * serialization even with the mutex. A queue wait must not race teardown.
 * Reservation is independent of same-handle synchronization.
 */

#ifdef __cplusplus
}
#endif
