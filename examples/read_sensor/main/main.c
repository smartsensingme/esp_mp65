#include "esp_err.h"
#include "esp_log.h"
#include "esp_mp65.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mp65_example";

void app_main(void) {
  const esp_mp65_config_t config = ESP_MP65_CONFIG_DEFAULT();
  esp_mp65_handle_t sensor = NULL;
  ESP_ERROR_CHECK(esp_mp65_init(&config, &sensor));

  while (true) {
    esp_mp65_sample_t sample;
    esp_err_t err = esp_mp65_read(sensor, &sample);
    if (err == ESP_OK) {
      ESP_LOGI(TAG,
               "accel [m/s^2] x=%+.3f y=%+.3f z=%+.3f | "
               "gyro [rad/s] x=%+.3f y=%+.3f z=%+.3f | temp=%.2f C",
               sample.accel_mps2[0], sample.accel_mps2[1], sample.accel_mps2[2],
               sample.gyro_rads[0], sample.gyro_rads[1], sample.gyro_rads[2],
               sample.temperature_c);
    } else {
      ESP_LOGE(TAG, "sensor read failed: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
