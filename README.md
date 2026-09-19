# esp_mp65

`esp_mp65` is an ESP-IDF SPI driver for the MPU-6500 six-axis IMU. It resets
and configures the device, verifies `WHO_AM_I == 0x70`, and reads acceleration,
temperature, and angular velocity in one coherent register burst. It is not a
complete inertial-navigation or sensor-fusion solution; calibration, bias
estimation, axis remapping and attitude estimation remain application
responsibilities. Optional INT synchronization is supported; sensor FIFO is not.

## Wiring and electrical assumptions

The Kconfig defaults reproduce [`docs/pins.txt`](docs/pins.txt):

| ESP32-S3 | MPU-6500 module | Signal |
|---|---|---|
| GPIO13 | SCL/SCLK | SPI SCLK |
| GPIO12 | SDA/SDI | SPI MOSI |
| GPIO11 | AD0/SDO | SPI MISO |
| GPIO10 | NCS | active-low CS |
| GPIO4 | INT | optional data-ready output (component default) |
| 3V3 | VCC | supply |
| GND | GND | reference |

The driver assumes a 3.3 V-compatible module. Verify the exact module's
regulator and logic levels before connecting it; the component cannot detect an
electrical mismatch.

## Add the component from GitHub

The released component is available at
[`https://github.com/smartsensingme/esp_mp65.git`](https://github.com/smartsensingme/esp_mp65.git).
For a Component Manager application, add only the direct dependency to the
consuming component's `main/idf_component.yml`:

```yaml
dependencies:
  esp_mp65:
    git: https://github.com/smartsensingme/esp_mp65.git
    version: "v0.1.0"
```

The component's own `idf_component.yml` describes its ESP-IDF relationship;
the application manifest is the file that selects the released component.
For an offline/manual installation, clone the same HTTPS URL into an ESP-IDF
component search directory. Do not use a developer-specific SSH host alias in
documentation or manifests.

The consuming component still declares the link dependency:

```cmake
idf_component_register(SRCS "main.c" PRIV_REQUIRES esp_mp65)
```

Then include `esp_mp65.h`. In `idf.py menuconfig`, open **ESP MP65
(MPU-6500)** to choose the default host, pins, SPI clock, ranges, digital
low-pass filter, sample divider, and same-handle synchronization policy.

## Basic lifecycle

```c
esp_mp65_handle_t sensor = NULL;
esp_mp65_config_t config = ESP_MP65_CONFIG_DEFAULT();

ESP_ERROR_CHECK(esp_mp65_init(&config, &sensor));

esp_mp65_sample_t sample;
ESP_ERROR_CHECK(esp_mp65_read(sensor, &sample));
/* sample.accel_mps2[], sample.gyro_rads[], sample.temperature_c */

ESP_ERROR_CHECK(esp_mp65_deinit(sensor));
```

Initialization allocates a handle and, by default, initializes SPI without DMA,
adds one mode-0 device, resets the sensor, waits for startup, disables its I2C
interface, selects the X-axis gyroscope PLL, enables all axes, checks identity,
and writes the configured filter/range/rate registers. It is a setup operation,
not part of the deterministic read path. Deinitialization must be serialized
with all other calls; a successfully destroyed handle is invalid afterward.

## SPI ownership versus exclusive reservation

These are independent per-instance choices in `esp_mp65_config_t`:

| Setting | Meaning | Default |
|---|---|---|
| `initialize_spi_bus` | The handle initializes and later frees the SPI host. If false, the application owns an already initialized host. | `true` |
| `acquire_bus_on_init` | The device reserves the entire host, delaying all transactions from other devices until release/deinit. | `false` |

This policy deliberately is not global Kconfig state: two sensor instances in
one firmware may need different ownership and latency policies. Owning host
initialization also does not imply exclusive reservation.

Attach to an application-owned shared bus as follows:

```c
esp_mp65_config_t config = ESP_MP65_CONFIG_DEFAULT();
config.initialize_spi_bus = false;
config.acquire_bus_on_init = false;
ESP_ERROR_CHECK(esp_mp65_init(&config, &sensor));
```

For a temporary high-rate phase, reserve once outside the periodic loop:

```c
ESP_ERROR_CHECK(esp_mp65_acquire_bus(sensor)); /* may wait indefinitely */
while (control_phase_active) {
    ESP_ERROR_CHECK(esp_mp65_read(sensor, &sample));
    /* bounded estimation/control work */
}
ESP_ERROR_CHECK(esp_mp65_release_bus(sensor));
```

Retaining the bus avoids repeated arbitration/unblocking overhead and excludes
other devices, but does not prevent task preemption. Do not reserve a shared
host permanently unless excluding every other device is intentional. ESP-IDF's
acquisition API uses `portMAX_DELAY`, so acquire before the timed phase and
define how shutdown/error paths release it.

## Samples, units, and rate

`esp_mp65_read()` performs one address byte plus the 14 consecutive sensor data
bytes. Array order is X, Y, Z. It reports raw signed register values and:

- acceleration in m/s² using the selected sensitivity and 9.80665 m/s² per g;
- angular velocity in rad/s using the selected sensitivity and pi/180;
- temperature in °C using `raw / 333.87 + 21`.

These are datasheet conversions, not calibrated values. Range trades resolution
for headroom. With DLPF enabled at the default setting, the gyroscope base rate
is 1 kHz and `SMPLRT_DIV` gives `1000 / (1 + divider)` Hz. Consult the exact
MPU-6500 register map for bandwidth/output-rate behavior of every DLPF value.
The read API does not wait for a new-data indication, so repeated calls can return
the same conversion when called faster than the configured sensor output rate.

## Data-ready interrupt

`esp_mp65_set_data_ready_interrupt(handle, true)` configures the MPU-6500 INT
pin as its default active-high, push-pull, 50 µs raw-data-ready pulse. It clears
pending sensor status before enabling `RAW_RDY_EN`; it does not configure an
ESP32 GPIO or install an ISR. Configure the host input and ISR first, then call
the function from task context, outside the timing-critical loop. Call it with
`false` before removing the host ISR or destroying the handle.

Alternatively, the managed mode owns the host GPIO handler and a one-slot
overwrite queue. It creates no acquisition task:

```c
/* Run setup/cleanup on a task pinned to Core 1 in this example. */
esp_mp65_data_ready_config_t ready = {
    .gpio_num = ESP_MP65_DEFAULT_INT_GPIO,
    .install_isr_service = true,
};
ESP_ERROR_CHECK(esp_mp65_data_ready_start(sensor, &ready));
esp_mp65_data_ready_event_t event;
if (esp_mp65_wait_data_ready(sensor, &event, 100) == ESP_OK) {
    esp_mp65_sample_t sample;
    ESP_ERROR_CHECK(esp_mp65_read(sensor, &sample));
    /* Application owns sample processing and stale-event policy. */
}
/* Finish/join any waiter before stopping. */
ESP_ERROR_CHECK(esp_mp65_data_ready_stop(sensor));
```

Start allocates the queue before enabling RAW_RDY; the ISR only timestamps,
increments a sequence and overwrites the event. No SPI, allocation or floating
point occurs inside it. Wait performs no SPI and holds no handle mutex. It has
one consumer and accepts a timeout in milliseconds: zero polls, UINT32_MAX
waits indefinitely, other values round up to ticks. Sequence starts at 1 after
each start and wraps; unsigned `sequence - previous - 1` counts overwritten
observed events. `isr_us` is ISR-entry time, not conversion time, and `gap_us`
is the observed ISR interval. These counters cannot detect every missed edge.
Queued events do not preserve old sensor samples. Late-event policy, deadlines,
retry/error recovery and sensor ODR selection remain in the application.

`install_isr_service=true` installs a new non-IRAM service on the calling core;
an existing service is rejected. This service must be exclusive: do not attach
other devices while it is owned by this handle. Stop uninstalls it.
For multiple devices/shared GPIO service, the application must install a
non-IRAM service first, then use `install_isr_service=false` for each handle.
Borrowed services are never uninstalled by the driver. The caller must ensure
the borrowed service was installed on the same core as start/stop; ESP-IDF's
public GPIO API does not expose its allocation core or flags for verification.
The INT GPIO must be free and not have an existing handler (adding a handler
could otherwise replace it). This ownership is an application precondition.

Start/stop require a pinned task; stop/deinit with active INT must run on the
start core. This keeps removal and ISR execution on the same core, so state
can be released after handler removal. Serialize lifecycle calls across users,
even in thread-safe builds. Stop/join waiters before stopping; it does not
cancel a blocked waiter. Stop first disables the sensor, then removes its
handler/queue and an owned service. Cleanup errors retain state for retry;
do not resume operation until cleanup succeeds. Deinit invokes stop before
freeing SPI resources. GPIO configuration is not restored. No cache-off/IRAM
guarantee is made. The original low-level INT setter rejects managed-active
handles; it remains available for applications with their own ISR.

`mp65_attitude` now uses this API on Core 1 with GPIO4 and 100 ms wait timeout.
Its nominal 1 kHz/1 ms budget and stale-event rejection remain unchanged. INT
does not raise the sensor's hardware output rate or require exclusive SPI.
Hardware validation after this migration (sustained timing, start/stop/restart,
borrowed-service coexistence and disconnected INT) is still pending.

The optional FSYNC pin is a sensor input for timestamping
an external event into a selected data-register bit; it is not a data-ready
output or a conversion clock.

## Concurrency and deterministic-loop behavior

`CONFIG_ESP_MP65_THREAD_SAFE=y` creates one mutex per handle. Read, identity,
acquire, and release operations on that handle are serialized with the bounded
`CONFIG_ESP_MP65_LOCK_TIMEOUT_MS`. Different handles have independent mutexes.
Initialization and deinitialization still require application-level lifecycle
serialization.

Disable thread safety only when one task exclusively owns the handle; otherwise
same-handle concurrent calls are unsafe. The read path allocates no memory and
uses a fixed-size polling transaction, but ESP-IDF's polling SPI call has no
component-level timeout. A hard timing claim therefore requires measurement
under representative load and a defined peripheral-fault response. Logging,
initialization delays, allocation, and bus acquisition belong outside the
critical loop.

## Configuration reference

- **SPI host:** SPI2 or SPI3. Confirm it is not claimed by another subsystem.
- **SCLK/MOSI/MISO/CS/INT:** defaults from `docs/pins.txt`. INT is a default for
  the optional managed data-ready service; the actual GPIO is still supplied
  per instance through `esp_mp65_data_ready_config_t`, so applications can
  use different pins or multiple sensor handles. When attaching to an
  existing bus, the application is responsible for matching its bus pins;
  this driver only validates CS in that mode. The component validates the
  managed INT GPIO and rejects conflicts with the handle's SPI signals.
- **SPI clock:** 100 kHz to 1 MHz. The 1 MHz maximum keeps configuration writes
  within the sensor's SPI limit.
- **Accelerometer/gyroscope ranges:** configure both registers and conversion
  factors.
- **DLPF:** raw register value 0 through 7, written to gyroscope and
  accelerometer filter fields.
- **Sample divider:** raw 8-bit `SMPLRT_DIV` value.
- **Thread safety:** compile-time because it changes handle layout and code;
  all translation units must use the same generated `sdkconfig.h`.

Bus initialization and reservation are runtime fields, not Kconfig options.

## Errors and troubleshooting

`ESP_ERR_INVALID_ARG` indicates an invalid pointer, host, GPIO, clock, range, or
DLPF value. `ESP_ERR_NO_MEM` indicates handle/mutex allocation failure.
`ESP_ERR_NOT_FOUND` means communication succeeded but identity was not `0x70`;
check the part number and wiring. `ESP_ERR_TIMEOUT` is the optional mutex timeout,
not an SPI wire timeout; managed INT wait also returns TIMEOUT when no event
arrives. `ESP_ERR_INVALID_STATE` from acquire/release usually
means the requested reservation state already exists. Other errors come from
the ESP-IDF SPI/GPIO drivers. Managed start also allocates its one-slot queue,
so it can return NO_MEM. A wrong/unpinned lifecycle core, already started mode,
missing borrowed service or existing service in exclusive mode can cause
INVALID_STATE. Validate the specific failed API, not just the error name.

Initialize output handles to NULL: invalid config is rejected before init
writes the output. After config/output validation, subsequent setup failures
leave it NULL and attempt reverse-order cleanup. Cleanup errors during setup
are not returned separately from the original failure.

Deinit is not transactional. If SPI device removal succeeds and owned-host
freeing fails (for example another attached device remains), the retained
handle is not safe for resumed reads or blind retry. Arrange host teardown
beforehand and treat partial SPI cleanup as a lifecycle fault. This differs
from managed `data_ready_stop`, whose retained state supports cleanup retry.
Do not confuse either with automatic sensor/driver recovery; none is provided.

The optional mutex uses truncated RTOS-tick conversion, so a configured timeout
shorter than a tick can become a nonblocking attempt. Managed event waits instead
round finite milliseconds upward. Neither timeout bounds SPI wire duration.

## Verification and compatibility

All public functions are task-context APIs. Only `data_ready_stop` is also
called internally, by deinit; the ISR callback is private. Existing polling and
manual INT APIs remain available; no public API was removed by managed INT.
There is no portable host test for the physical SPI/GPIO lifecycle. Build both
examples and validate wiring, identity, new-data timing, start/stop/restart,
borrowed-service coexistence and timeout/error recovery on the actual board.

```sh
idf.py -C examples/read_sensor set-target esp32s3
idf.py -C examples/read_sensor build
idf.py -C examples/mp65_attitude build
```

Run these from the containing project with ESP-IDF activated. `set-target`
changes that example's build configuration; keep intentional local settings.
Observed latency maxima are measurements, not hard real-time bounds. No task
or complete SPI execution path is certified cache-safe by this component.

Common checks:

- confirm common ground, 3.3 V compatibility, NCS, and SDI/SDO orientation;
- confirm the selected SPI host and GPIOs are not already initialized/claimed;
- set `initialize_spi_bus=false` when the application owns that host;
- do not expect another SPI device to run while this handle has reserved it;
- compare raw values before diagnosing unit conversion or sensor calibration.

The repository includes a self-contained two-second smoke-test application in
[`examples/read_sensor`](examples/read_sensor). It is intentionally not a
deterministic control loop.
