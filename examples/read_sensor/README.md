# MPU-6500 read example

This example initializes the sensor with the component's Kconfig defaults and
prints acceleration, angular velocity, and temperature every two seconds.

It is an integration/smoke-test example, not a deterministic control loop: it
logs synchronously and uses `vTaskDelay()`, so the interval is relative rather
than an absolute hardware-timed deadline. The default instance initializes and
owns SPI2, while leaving the bus unreserved for other devices.

```sh
idf.py set-target esp32s3
idf.py menuconfig
idf.py build flash monitor
```

The default wiring is GPIO13 SCLK, GPIO12 MOSI, GPIO11 MISO, and GPIO10 CS.
Connect the module to 3.3 V and ground as documented in
[`docs/pins.txt`](../../docs/pins.txt).

Expected output contains SI units for all three axes and temperature, for
example `accel [m/s^2] ... gyro [rad/s] ... temp=... C`. Initialization aborts
through `ESP_ERROR_CHECK` if SPI setup, identity, or configuration fails; later
read failures are logged and the loop continues. Press the monitor's quit key
to stop the example. Because `app_main()` never exits its loop, explicit
deinitialization is intentionally not demonstrated; finite-lifetime code must
call `esp_mp65_deinit()` after stopping all users.
