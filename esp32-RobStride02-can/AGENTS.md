# ESP32-S3 + RobStride RS02 CAN example

This is an advanced 48 V actuator experiment, not an xbot Robot Bus V1 implementation. Keep its RS02-specific 1 Mbit/s protocol and safety boundary separate from 250 kbit/s Robot Bus examples.

When changing this example:

- keep commanded motion separately opt-in; require explicit user authorization before adding or increasing a profile, and record its RS02-specific physical validation scope;
- preserve the fail-closed stop, exact motor/host identity checks, feedback guards, and fatal TWAI alert handling;
- run the host tests and an ESP32-S3 ESP-IDF build;
- update both English and Simplified Chinese documentation when wiring, configuration, behavior, or safety boundaries change;
- never present a successful build or host test as physical motor validation.

From the repository root, run:

```bash
cmake -S esp32-RobStride02-can/test \
  -B esp32-RobStride02-can/build-host
cmake --build esp32-RobStride02-can/build-host
ctest --test-dir esp32-RobStride02-can/build-host --output-on-failure
```

Build the firmware after loading ESP-IDF 5.5.x:

```bash
cd esp32-RobStride02-can
idf.py set-target esp32s3
idf.py build
```
