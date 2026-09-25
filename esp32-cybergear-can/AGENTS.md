# ESP32-S3 + CyberGear CAN example

This is an advanced developer experiment. Keep its vendor-specific 1 Mbit/s protocol, 24 V power boundary, and acceptance procedure separate from incompatible CAN networks and beginner-facing APIs.

When changing this example:

- keep motion disabled by default and require an explicit configuration opt-in;
- preserve fail-closed stop, feedback checks, and the combined pre-transmission controller-effort check;
- run its host protocol tests and an ESP32-S3 ESP-IDF build;
- update both English and Simplified Chinese documentation when wiring, configuration, behavior, or safety boundaries change;
- never present a successful build or host test as physical motor validation.

From the repository root, run the host checks with:

```bash
cmake -S esp32-cybergear-can/test \
  -B esp32-cybergear-can/build-host
cmake --build esp32-cybergear-can/build-host
ctest --test-dir esp32-cybergear-can/build-host --output-on-failure
```

Build the firmware after loading ESP-IDF 5.5.x:

```bash
cd esp32-cybergear-can
idf.py set-target esp32s3
idf.py build
```
