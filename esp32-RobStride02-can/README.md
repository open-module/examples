[简体中文](README.zh-CN.md)

# ESP32-S3 + RobStride RS02 CAN example

This standalone ESP-IDF example exercises the RobStride RS02 private CAN protocol through an ESP32-S3 and a 3.3 V CAN transceiver. The default build performs no commanded movement. Separate configurations provide a Wi-Fi web console with click confirmation and an opt-in 60-degree test that runs once at boot.

The implementation follows the RS02 user manual dated 2026-07-13: Classical CAN at 1 Mbit/s, 29-bit extended identifiers, and eight-byte payloads. The ESP32 controller uses 20 time quanta per bit and an 80% sample point; the manual specifies the bit rate, not that controller-side sample point.

Source baseline:

- [`RS02 User Manual 260713.pdf`](https://github.com/RobStride/Product_Information/blob/0f4ad74fdb67023e75bbcbeebecd6f1a003ce000/Product%20Literature/RS02/RS02User%20Manual260713.pdf), SHA-256 `87baa04792996978d1d4edc4176cd583e695e8bc09a65f97fc982b2e0a7f71b7`.
- Product Information revision `0f4ad74fdb67023e75bbcbeebecd6f1a003ce000`.
- Official [`RobStride/SampleProgram`](https://github.com/RobStride/SampleProgram/tree/5f598686b05fcc527ee0fc0ea954f0afd652b234), revision `5f598686b05fcc527ee0fc0ea954f0afd652b234`.

## Safety boundary

RS02 is a high-torque actuator with a nominal 48 V supply and documented 24–60 V operating range. Before powering it:

- Unload and secure the motor with the output clear. Do not test on an occupied or safety-critical mechanism.
- Use a current-limited bench supply and keep a physical motor-power cutoff within reach.
- Keep motor power off while wiring, flashing, reconnecting USB, or measuring termination.
- Connect motor voltage only to RS02 `VBAT+`; never connect it to the ESP32 or transceiver supply.
- A non-isolated CAN transceiver requires a common signal ground with motor supply negative. Use an appropriate isolated transceiver if the grounds must remain separated.
- Check connector numbering from the manual, not connector orientation alone.

The software rejects unexpected IDs, malformed frames, non-finite values, nonzero feedback faults, type 21 faults/warnings, fatal TWAI alerts, feedback timeout, bus voltage outside 24–60 V, and temperature outside the provisional `[-20, 80) °C` range. The zero-effort check also uses fixed `0.5 rad/s`, `1.0 N·m`, and `0.10 rad` observation guards; motion mode derives its feedback-speed, feedback-torque, and estimated-effort guards from the speed and torque limits selected in the page while retaining separate tracking-error and endpoint checks. These guards are provisional software observations, not certified motor limits, current limiting, or power isolation. Loss of CAN or MCU power can prevent a stop command from reaching the motor.

## Wiring

ESP32-S3 to the CAN transceiver:

When using the same development board as the CyberGear example, see the local [SN65HVD230 board references](../hardware/SN65HVD230-CAN-Board/) for its schematic, fixed termination, and limitations.

| ESP32-S3 | CAN transceiver |
|---|---|
| GPIO4 | TX / D |
| GPIO5 | RX / R |
| 3V3 | 3.3 V supply |
| GND | signal ground |

Transceiver and power to the RS02 XT30PB (2+2) connector:

| RS02 pin | Signal | Connection |
|---|---|---|
| 1 | VBAT+ | Current-limited motor supply positive |
| 2 | GND | Motor supply negative; signal ground for a non-isolated transceiver |
| 3 | CAN_L | Transceiver CANL |
| 4 | CAN_H | Transceiver CANH |

Use a short twisted CANH/CANL pair and proper 120 ohm termination at both bus endpoints. With the complete bus unpowered, two simple endpoint terminators normally measure about 60 ohms across CANH and CANL. Stop if the reading or wiring is uncertain.

## Protocol coverage

The example implements and tests:

- device-ID request (communication type 0);
- zero-effort operation control (type 1), using big-endian position, velocity, Kp, and Kd fields and torque in identifier bits 23..8;
- feedback parsing (type 2), including mode, fault bits, position, velocity, torque, and temperature;
- enable and stop (types 3 and 4);
- float parameter reads (type 17) for load-side cycle-counting mechanical position `0x7019` and bus voltage `0x701C`;
- type 18 parameter writes and readback for `run_mode` (`0x7005`) and the temporary motion torque limit (`0x700B`);
- fault feedback (type 21).

The manual's type 21 prose and identifier example swap the motor/host bytes. The parser accepts either layout only when both bytes exactly match the configured motor and host IDs. It does not accept arbitrary frames.

The operation-control ranges are position `-12.5..12.5 rad`, velocity `-44..44 rad/s`, torque `-17..17 N·m`, Kp `0..500`, and Kd `0..5`. The position scaling follows the official SampleProgram implementation; the manual describes the nominal range as `-4π..4π` and its embedded sample uses `±12.57`. The default and link-soak configurations set desired velocity, feed-forward torque, Kp, and Kd to zero.

## Host tests

From the repository root:

```bash
cmake -S esp32-RobStride02-can/test \
  -B esp32-RobStride02-can/build-host
cmake --build esp32-RobStride02-can/build-host
ctest --test-dir esp32-RobStride02-can/build-host --output-on-failure
```

The tests cover exact wire identifiers and byte order, clamping, identity validation, parameter floats, both documented type 21 layouts, non-finite rejection, safety boundaries, and TWAI timing. Passing tests do not validate physical RS02 behavior.

## Firmware build and behavior

Load ESP-IDF 5.5.x and build for ESP32-S3:

```bash
. /path/to/esp-idf/export.sh
cd esp32-RobStride02-can
idf.py set-target esp32s3
idf.py build
```

Default IDs are motor `0x7F` and host `0xFD`; default TWAI pins are GPIO4/GPIO5. They can be changed under `RobStride RS02 CAN example` in `idf.py menuconfig`.

On each boot the firmware runs one fail-closed sequence:

1. Send stop and require matching Reset-mode feedback.
2. Request and validate the device ID.
3. Read and validate bus voltage, then read `mechPos`.
4. Enable and require matching Motor-mode feedback.
5. For 500 ms at nominal 50 Hz, keep the measured initial position in the position field while sending desired velocity, feed-forward torque, Kp, and Kd all as zero. Every response is checked.
6. Send stop, require Reset-mode feedback, and shut TWAI down.

Zero Kp and Kd mean the position field does not actively hold position. The motor is still enabled during the test, so unexpected motion remains possible.

For a longer link-only diagnostic, build with `sdkconfig.link-soak.defaults`; it extends the same zero-effort phase to 60 seconds without changing the command terms:

```bash
idf.py -B build-link-soak \
  -D SDKCONFIG=build-link-soak/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.link-soak.defaults' \
  set-target esp32s3
idf.py -B build-link-soak \
  -D SDKCONFIG=build-link-soak/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.link-soak.defaults' \
  build
```

## Wi-Fi motion console

The web console is intended for repeated bench tests with adjusted parameters. The ESP32-S3 creates its own WPA2 access point and serves the complete page locally, with no internet or external service required. Use a separate build directory:

```bash
idf.py -B build-web \
  -D SDKCONFIG=build-web/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.web-console.defaults' \
  set-target esp32s3
idf.py -B build-web \
  -D SDKCONFIG=build-web/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.web-console.defaults' \
  build
```

After flashing, the ESP32 advertises an access point named `RS02-BENCH-xxxx`. The default password is `rs02-bench`; connect to it and open `http://192.168.4.1`. The password can be changed in `menuconfig`. Web mode no longer uses the BOOT button, and the access point allows only one connected client.

The page uses three sliders for absolute single-turn target angle, one-way travel time, and temporary motor torque limit. Slider changes update the page immediately and start a no-motion preflight automatically after a short debounce:

- the single-turn angle slider spans `0–360°` and represents an absolute load-side target referenced to the encoder origin, not a travel amount. Automatic preflight reads `0x7019 mechPos`, normalizes it to the current `0–360°` position, and commands only `target - current`: for example, moving from `45°` to `90°` travels `+45°`, while moving from `90°` to `45°` travels `-45°`. The page and confirmation layer show the current angle, target angle, direction, and actual signed travel. The two slider endpoints are deliberately distinct: `0° → 360°` commands one positive turn, `360° → 0°` commands one negative turn, and equal endpoints hold position. This prevents an exact positive full-turn encoder reading from being folded back to `0°`;
- the one-way travel-time slider spans `0.3–30.0 s`. The `0.3 s` lower bound is checked against a full `360°` smooth trajectory and the protocol's `33 rad/s` speed range. The 30-second upper bound is a practical bench-UI window, not a physical maximum motor run time;
- the page no longer exposes a speed slider. Firmware calculates theoretical peak speed from angle and duration and automatically adds a `0.25 rad/s` feedback-abort margin. Automatically calculated guards above the physically observed `0.8 rad/s` boundary still require an extra browser acknowledgement. The manual's `0x7017 limit_spd` belongs to `run_mode=5` CSP position mode; this example uses `run_mode=0` operation control and therefore does not misrepresent that parameter as a motor-side limit for the active mode;
- the torque slider spans `0.1–14.0 N·m` and writes parameter `0x700B`. The manual specifies `6 N·m` rated load and `17 N·m` peak load, but the writable `0x700B limit_torque` range is only `0–14 N·m`; this example excludes zero because it cannot execute position control. The setting is both the motor torque limit and software estimated-effort limit, not a constant torque command; feed-forward torque remains zero. Values above `1 N·m` require an extra browser acknowledgement, and values above `6 N·m` are explicitly marked above-rated.

After a slider change, the browser automatically submits the three parameters and the firmware performs a no-motion preflight: send stop, confirm Reset mode, identify the device, check bus voltage, read `mechPos` and control feedback, and resolve the absolute single-turn target into the required signed delta and protocol endpoint. Preflight never enables the motor. After it passes, the page shows the current single-turn angle, planned direction, and absolute target. Selecting “Start Test” opens a confirmation layer; one explicit confirmation click within 30 seconds makes the firmware recheck the latest angle, recompute the remaining delta, run one trajectory, settle for 700 ms, and send stop without returning to the origin. The server issues a one-time motion-plan token and independently requires the high-risk acknowledgement when the resolved speed or torque exceeds the validated bench envelope, so a bodyless or stale start request cannot bypass the confirmation. The sliders stay locked from accepted confirmation through the final motion state. Automatic preflight and motion start remain separate HTTP requests, so adjusting a slider, refreshing the page, reconnecting Wi-Fi, or pressing RESET cannot start motion. During preflight and motion, the browser sends a heartbeat every 500 ms; if it is absent for more than 2 seconds, or the web Stop button is selected, the control loop aborts and attempts to send a stop command.

The final stop leaves the motor in Reset mode; it does not actively hold the target with torque. The web Stop button, heartbeat, and CAN stop frame are not hardware emergency stops. Keep a physical motor-power cutoff immediately available. Every completed or aborted run requires a fresh automatic preflight and click confirmation. The web mode has host-test and ESP32-S3 build coverage, but this one-way browser trigger path, settings above `0.8 rad/s` or `1 N·m`, and arbitrary user-selected profiles had not completed physical-motion acceptance when this note was updated.

## Opt-in 60-degree motion test

Only use this configuration with the RS02 unloaded and secured, at least 60 degrees of clear motion in either possible commanded direction, a current-limited motor supply, and a physical cutoff ready. Build it in a separate directory:

```bash
idf.py -B build-motion \
  -D SDKCONFIG=build-motion/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.motion-test.defaults' \
  set-target esp32s3
idf.py -B build-motion \
  -D SDKCONFIG=build-motion/sdkconfig \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.motion-test.defaults' \
  build
```

While stopped, the firmware first writes `run_mode=0` and confirms it by reading `0x7005`, matching the official SampleProgram. The motion build also writes and confirms a temporary `1.0 N·m` limit at `0x700B`; the manual documents type 18 writes as lost on power failure. After the normal 500 ms zero-effort check, it captures the latest feedback position, follows a smooth 60-degree trajectory over 3 seconds, settles for 700 ms, returns over 3 seconds, settles again, then sends stop and requires Reset-mode feedback. It runs only once per boot. Kp is `12.0`, Kd is `1.0`, feed-forward torque is zero, the trajectory's theoretical peak target speed is about `0.524 rad/s`, and the software speed guard is `0.8 rad/s`.

The motion phase aborts when absolute speed reaches `0.8 rad/s`, feedback torque reaches `1.5 N·m`, estimated outgoing controller effort reaches `1.0 N·m`, tracking error reaches `0.075 rad`, endpoint error remains at or above `0.035 rad`, feedback times out, a fault appears, or TWAI reports a fatal alert. These checks are delayed software observations and estimates, not hard limits. Disconnect motor power immediately on any abnormal behavior.

## Controlled flash and acceptance

Flash with motor power disconnected. Start the serial monitor before deliberately applying current-limited motor power and resetting the ESP32. Keep the motor unloaded and the cutoff ready throughout.

A successful communication check requires a matching device ID, credible 24–60 V bus voltage, a finite `mechPos`, Reset mode after both stops, Motor mode during active control, no fault/warning or fatal TWAI alert, and final TWAI shutdown. A motion-test result additionally requires visible movement in both directions and endpoint checks to pass. Disconnect motor power immediately on timeout, unexpected direction or magnitude, noise, vibration, high current, invalid telemetry, or an unconfirmed stop.

The code has host-test and firmware-build coverage. The earlier zero-effort sequence completed on one RS02. In the first supervised motion attempt (`Kp=8.0`, `Kd=0.3`, 3-second leg), the motor did not move under command; manual shaft rotation then produced `0.268 rad/s` feedback, crossed the then-current `0.25 rad/s` guard, and caused a confirmed stop with no motor fault. That attempt is not motion validation. A later supervised unloaded 5-degree run using `Kp=12.0`, `Kd=1.0`, and 5-second legs was visibly confirmed to move and return. Its logged outbound and return endpoint errors were `-0.0186 rad` and `+0.0065 rad`; maximum observed absolute speed, feedback torque, estimated effort, and temperature were `0.193 rad/s`, `0.271 N·m`, `0.457 N·m`, and `32 °C`, with no reported fault and a confirmed final stop. Two subsequent 30-degree, 8-second attempts were intentionally stopped early by the same `0.25 rad/s` speed guard when feedback reached `0.314 rad/s` and `0.312 rad/s`; the observed outbound displacements before stopping were about 6.7 and 3.1 degrees, with no reported fault and confirmed stops. The current automatic-motion build's 60-degree, 3-second profile with a `0.8 rad/s` speed guard then completed one supervised unloaded out-and-back run with visible movement. Logged outbound and return endpoint errors were `-0.0092 rad` and `+0.0069 rad`; maximum logged absolute speed, feedback torque, estimated effort, and temperature were `0.790 rad/s`, `0.363 N·m`, `0.556 N·m`, and `33 °C`, with no reported fault and a confirmed final stop. This validates only that fixed-profile automatic-motion build; the web console and other parameter combinations still require separate controlled physical tests. It does not establish mechanical, thermal, electrical, firmware, or occupied-system safety.
