[简体中文](README.zh-CN.md)

# ESP32-S3 + CyberGear CAN Experiment

This standalone ESP-IDF example is intended to validate a physical ESP32-S3-DevKitC-1 N16R8, SN65HVD230 transceiver, and CyberGear motor link.

It uses the vendor protocol: Classical CAN at 1 Mbit/s with 29-bit extended identifiers. The ESP32-S3 timing is explicit: 20 time quanta per bit with an 80% sample point, matching the working STM32 comparison rather than the ESP-IDF ESP32-S3 default timing. It is not an xbot Robot Bus V1 example and cannot share a CAN bus with 250 kbit/s Robot Bus nodes.

## Safety boundary

CyberGear is a 24 V actuator capable of substantial torque. For the first test:

- Unload and secure the motor with its output clear; never start on a wheelchair or occupied mechanism.
- Use an independent, current-limited 24 V bench supply and keep a physical power cutoff within reach.
- Keep motor power off while wiring or measuring termination.
- Connect 24 V only to the motor `VBAT+`; never to the ESP32 or transceiver 3.3 V rail.
- This board is not isolated, so motor supply negative, transceiver GND, and ESP32 GND must be common.
- Motion is disabled by default and requires a clear full-turn sweep. Never stall the shaft by hand.

Software aborts on feedback timeout, type 2 status faults, type 21 fault/warning frames, temperature at or above 70 C, and fatal TWAI alerts. Every active phase requires Motor-mode feedback and finite in-range position, velocity, and torque. Absolute measured velocity must remain below `0.5 rad/s` during zero effort or `1.0 rad/s` during Motion. Absolute measured feedback torque must remain below `1.0 N m` during zero effort or `10.0 N m` during Motion. Position error from the current target must remain below `0.10 rad` (about `5.7 deg`). Reaching any threshold aborts both before transmission and after feedback. Before every transmission, the firmware also estimates `feed-forward + Kp * (target - measured position) + Kd * (desired - measured velocity)` and requires its absolute value to remain strictly below the phase torque threshold. Motion checks the current interpolated target. These are software estimates and delayed observations, not hard torque, speed, or power limits.

On an abnormal exit the program attempts stop and waits briefly for matching Reset-mode feedback before shutting TWAI down. Normal initial/final stops also require Reset mode; a Motor-mode response does not confirm a stop. These provisional software checks are not hard speed/position limits and do not replace mechanical restraint, current limiting, or a physical cutoff. Losing CAN or MCU power may prevent the stop from reaching the motor; TWAI shutdown is not motor power isolation.

## Wiring

ESP32-S3 to the SN65HVD230 board:

| ESP32-S3 | SN65HVD230 board | Schematic connector |
|---|---|---|
| GPIO4 | CAN_TX / D | P1 pin 1 |
| GND | GND | P1 pin 2 |
| 3V3 | 3.3V | P1 pin 3 |
| GPIO5 | CAN_RX / R | P1 pin 4 |

Transceiver, motor, and supply:

| SN65HVD230 / supply | CyberGear integrated connector |
|---|---|
| P2 pin 2 CANH | pin 4 CAN_H |
| P2 pin 1 CANL | pin 3 CAN_L |
| ESP32 / transceiver GND | pin 2 supply negative |
| External +24 V | pin 1 VBAT+ |

Use a short twisted CANH/CANL pair, preferably no longer than 1 m for this experiment. Verify continuity against the schematics before power-up; do not infer pins from connector orientation alone.

## Termination

The board has a fixed `120 ohm` resistor between CANH and CANL and no disable jumper, so it belongs at a bus endpoint.

With the complete bus unpowered, measure CANH to CANL:

- About `60 ohm`: two endpoint terminators are present.
- About `120 ohm`: only one simple terminator is visible to the meter. Confirm the motor's documented termination before changing the bus.
- Open circuit, an unstable reading, or a substantially different value: stop and inspect wiring and termination. Unpowered motor electronics can make the reading nonlinear or time-varying; do not calculate or add a terminator from an unstable resistance measurement alone.

## Host protocol and motion tests

From this repository root:

```bash
cmake -S esp32-cybergear-can/test \
  -B esp32-cybergear-can/build-host
cmake --build esp32-cybergear-can/build-host
ctest --test-dir esp32-cybergear-can/build-host --output-on-failure
```

The tests cover the corrected torque field in identifier bits 23..8, enable/stop identifiers, physical-value encoding and clamping, device/status/fault-feedback parsing, invalid frames, and non-finite control inputs. Motion tests cover ten-second full-turn timing/endpoints, reversed direction near the protocol limit, invalid inputs, phase-specific speed guards, the strict `10.0 N m` feedback and combined command-effort boundaries, invalid limits, tracking-error/mode guards, and the zero-effort diagnostic profile. A separate test derives the 1 Mbit/s rate and 80% sample point from the timing used by the firmware. Host tests and successful firmware builds do not validate physical motor behavior.

## Default build: communication and zero effort only

Load ESP-IDF 5.5.x, then run:

```bash
. /path/to/esp-idf/export.sh
cd esp32-cybergear-can
idf.py set-target esp32s3
idf.py build
```

Verify `CONFIG_CYBERGEAR_RUN_MOTION_TEST` is disabled, including when reusing a local `sdkconfig` from an earlier motion build. Follow the controlled flash/start procedure below; replace `PORT` with the actual serial port, for example `/dev/cu.usbmodem1101`.

The default sequence sends stop immediately after TWAI starts, requires Reset-mode feedback, requests the device ID, enables the motor, requires Motor-mode feedback, sends zero torque/Kp/Kd at the measured position for 500 ms, then stops and shuts TWAI down. A missing or unsafe response aborts the sequence.

## 60-second zero-effort CAN link soak

Use this diagnostic build before another motion test. It keeps Motion disabled and extends only the stationary zero-effort phase to 60 seconds. The measured initial position remains the target; desired velocity, feed-forward torque, Kp, and Kd remain zero. Frames and matching feedback are exercised at a nominal 50 Hz while all existing identity, mode, fault, temperature, speed, feedback-torque, tracking-error, timeout, and fatal-TWAI checks remain active.

Build it without changing the project's ordinary `sdkconfig`:

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

The startup log must report `link_soak=ENABLED, motion=disabled`. Success requires the full soak, a final TWAI status line with `TEC=0`, `REC=0`, `tx_failed=0`, and `bus_errors=0`, confirmed Reset-mode stop feedback, and TWAI shutdown. Any fatal alert aborts immediately. This mode does not request displacement or effort, but it enables the motor and cannot prevent movement caused by external force or hardware faults; keep the motor unloaded and the 24 V cutoff ready.

## Explicitly enable the 360-degree test (10 seconds per leg)

Only after the default sequence succeeds with sane feedback and termination:

```bash
idf.py menuconfig
```

Under `CyberGear CAN example`, enable `Explicitly allow the 360-degree out-and-back test (10 seconds per leg)`, then run `idf.py build`. Follow the controlled flash/start procedure below, not a combined `flash monitor` command with the motor powered.

After zero effort, the opt-in sequence captures the latest measured position and commands one out-and-back trajectory:

1. Linearly ramp the position target from that origin to `origin + 2pi rad` (360 degrees) over **10 seconds**.
2. Linearly ramp the target back to the same origin over another **10 seconds**.
3. Send stop, require Reset-mode feedback, and shut TWAI down. There is no repeating loop.

Near the positive protocol limit the outbound target uses `-2pi rad` instead. Both legs use Kp 90.0, Kd 1.0, signed desired velocity derived from the trajectory slope (`+/-0.628 rad/s` for this profile), and zero feed-forward torque. Targets are updated on a nominal 20 ms cycle with feedback required within 250 ms; each endpoint is transmitted and checked. Scheduling and feedback latency may extend elapsed time slightly. The full out-and-back sequence takes approximately **20 seconds**, plus the zero-effort/start/stop stages; it does not continuously rotate.

The nominal **target slope** is approximately `0.628 rad/s` (`36 deg/s`, or 6 rpm), commanding a full turn over 10 seconds. The same signed value is sent as the Motion desired-velocity term so Kd damps velocity error around the intended trajectory instead of opposing all movement. It is not a hard limit on actual motor speed. Zero feed-forward torque does not impose a total-torque cap. Protocol quantization, friction and finite gains can cause lag or stick-slip. Phase-end logs report the target, measured position, and error; finishing the command sequence does not prove the motor reached either endpoint.

The position and velocity feedback terms make this profile load-adaptive: with no opposing load and small tracking error, commanded effort remains low; a changing load increases position or velocity error and the controller raises effort to keep the same trajectory. Before every command, the firmware evaluates the complete controller estimate, including measured velocity direction, and aborts when its absolute value reaches `10.0 N m` during Motion. This prevents separately acceptable position and velocity errors from adding into an outgoing estimate beyond that software boundary. It is still a model-based pre-transmission check, not a hard motor torque cap.

The 1.0 rad/s motion trip threshold accommodates the target; zero effort retains its original 0.5 rad/s threshold. Neither is a certified or hard speed limit, and the 0.10 rad tracking-error check is unchanged. The Kp 7.5/Kd 0.3, Kp 30.0/Kd 1.0, and Kp 90.0/Kd 1.0 profiles have each been observed completing one unloaded full-turn out-and-back. In the Kp 90.0 run, outbound/return endpoint errors were `+0.0001/+0.0012 rad`; among the routine 250 ms INFO telemetry samples, the maximum absolute velocity was `0.583 rad/s`, maximum absolute feedback torque was `1.422 N m`, and maximum reported temperature was `28.6 C`. The user reported normal motion and sound. These individual experiment records do not validate changing loads, stalls, or reliability. The manual documents `4 N m` rated load and `12 N m` peak load; `Kp=500` and `12 N m` are protocol/peak bounds, not recommended bench commands. Motion has strict `<10.0 N m` feedback and combined command-effort checks while zero effort remains `<1.0 N m`, but these are software observations and estimates, not hard current/torque limits. The firmware does not write the motor's `limit_torque`, `limit_cur`, or current-loop parameters.

This is a low-parameter, unloaded communication experiment. It does not validate motor-control performance, mechanical safety, or suitability for an occupied system.

Earlier small-angle and 180-degree bench logs included CAN bus-error aborts as well as completed runs. One 180-degree attempt also stopped at a measured tracking error of -0.1004 rad. The latest completed half-turn had outbound/return endpoint errors of -0.0065/+0.0061 rad, but does not establish link reliability or validate the faster full-turn test. Changing angle or timing does not resolve these faults; the firmware still aborts on them and may stop early. Do not bypass that protection or repeatedly reset to force completion; cut 24 V and inspect the log and physical bus first.

Firmware startup logs `reset reason=<number> (<name>)` to distinguish external, USB, brownout, watchdog, and software resets. Control feedback is still checked at 50 Hz, while routine INFO telemetry is emitted about every 250 ms so the complete run can be captured.

## Controlled flash and start

1. Switch **motor 24 V off**; leave ESP32 USB connected. Close any existing serial monitor. Secure and unload the motor, check wiring/termination, and keep the physical cutoff within reach. Termination measurements require USB power off too.
2. Enter the ROM downloader manually: hold **BOOT**, press and release **RESET**, then release **BOOT**. Confirm the current serial port (USB re-enumeration can change its name).
3. With ESP-IDF loaded and the intended configuration built, flash without an automatic application reset:

   ```bash
   cd build
   python -m esptool --chip esp32s3 --port PORT \
     --before no_reset --after no_reset write_flash @flash_args
   cd ..
   ```

   Keep 24 V off throughout flashing. Do not press RESET yet. If using another build directory, use its `flash_args` and matching binaries instead.
4. Start the matching serial monitor without requesting a reset:

   ```bash
   idf.py -p PORT monitor --no-reset --timestamps
   ```

   Enable file logging with **Ctrl+T**, then **Ctrl+L**; the monitor prints the log path. Open/reconnect the USB port only while motor 24 V is off, because USB/driver behavior can still disturb the board. Keep the monitor open.
5. With the output clear and the cutoff ready, apply the current-limited 24 V supply, then deliberately press **RESET once**. Check the reported `link_soak` and `motion` states against the intended build and watch the feedback. If the monitor disconnects, cut 24 V before reopening it.
6. After final Reset-mode feedback and TWAI shutdown, switch **24 V off**. On abnormal motion or an abort, cut 24 V immediately; do not rely on software stop alone.

Motion-enabled firmware can run this sequence on **every boot**, including resets or USB reconnections. It has no separate serial arm command. Restore the disabled configuration and rebuild/reflash when motion testing is finished.

## Acceptance

The serial log must show a matching device ID, Reset mode after stop, Motor mode throughout active control, reasonable position/velocity/torque/temperature, and final Reset-mode feedback followed by TWAI shutdown. The default build must report Motion disabled. For Motion, also inspect outbound/return endpoint errors and actual movement in both directions; command completion alone is not physical acceptance.

Disconnect 24 V immediately on timeout, invalid feedback mode, speed/feedback-torque/tracking-error/combined-effort guard rejection, unconfirmed abnormal stop, fatal TWAI alert, bus-off, motor fault/warning, temperature limit, unexpected motion, noise, vibration, or current.

The board schematic and component datasheet are maintained in the local [SN65HVD230 board references](../hardware/SN65HVD230-CAN-Board/).
