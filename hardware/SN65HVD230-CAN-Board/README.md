[简体中文](README.zh-CN.md)

# SN65HVD230 CAN Transceiver Board References

This directory maintains the board schematic and TI component datasheet used by the CAN examples in this repository:

- [`SN65HVD230-CAN-Board-Schematic.pdf`](SN65HVD230-CAN-Board-Schematic.pdf) - board-level schematic.
- [`SN65HVD230-CAN-Board_Datasheets.pdf`](SN65HVD230-CAN-Board_Datasheets.pdf) - SN65HVD230/231/232 datasheet.

The board is suitable for short developer CAN/TWAI experiments. It is not a production-ready controller or hub reference design.

## Schematic facts

| Connector | Pin | Signal |
|---|---:|---|
| P1 | 1 | CAN_TX to the SN65HVD230 `D` input |
| P1 | 2 | GND |
| P1 | 3 | 3.3 V |
| P1 | 4 | CAN_RX from the SN65HVD230 `R` output |
| P2 | 1 | CANL |
| P2 | 2 | CANH |

Other board configuration:

- The SN65HVD230 is powered at 3.3 V.
- `RS` is connected to ground through `10 kohm`, selecting slope-control operation.
- A fixed `120 ohm` resistor is installed between CANH and CANL with no disable jumper.
- The schematic shows no isolation, external TVS, reverse-polarity protection, or power conversion.

## Limits

- Supply only regulated 3.3 V to P1 pin 3; never apply 5 V, 12 V, or 24 V.
- The non-isolated design requires an appropriate common ground and bus common-mode range.
- The fixed terminator places this board at a bus endpoint; do not arbitrarily parallel several of these boards on one bus.
- The SN65HVD230 device is rated for signaling up to 1 Mbit/s, but the target wiring, topology, termination, and actual `RS=10 kohm` waveform still require hardware validation.
- The CyberGear experiment now uses an explicit 1 Mbit/s, 20-time-quanta, 80% sample-point controller timing and offers an opt-in 60-second zero-effort 50 Hz link soak. Passing it validates only that specific bench run; it does not prove the transceiver waveform margin.
- This board does not provide power distribution, managed termination, miswiring protection, or a beginner-facing keyed connector.

See [`open-module/examples/esp32-cybergear-can`](https://github.com/open-module/examples/tree/main/esp32-cybergear-can) for the related physical test example.

An unstable CANH-to-CANL resistance measured through an unpowered motor is not evidence of a simple fixed terminator. Do not derive or add a termination value from a time-varying reading alone; use documented device termination and, when needed, waveform measurements.

## CyberGear motion experiment

The example keeps motion disabled by default. Its explicit opt-in ramps a position target by 2pi radians (360 degrees) over 10 seconds and back over 10 seconds, then stops once. The approximately `+/-0.628 rad/s` target slope is also sent as the signed desired-velocity term; it is not a hard actual-speed limit. Absolute measured speed at or above 0.5 rad/s during zero effort or 1.0 rad/s during opt-in motion, absolute feedback torque at or above 1.0 N m during zero effort or 10.0 N m during Motion, absolute tracking error at or above 0.10 rad from the current trajectory target, or loss of Motor-mode feedback aborts active control; before each transmission, the complete feed-forward plus position/velocity-error effort estimate must also remain below the phase torque threshold. Stop confirmation requires Reset mode. Zero effort retains a fixed reference. These provisional software checks do not make this board a safety controller.

Clear the full-turn sweep, prevent cable winding, and supervise the approximately 20-second run. Earlier small-angle and half-turn tests included intermittent CAN bus-error aborts, and one half-turn attempt tripped the 0.10 rad tracking-error guard. With the explicit 80% sample point, one unloaded 60-second link soak later completed with TEC/REC/failed/error counters all zero. The Kp 7.5/Kd 0.3, Kp 30.0/Kd 1.0, and Kp 90.0/Kd 1.0 profiles were each observed completing one unloaded 360-degree out-and-back. The Kp 90.0 run reported `+0.0001/+0.0012 rad` endpoint errors; among the routine 250 ms INFO telemetry samples, maximum absolute velocity was `0.583 rad/s`, maximum absolute feedback torque was `1.422 N m`, and maximum reported temperature was `28.6 C`. The user reported normal motion and sound. These individual runs do not validate changing loads or reliability.

Motion now uses Kp 90.0, Kd 1.0, and zero feed-forward torque. Small unloaded tracking errors keep effort low, while load-induced position or velocity error automatically raises controller effort. Before every transmission, the complete controller estimate must remain strictly below 10.0 N m during Motion, so separately acceptable position and velocity errors cannot combine into a larger outgoing estimate. This model-based check and the delayed 10.0 N m feedback trip are not hard torque limits. The example does not write motor-side `limit_torque`, `limit_cur`, or current-loop parameters. Only one unloaded run of this profile has been physically observed; changing-load behavior remains to be validated. Startup reset-cause logging helps diagnose USB, brownout, watchdog, and other resets. Do not repeatedly reset to force a failed run to complete.

Keep motor 24 V off during wiring, flashing, and opening/reconnecting the serial monitor. Follow the example's controlled flash/start procedure and prepare a physical cutoff. Motion-enabled firmware may run again after every reset or USB reconnection. Neither TWAI shutdown nor a successful build proves power isolation or physical motor safety.
