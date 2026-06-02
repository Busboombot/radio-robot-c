# Hardware Bench Testing

The robot is **connected and mounted on a stand**. Its wheels are off the ground,
so it **cannot drive away** — it is safe to power the motors and spin the wheels
freely during verification. Use the real hardware to confirm changes, not just
unit tests.

## Standing verification gate

Every firmware sprint that touches the HAL, motor control, sensing, or the
command protocol must, as part of its acceptance, **deploy to the robot and
exercise it on the stand**:

1. **Sensors are alive.** Talk to every sensor and confirm it responds with
   plausible, changing values: encoders (motor controller), OTOS
   (position/velocity), line sensor (4 channels), color sensor (RGBC), and the
   digital/analog ports.
2. **Wheels drive and encoders run.** Command the wheels (both directions) and
   confirm the **encoders increment** in the expected direction and roughly in
   proportion to commanded speed. Because the robot is on the stand, drive freely.
3. **Round-trip over the real link.** Confirm commands and replies work over the
   actual transport (serial at the bench; radio relay when testing the relay
   path).

A sprint is not "done" on tests alone — it must be seen working on the stand.

## How to deploy and drive

Build + flash with the project's deploy tool (the robot is a flashable probe):

```bash
mbdeploy probe          # discover/refresh the connected device registry
mbdeploy deploy --build # build firmware and flash the robot
```

Then open the serial port (or drive through the radio relay) and issue commands.
See the current command set in [source/app/CommandProcessor.cpp](../source/app/CommandProcessor.cpp)
and, once Sprint 009 lands, the [protocol v2 spec](kinematics-model.md) command
surface (PING/ECHO/ID, SET/GET, TLM/STREAM, motion verbs).

### Quick smoke sequence (current protocol)

| Step | Command | Expect |
|---|---|---|
| Identify | `HELLO` | `DEVICE:Nezha2:<name>:microbit:<serial>` |
| Encoders zero/read | `EZ` then `ENC` | `ACK:EZ`, then `ENC+0+0` |
| Drive (on stand) | `S+150+150` | wheels spin; streamed `ENC…` values climb |
| Stop | `X` | `ACK:X`; encoders hold |
| Odometry | `SO` | `SO±x±y±h` updates after driving |
| Line / color | `LS` / `CS` | 4 plausible channel values each |
| OTOS | `O` then `OP` / `OR` | init ack, then position / velocity reads |

(After Sprint 009 these become the v2 forms — `PING`, `ECHO`, `SET`/`GET`,
combined `TLM` frames, `GRIP`, etc. — and the bench gate is re-run against them.)

## Safety notes

- On the stand the wheels spin free; still avoid loose clothing/fingers near the
  drivetrain and the gripper.
- A locked/protected nRF recovers via `mbdeploy`'s automatic mass-erase on a
  failed flash (no manual step needed).
