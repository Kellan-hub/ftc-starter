# Pedro Pathing Quickstart

The FTC SDK (DECODE, 2025-2026) with [Pedro Pathing](https://pedropathing.com) 2.1.2 wired up and
ready to tune, plus FTC Dashboard and Panels for live telemetry.

Everything you touch lives in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/`:

| File | What it is |
| --- | --- |
| `Constants.java` | Every constant for your robot. This is the file tuning writes into. |
| `Tuning.java` | The `Tuning` TeleOp: a menu of every Pedro Pathing tuner. Don't edit it. |
| `ExampleAuto.java` | Template autonomous - path chains driven by a state machine. |
| `ExampleTeleop.java` | Field-centric TeleOp that drives through the follower. |

## Before you tune

1. **Set the hardware names.** In `Constants.java`, `LEFT_FRONT_MOTOR` / `RIGHT_FRONT_MOTOR` /
   `LEFT_REAR_MOTOR` / `RIGHT_REAR_MOTOR` and `PINPOINT_NAME` must match your robot
   configuration on the Driver Hub exactly. This repo ships with `fl`, `fr`, `bl`, `br` and
   `pinpoint`.
2. **Weigh the robot** and set `MASS` (kilograms).
3. **Measure the odometry pods** and set `FORWARD_POD_Y` and `STRAFE_POD_X` (inches, from the
   center of rotation, +X forward and +Y left).

This quickstart assumes a **mecanum drivetrain with a goBILDA Pinpoint**. For a different
localizer (OTOS, two-wheel, three-wheel, drive encoders), swap `pinpointLocalizer(...)` at the
bottom of `Constants.java` for the matching `FollowerBuilder` method and its constants class.

## Connecting the tuning UI

Connect your laptop to the Robot Controller's Wi-Fi, then open either:

- **Panels** - `http://192.168.43.1:8001/` . The `Tuning` OpMode's menu and field view are here.
- **FTC Dashboard** - `http://192.168.43.1:8080/dash` . Live graphs and `@Config` values.

(On a Control Hub the address is `192.168.43.1`; on a phone RC, use the phone's IP.)

## Tuning order

Run the **Tuning** TeleOp (group "Pedro Pathing") and work down its menu. After each tuner,
copy the number it reports into the matching constant in `Constants.java` and re-deploy.

### 1. Localization

Nothing else works until the robot knows where it is.

| Tuner | Sets |
| --- | --- |
| Localization Test | Motor + encoder directions. Push the robot by hand; x, y and heading must all move the right way. |
| Offsets Tuner | `FORWARD_POD_Y`, `STRAFE_POD_X` |
| Forward Tuner | Multiplier check - push forward 48", the reported distance should match. |
| Lateral Tuner | Same, strafing. |
| Turn Tuner | Heading scalar - spin the robot a known number of turns. |

### 2. Automatic

Drives the robot itself, so give it clear floor - roughly 8 feet each way.

| Tuner | Sets |
| --- | --- |
| Forward Velocity Tuner | `X_VELOCITY` |
| Lateral Velocity Tuner | `Y_VELOCITY` |
| Forward Zero Power Acceleration Tuner | `FORWARD_ZERO_POWER_ACCELERATION` |
| Lateral Zero Power Acceleration Tuner | `LATERAL_ZERO_POWER_ACCELERATION` |

### 3. Manual

Each of these is an interactive PIDF tune. Adjust the coefficients live in Panels, watch the
error graph, then write the values you settle on into `Constants.java`.

| Tuner | Sets | Check against |
| --- | --- | --- |
| Translational Tuner | `TRANSLATIONAL_PIDF` | Tests > Line |
| Heading Tuner | `HEADING_PIDF` | Tests > Line |
| Drive Tuner | `DRIVE_PIDF` | Tests > Line |
| Centripetal Tuner | `CENTRIPETAL_SCALING` | Tests > Circle |

### 4. Tests

**Line**, **Triangle** and **Circle** should all track cleanly with no oscillation, no
overshoot, and no drifting wide on the curves. If one of them misbehaves, go back to the tuner
in the table above that it checks.

Then run **Example Auto** to confirm a real path chain works end to end.

## Building

```
./gradlew :TeamCode:assembleDebug
```

Or open the project in Android Studio (Ladybug 2024.2 or later) and hit Run.

## Reference

- [Pedro Pathing docs](https://pedropathing.com) - tuning guide, path geometry, API reference
- [Pedro Pathing Discord](https://discord.gg/pedropathing)
- [FIRST Tech Challenge documentation](https://ftc-docs.firstinspires.org/index.html)
- [FTC Dashboard docs](https://acmerobotics.github.io/ftc-dashboard/)
- `doc/` - the stock FTC SDK release notes and API javadoc that ship with this repo
