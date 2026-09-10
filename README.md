# FTC Starter (SDK + Pedro Pathing)

A bare-bones Android Studio project for *FIRST* Tech Challenge: the FTC SDK and
[Pedro Pathing](https://pedropathing.com/) wired up as Gradle dependencies, and
nothing else. No team framework, no sample OpModes, no vendored SDK source —
just a project that builds and deploys, ready for you to write your own code
in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

## Requirements

- Android Studio (current stable release)
- JDK 17 for the Gradle daemon (pinned via `gradle/gradle-daemon-jvm.properties`)
- A REV Control Hub

## Project layout

Two modules, matching the official FTC template:

| Module | Type | Contents |
| --- | --- | --- |
| `FtcRobotController` | `com.android.application` | The Robot Controller app — stock, unmodified FIRST/Qualcomm scaffolding (`FtcRobotControllerActivity`, `PermissionValidatorWrapper`, `FtcOpModeRegister`) and its resources. Builds the APK you install on the Control Hub. |
| `TeamCode` | `com.android.library` | **Your code.** Compiled into the Robot Controller APK. |

Shared dependency versions live in `build.dependencies.gradle`, which both
modules apply, so the SDK version is declared once.

## What's included

- `org.firstinspires.ftc:*:11.1.0` — the FTC SDK, pulled from Maven Central
- `com.pedropathing:ftc:2.1.2` + `com.pedropathing:telemetry:1.0.0` — path following
- `com.bylazar:fullpanels` and `com.acmerobotics.dashboard:dashboard` — the
  telemetry/tuning dashboards the Pedro Pathing tuners report to

## Getting Started

```bash
git clone <this-repo-url>
```

Open the cloned folder in Android Studio and let Gradle sync. The run
configuration to pick is **FtcRobotController**.

```bash
./gradlew :FtcRobotController:assembleDebug   # build the APK
./gradlew :FtcRobotController:installDebug    # install onto a connected Control Hub
```

The APK lands in `FtcRobotController/build/outputs/apk/debug/`.

## Deploying and running

The Robot Controller app runs on the **Control Hub**, not the Driver Hub. The
Driver Hub runs the Driver Station app, which ships preinstalled and is not
built from this repo.

1. Connect to the Control Hub — USB-C from your laptop, or join its Wi-Fi
   network and use `adb connect 192.168.43.1:5555`.
2. `./gradlew :FtcRobotController:installDebug`
3. Power-cycle or restart the robot, then pair the Driver Hub to the Control
   Hub's network.
4. Your OpModes appear in the Driver Hub's Autonomous/TeleOp dropdowns.

## Writing OpModes

Write them in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Annotate
with `@TeleOp` or `@Autonomous` — the SDK discovers them by annotation, so there
is nothing to register by hand.

For Pedro Pathing setup (drivetrain constants, hardware config, `Follower`),
see the [Pedro Pathing docs](https://pedropathing.com/).

## License

The FTC SDK scaffolding in this repo is unmodified FIRST/Qualcomm source,
distributed under the license in [LICENSE](LICENSE).
