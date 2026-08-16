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

## What's included

- `org.firstinspires.ftc:*:11.1.0` — the FTC SDK, pulled from Maven Central
- `com.pedropathing:ftc:2.1.2` + `com.pedropathing:telemetry:1.0.0` — path following
- The stock Robot Controller app scaffolding (`FtcRobotControllerActivity`,
  `FtcOpModeRegister`, `PermissionValidatorWrapper`, and their resources) —
  unmodified FIRST/Qualcomm source, required for the app to run on the Control Hub

## Getting Started

```bash
git clone <this-repo-url>
```

Open the cloned folder in Android Studio and let Gradle sync. Then:

```bash
./gradlew :TeamCode:assembleDebug   # build the APK
./gradlew :TeamCode:installDebug    # install onto a connected Control Hub
```

Write your OpModes in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
For Pedro Pathing setup (drivetrain constants, hardware config, `Follower`),
see the [Pedro Pathing docs](https://pedropathing.com/).

## License

The FTC SDK scaffolding in this repo is unmodified FIRST/Qualcomm source,
distributed under the license in [LICENSE](LICENSE).
