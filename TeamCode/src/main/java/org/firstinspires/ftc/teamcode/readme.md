## TeamCode Module

This is where your OpModes go. Everything here is compiled into the Robot Controller app and
shows up in the OpMode list on the Driver Station.

### What ships here

`pedroPathing/` is the Pedro Pathing quickstart:

- **`Constants.java`** - every constant for your robot, grouped by which tuner produces it.
  This is the file you edit while tuning.
- **`Tuning.java`** - the `Tuning` TeleOp (group "Pedro Pathing"), a menu of every Pedro
  Pathing tuner. This is stock Pedro Pathing code; leave it alone.
- **`ExampleAuto.java`** - a template autonomous. Copy it, don't build on it directly.
- **`ExampleTeleop.java`** - field-centric mecanum drive through the follower.

See the [repository README](../../../../../../../../README.md) for the full tuning walkthrough.

### Adding your own OpModes

Create new `.java` files in `org.firstinspires.ftc.teamcode` (or a subpackage of it) and
annotate the class with `@TeleOp` or `@Autonomous`. Give each one a `name` and a `group` so it
is easy to find on the Driver Station:

```java
@TeleOp(name = "My TeleOp", group = "Competition")
public class MyTeleOp extends OpMode { ... }
```

Add `@Disabled` to keep a class in the project but off the Driver Station list.

To reuse the tuned follower, call `Constants.createFollower(hardwareMap)` in `init()` rather
than constructing a `Follower` yourself - that way every OpMode shares one set of constants.

### Naming

Avoid duplicate class names across packages; it makes the OpMode list ambiguous. If you need
the same concept in two places, prefix it (`AutoRed`, `AutoBlue`) instead.
