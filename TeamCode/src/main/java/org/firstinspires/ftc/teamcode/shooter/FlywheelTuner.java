package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * Tuning OpMode for {@link FlywheelShooter}.
 *
 * <p>Open FTC Dashboard at http://192.168.43.1:8080/dash, run this OpMode, then edit
 * the {@code FlywheelShooter} block in the Configuration panel while it runs. Graph
 * {@code measuredRpm} against {@code targetRpm} to see the response.
 *
 * <p>Controls: A spins up to {@code TARGET_RPM}, B coasts down, X toggles a
 * step between {@code TARGET_RPM} and {@code STEP_RPM} so overshoot and recovery
 * are easy to see.
 *
 * <p>Suggested order:
 * <ol>
 *   <li>Zero kP, kI and kD. Raise kV until the measured speed settles near the target
 *       on feedforward alone; kV is roughly 1 / free speed in flywheel RPM.</li>
 *   <li>Raise kS until a low target (say 500 RPM) actually starts spinning.</li>
 *   <li>Raise kP until the remaining error closes quickly without ringing.</li>
 *   <li>Add a little kD only if kP overshoots.</li>
 *   <li>Add kI only if a constant offset survives everything above.</li>
 * </ol>
 */
@Config
@TeleOp(name = "Flywheel PIDF Tuner", group = "tuning")
public class FlywheelTuner extends LinearOpMode {

    /** Hardware map names. Changing these takes effect the next time the OpMode starts. */
    public static String LEAD_MOTOR = "shooterLeft";
    /** Set to an empty string for a single-motor shooter. */
    public static String FOLLOWER_MOTOR = "shooterRight";

    /** The other speed the X button steps to, for watching disturbance recovery. */
    public static double STEP_RPM = 1500;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        String followerName = FOLLOWER_MOTOR == null || FOLLOWER_MOTOR.trim().isEmpty()
                ? null : FOLLOWER_MOTOR.trim();
        FlywheelShooter flywheel = new FlywheelShooter(hardwareMap, LEAD_MOTOR.trim(), followerName);

        // Bulk reads keep the control loop fast: one transaction per hub per loop
        // instead of one per sensor call.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addLine("Ready. A = spin up, B = stop, X = step speed.");
        telemetry.update();
        waitForStart();

        boolean lastX = false;
        boolean stepped = false;

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            if (gamepad1.a) {
                stepped = false;
                flywheel.setTargetRpm(FlywheelShooter.TARGET_RPM);
            }
            if (gamepad1.b) {
                stepped = false;
                flywheel.stop();
            }
            if (gamepad1.x && !lastX && flywheel.getTargetRpm() > 0) {
                stepped = !stepped;
                flywheel.setTargetRpm(stepped ? STEP_RPM : FlywheelShooter.TARGET_RPM);
            }
            lastX = gamepad1.x;

            flywheel.update();

            flywheel.addTelemetry(telemetry);
            telemetry.update();
        }

        flywheel.stop();
        flywheel.update();
    }
}
