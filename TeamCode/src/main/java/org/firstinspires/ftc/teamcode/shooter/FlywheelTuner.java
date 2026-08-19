package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * Tuning OpMode for {@link FlywheelShooter}. Everything is driven from FTC Dashboard;
 * the gamepad is not used.
 *
 * <p>Open FTC Dashboard at http://192.168.43.1:8080/dash and run this OpMode. Flip
 * {@code SHOOTER_ON} in the {@code FlywheelTuner} block to spin up and down, and edit
 * {@code TARGET_RPM} and the gains in the {@code FlywheelShooter} block while it runs.
 * Graph {@code rpm} against {@code targetRpm} to see the response.
 *
 * <p>Suggested order:
 * <ol>
 *   <li>Zero kP, kI and kD. Raise kF until the measured speed settles near the target
 *       on feedforward alone; kF is roughly 1 / free speed in flywheel RPM.</li>
 *   <li>Raise kP until the remaining error closes quickly without ringing.</li>
 *   <li>Add a little kD only if kP overshoots.</li>
 *   <li>Add kI only if a constant offset survives everything above.</li>
 * </ol>
 */
@Config
@TeleOp(name = "Flywheel PIDF Tuner", group = "tuning")
public class FlywheelTuner extends LinearOpMode {

    /** Spins the flywheel up to {@code FlywheelShooter.TARGET_RPM}; false coasts it down. */
    public static boolean SHOOTER_ON = false;

    /** Hardware map names. Changing these takes effect the next time the OpMode starts. */
    public static String LEFT_MOTOR = "leftFlywheel";
    public static String RIGHT_MOTOR = "rightFlywheel";

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FlywheelShooter flywheel =
                new FlywheelShooter(hardwareMap, LEFT_MOTOR.trim(), RIGHT_MOTOR.trim());

        // Bulk reads keep the control loop fast: one transaction per hub per loop
        // instead of one per sensor call.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        SHOOTER_ON = false; // never start a run with the flywheel already commanded on
        telemetry.addLine("Ready. Tune from the dashboard; flip SHOOTER_ON to spin up.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            flywheel.setTargetRpm(SHOOTER_ON ? FlywheelShooter.TARGET_RPM : 0);
            flywheel.update();

            flywheel.addTelemetry(telemetry);
            telemetry.update();
        }

        flywheel.stop();
        flywheel.update();
    }
}
