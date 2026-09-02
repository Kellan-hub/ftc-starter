package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * PIDF tuning OpMode for {@link FlywheelShooter}, powered the same way team 7641's
 * {@code testing/ShooterTest.java} powers theirs: updateRPM, set the target, updatePID,
 * every loop, for as long as the OpMode runs. The gamepad is not used.
 *
 * <p>Open FTC Dashboard at http://192.168.43.1:8080/dash, run this OpMode, and edit
 * {@code TARGET_RPM} and the gains in the {@code FlywheelShooter} block while it spins.
 * Graph {@code RPM} against {@code targetRPM} to see the response. Set {@code TARGET_RPM}
 * to 0 to spin down.
 *
 * <p>Tune kF first, then kP, then kD, and kI last.
 */
@Config
@TeleOp(name = "Flywheel PIDF Tuner", group = "tuning")
public class FlywheelTuner extends LinearOpMode {

    FlywheelShooter shooter;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new FlywheelShooter(hardwareMap);

        ElapsedTime time = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            shooter.updateRPM();
            shooter.targetRPM = FlywheelShooter.TARGET_RPM;
            shooter.updatePID();

            telemetry.addData("RPM", shooter.RPM);
            telemetry.addData("targetRPM", shooter.targetRPM);
            telemetry.addData("power", shooter.leftMotor.getPower());
            telemetry.addData("inThreshold", shooter.RPMInThreshold());
            telemetry.addData("time change", time.milliseconds());
            telemetry.update();

            time.reset();
        }

        shooter.setPower(0);
    }
}
