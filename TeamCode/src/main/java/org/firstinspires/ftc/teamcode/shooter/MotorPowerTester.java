package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Open-loop motor tester. Each motor gets its own power variable, set straight from
 * FTC Dashboard and written with {@code setPower()} — no PIDF, no closed loop, no
 * motor setup of any kind.
 *
 * <p>Run the OpMode, then edit {@code LEFT_POWER} and {@code RIGHT_POWER} in the
 * {@code MotorPowerTester} block and hit the dashboard's save button; the values apply
 * on the next loop. The motors come from the robot configuration as "leftMotor" and
 * "rightMotor".
 *
 * <p>{@code leftActual} is read back from the motor rather than echoed from the
 * variable, so if commanded and actual disagree the problem is between the dashboard
 * and the code, not in the wiring.
 */
@Config
@TeleOp(name = "Motor Power Tester", group = "tuning")
public class MotorPowerTester extends LinearOpMode {

    /** Power for the left motor, -1 to 1. */
    public static double LEFT_POWER = 0;
    /** Power for the right motor, -1 to 1. */
    public static double RIGHT_POWER = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor left = hardwareMap.get(DcMotor.class, "leftMotor");
        DcMotor right = hardwareMap.get(DcMotor.class, "rightMotor");

        telemetry.addLine("Ready. Set LEFT_POWER / RIGHT_POWER from the dashboard.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            left.setPower(LEFT_POWER);
            right.setPower(RIGHT_POWER);

            telemetry.addData("leftPower", LEFT_POWER);
            telemetry.addData("rightPower", RIGHT_POWER);
            telemetry.addData("leftActual", left.getPower());
            telemetry.addData("rightActual", right.getPower());
            telemetry.addData("leftMode", left.getMode());
            telemetry.addData("rightMode", right.getMode());
            telemetry.addData("leftTicks", left.getCurrentPosition());
            telemetry.addData("rightTicks", right.getCurrentPosition());
            telemetry.update();
        }

        left.setPower(0);
        right.setPower(0);
    }
}
