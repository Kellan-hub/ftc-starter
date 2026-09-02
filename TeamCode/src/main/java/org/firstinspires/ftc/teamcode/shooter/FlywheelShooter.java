package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Raw PIDF velocity control for a two-motor flywheel, modeled on team 7641's
 * {@code mechanisms/Shooter.java}.
 *
 * <p>RPM comes from differencing the left motor's encoder count over the elapsed time
 * since the last {@link #updateRPM()}. Nothing smooths it: no EMA / low-pass, no jitter
 * rejection, no bang-bang assist, no battery voltage compensation. The PIDF output goes
 * straight to {@code setPower()}.
 *
 * <p>Call {@link #updateRPM()} then {@link #updatePID()} once per loop, in that order.
 */
@Config
public class FlywheelShooter {

    public static double kP = 0.000;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.000;

    /** Commanded flywheel speed, in RPM. */


    /** RPMInThreshold() band, in RPM. */
    public static double thresholdTol = 65;

    /** Encoder counts per revolution of the motor shaft. */
    public static double motorTicksPerRevolution = 103.8;
    /** Flywheel revolutions per motor revolution. 2.5 means a 1:2.5 gear-up. */
    public static double GEAR_RATIO = 2.5;

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public double lastPosition;
    public double RPM;
    public double targetRPM = 0;

    public ElapsedTime RPMTimer;
    public PIDFController PIDF;

    /** Only the left motor needs an encoder plugged in; it is the one the loop reads. */
    public FlywheelShooter(HardwareMap HWMap) {
        leftMotor = HWMap.get(DcMotor.class, "leftMotor");
        rightMotor = HWMap.get(DcMotor.class, "rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RPMTimer = new ElapsedTime();
        RPMTimer.reset();
        lastPosition = leftMotor.getCurrentPosition();

        PIDF = new PIDFController(new PIDFCoefficients(kP, kI, kD, kF));
    }

    /** Measures flywheel RPM off the left encoder and picks up any gain edits. */
    public void updateRPM() {
        double currentTime = RPMTimer.milliseconds();

        RPM = ((leftMotor.getCurrentPosition() - lastPosition) / motorTicksPerRevolution)
                * GEAR_RATIO * (60000 / currentTime);

        RPMTimer.reset();
        lastPosition = leftMotor.getCurrentPosition();

        PIDF.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
    }

    /** Runs one PIDF iteration and writes the output straight to both motors. */
    public void updatePID() {
        PIDF.setTargetPosition(targetRPM);
        PIDF.updatePosition(RPM);
        PIDF.updateFeedForwardInput(targetRPM);

        setPower(PIDF.run());
    }

    public void setPower(double p) {
        leftMotor.setPower(p);
        rightMotor.setPower(p);
    }

    public boolean RPMInThreshold() {
        return Math.abs(targetRPM - RPM) < thresholdTol;
    }
}
