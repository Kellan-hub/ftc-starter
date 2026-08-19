package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Closed-loop velocity control for a two-wheel flywheel shooter, built on Pedro Pathing's
 * {@link PIDFController}.
 *
 * <p>Both wheels run open-loop ({@code RUN_WITHOUT_ENCODER}) off a single PIDF: raw
 * encoder velocity in, clipped power out, nothing else in between. One loop drives both
 * motors with the same power, since the two wheels squeeze the same game element and have
 * to turn together. Speed is measured from the left motor's encoder.
 *
 * <p>Tune in this order: kF until the flywheel settles near the target on feedforward
 * alone, then kP, then kD, and kI last.
 *
 * <p>Usage:
 *
 * <pre>
 *   flywheel = new FlywheelShooter(hardwareMap, "leftFlywheel", "rightFlywheel");
 *   flywheel.setTargetRpm(3200);
 *   while (opModeIsActive()) {
 *       flywheel.update();              // once per loop
 *       if (flywheel.atSpeed()) feeder.fire();
 *       flywheel.addTelemetry(telemetry);
 *       telemetry.update();
 *   }
 * </pre>
 */
@Config
public class FlywheelShooter {

    // ---------------------------------------------------------------------
    // Dashboard-tunable configuration. Public static so FTC Dashboard can edit
    // them at runtime; edits take effect on the next update() call.
    // ---------------------------------------------------------------------

    /** Proportional gain, power per RPM of error. */
    public static double kP = 0.0;
    /** Integral gain, power per (RPM * s). */
    public static double kI = 0.0;
    /** Derivative gain, power per (RPM/s). */
    public static double kD = 0.0;
    /** Feedforward, power per RPM of target. Start at 1 / free speed in flywheel RPM. */
    public static double kF = 0.0;

    /** Commanded flywheel speed, in RPM. */
    public static double TARGET_RPM = 3000;

    /** Encoder counts per revolution of the motor shaft (bare goBILDA 6000 = 28). */
    public static double TICKS_PER_MOTOR_REV = 8192;
    /** Flywheel revolutions per motor revolution. 2.0 means the wheel spins twice as fast. */
    public static double FLYWHEEL_PER_MOTOR = 1.0;

    /** atSpeed() band, in RPM. */
    public static double TOLERANCE_RPM = 75;

    /** Flip if the left wheel spins the wrong way. */
    public static boolean LEFT_REVERSED = false;
    /** Flip if the right wheel spins the wrong way. Mirrored mounting is the usual case. */
    public static boolean RIGHT_REVERSED = true;

    // ---------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final PIDFController pidf = new PIDFController(new PIDFCoefficients(kP, kI, kD, kF));

    private double targetRpm = 0;
    private double rpm = 0;
    private double power = 0;

    /** Only the left motor needs an encoder plugged in; it is the one the loop reads. */
    public FlywheelShooter(HardwareMap hardwareMap, String leftName, String rightName) {
        leftMotor = hardwareMap.get(DcMotorEx.class, leftName);
        rightMotor = hardwareMap.get(DcMotorEx.class, rightName);

        configure(leftMotor);
        configure(rightMotor);

        leftMotor.setDirection(LEFT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(RIGHT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    private void configure(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Our loop drives the motor directly, so bypass the controller velocity PID.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Coast at zero power: braking a flywheel wastes spin-up time and stresses the gearbox.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // ---------------------------------------------------------------------
    // Commands
    // ---------------------------------------------------------------------

    /** Commands a flywheel speed in RPM. Zero or less coasts the shooter down. */
    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
    }

    /** Spins up to the dashboard-set {@link #TARGET_RPM}. */
    public void spinUp() {
        setTargetRpm(TARGET_RPM);
    }

    /** Cuts power and lets the flywheel coast down. */
    public void stop() {
        setTargetRpm(0);
    }

    // ---------------------------------------------------------------------
    // Control loop
    // ---------------------------------------------------------------------

    /** Runs one iteration of the PIDF and writes power to both motors. Call once per loop. */
    public void update() {
        rpm = ticksPerSecToRpm(leftMotor.getVelocity());

        if (targetRpm <= 0) {
            setPower(0);
            return;
        }

        pidf.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
        pidf.setTargetPosition(targetRpm);
        pidf.updatePosition(rpm);
        pidf.updateFeedForwardInput(targetRpm);

        setPower(Range.clip(pidf.run(), -1, 1));
    }

    private void setPower(double p) {
        power = p;
        leftMotor.setPower(p);
        rightMotor.setPower(p);
    }

    private double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec / TICKS_PER_MOTOR_REV * 60.0 * FLYWHEEL_PER_MOTOR;
    }

    // ---------------------------------------------------------------------
    // Queries
    // ---------------------------------------------------------------------

    /** True when the flywheel is within {@link #TOLERANCE_RPM} of the commanded speed. */
    public boolean atSpeed() {
        return targetRpm > 0 && Math.abs(targetRpm - rpm) <= TOLERANCE_RPM;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    /** Flywheel speed, in RPM, straight off the left encoder. */
    public double getRpm() {
        return rpm;
    }

    public double getErrorRpm() {
        return targetRpm - rpm;
    }

    public double getPower() {
        return power;
    }

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    /**
     * Adds the values worth graphing. Wrap your telemetry in
     * {@code new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())}
     * to see them plotted.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("targetRpm", targetRpm);
        telemetry.addData("rpm", rpm);
        telemetry.addData("errorRpm", getErrorRpm());
        telemetry.addData("power", power);
        telemetry.addData("atSpeed", atSpeed());
    }

    /** The same values, for code that builds its own dashboard packets. */
    public void addTelemetry(TelemetryPacket packet) {
        packet.put("targetRpm", targetRpm);
        packet.put("rpm", rpm);
        packet.put("errorRpm", getErrorRpm());
        packet.put("power", power);
        packet.put("atSpeed", atSpeed() ? 1 : 0);
    }
}
