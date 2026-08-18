package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Closed-loop velocity control for a flywheel shooter.
 *
 * <p>Runs its own PIDF loop on the Robot Controller instead of the motor
 * controller's built-in velocity mode, so every gain is live-tunable from FTC
 * Dashboard (Configuration panel, {@code FlywheelShooter}) and the response can
 * be watched on a graph without redeploying.
 *
 * <p>Control law, in motor power (-1..1), evaluated once per loop:
 *
 * <pre>
 *   power = kS + kV * setpoint + kA * setpointAccel   (feedforward)
 *         + kP * error                                (proportional)
 *         + kI * integral(error)                      (integral, clamped)
 *         - kD * dMeasuredRpm/dt                      (derivative on measurement)
 * </pre>
 *
 * <p>The feedforward does nearly all the work on a flywheel; the PID terms only
 * have to reject the disturbance of a game element passing through. Tune in that
 * order: kV, then kS, then kP, then kD, and only add kI if a steady-state offset
 * remains.
 *
 * <p>Usage:
 *
 * <pre>
 *   flywheel = new FlywheelShooter(hardwareMap, "shooterLeft", "shooterRight");
 *   flywheel.setTargetRpm(3200);
 *   while (opModeIsActive()) {
 *       flywheel.update();              // once per loop, as fast as possible
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

    /** Feedforward: motor power per RPM of setpoint. Start at 1 / freeSpeedRpm. */
    public static double kV = 1.0 / 6000.0;
    /** Feedforward: constant power that overcomes friction and stiction. */
    public static double kS = 0.03;
    /** Feedforward: power per (RPM/s) of commanded acceleration. Usually 0. */
    public static double kA = 0.0;

    /** Proportional gain, power per RPM of error. */
    public static double kP = 0.0004;
    /** Integral gain, power per (RPM * s). Leave at 0 unless the speed droops. */
    public static double kI = 0.0;
    /** Derivative gain, power per (RPM/s). Damps overshoot; keep it small. */
    public static double kD = 0.00002;

    /** Default commanded speed, in RPM at the flywheel. */
    public static double TARGET_RPM = 3000;

    /** Encoder counts per revolution of the motor shaft (bare goBILDA 6000 = 28). */
    public static double TICKS_PER_MOTOR_REV = 28.0;
    /** Flywheel revolutions per motor revolution. 2.0 means the wheel spins twice as fast. */
    public static double FLYWHEEL_PER_MOTOR = 1.0;

    /** The integral only accumulates inside this error band, in RPM. */
    public static double I_ZONE_RPM = 300;
    /** Hard cap on how much power the integral term may contribute. */
    public static double MAX_I_POWER = 0.15;

    /** Setpoint slew limit, RPM/s. Keeps a step command from saturating the loop. */
    public static double MAX_RPM_PER_SEC = 12000;

    /** Measured-velocity low-pass, 0..0.95. 0 = raw encoder, higher = smoother but laggier. */
    public static double VELOCITY_FILTER = 0.75;
    /** Derivative low-pass, 0..0.95. Encoder velocity is noisy, so this one matters. */
    public static double DERIVATIVE_FILTER = 0.85;

    /** atSpeed() band, in RPM. */
    public static double TOLERANCE_RPM = 75;
    /** Error must stay inside TOLERANCE_RPM this long before atSpeed() returns true. */
    public static double AT_SPEED_HOLD_SEC = 0.15;

    /** Output floor. Keep it at 0 so the loop coasts down instead of braking backwards. */
    public static double MIN_POWER = 0.0;
    /** Output ceiling. */
    public static double MAX_POWER = 1.0;

    /** Flip if the lead motor spins the wrong way. */
    public static boolean LEAD_REVERSED = false;
    /** Flip if the second motor is mounted mirrored, which is the usual dual-motor layout. */
    public static boolean FOLLOWER_REVERSED = true;

    // ---------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------

    private final DcMotorEx lead;
    private final DcMotorEx follower; // null on a single-motor shooter

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime inToleranceTimer = new ElapsedTime();

    private double targetRpm = 0;
    private double setpointRpm = 0; // slew-limited target actually fed to the loop
    private double lastSetpointRpm = 0;
    private double measuredRpm = 0; // filtered
    private double rawRpm = 0;
    private double lastMeasuredRpm = 0;
    private double derivativeRpm = 0; // filtered d(measured)/dt
    private double integral = 0;
    private double outputPower = 0;
    private double dt = 0;
    private boolean firstUpdate = true;
    private boolean lastLeadReversed = LEAD_REVERSED;
    private boolean lastFollowerReversed = FOLLOWER_REVERSED;

    /** Single-motor shooter. */
    public FlywheelShooter(HardwareMap hardwareMap, String leadName) {
        this(hardwareMap, leadName, null);
    }

    /**
     * Two-motor shooter. Pass {@code null} for {@code followerName} to use one motor.
     * Both motors get the same power; there is no second loop, which is what you want
     * when they drive a shared shaft or belt.
     */
    public FlywheelShooter(HardwareMap hardwareMap, String leadName, String followerName) {
        lead = hardwareMap.get(DcMotorEx.class, leadName);
        follower = followerName == null ? null : hardwareMap.get(DcMotorEx.class, followerName);

        configure(lead);
        if (follower != null) {
            configure(follower);
        }
        applyDirections();
        loopTimer.reset();
        inToleranceTimer.reset();
    }

    private void configure(DcMotorEx motor) {
        // Our loop drives the motor directly, so bypass the controller velocity PID.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Coast at zero power: braking a flywheel wastes spin-up time and stresses the gearbox.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // The SDK caps commanded output at 85% of free speed by default. A flywheel
        // needs the whole range for the kV feedforward to stay linear near the top.
        MotorConfigurationType type = motor.getMotorType().clone();
        type.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(type);
    }

    private void applyDirections() {
        lead.setDirection(LEAD_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (follower != null) {
            boolean reversed = LEAD_REVERSED ^ FOLLOWER_REVERSED;
            follower.setDirection(reversed
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }
        lastLeadReversed = LEAD_REVERSED;
        lastFollowerReversed = FOLLOWER_REVERSED;
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

    /**
     * Runs one iteration of the PIDF loop and writes power to the motors. Call this
     * exactly once per OpMode loop, as often as possible: the gains assume a fast,
     * reasonably consistent update rate.
     */
    public void update() {
        dt = loopTimer.seconds();
        loopTimer.reset();
        // Guards a divide-by-zero on the first tick, and stops a long stall (a vision
        // frame, a hardware timeout) from blowing up the D and I terms.
        dt = Range.clip(dt, 1e-4, 0.1);

        if (LEAD_REVERSED != lastLeadReversed || FOLLOWER_REVERSED != lastFollowerReversed) {
            applyDirections();
        }

        // --- measure ------------------------------------------------------
        rawRpm = ticksPerSecToRpm(lead.getVelocity());
        double velAlpha = Range.clip(VELOCITY_FILTER, 0, 0.95);
        if (firstUpdate) {
            measuredRpm = rawRpm;
            lastMeasuredRpm = rawRpm;
            firstUpdate = false;
        } else {
            measuredRpm = velAlpha * measuredRpm + (1 - velAlpha) * rawRpm;
        }

        // --- setpoint slew --------------------------------------------------
        lastSetpointRpm = setpointRpm;
        if (targetRpm <= 0) {
            setpointRpm = 0;
        } else {
            double maxStep = Math.max(0, MAX_RPM_PER_SEC) * dt;
            setpointRpm += Range.clip(targetRpm - setpointRpm, -maxStep, maxStep);
        }
        double setpointAccel = (setpointRpm - lastSetpointRpm) / dt;

        // --- idle -----------------------------------------------------------
        if (targetRpm <= 0) {
            integral = 0;
            derivativeRpm = 0;
            lastMeasuredRpm = measuredRpm;
            outputPower = 0;
            write(0);
            inToleranceTimer.reset();
            return;
        }

        double error = setpointRpm - measuredRpm;

        // --- feedforward ------------------------------------------------------
        double feedforward = kS + kV * setpointRpm + kA * setpointAccel;

        // --- proportional -----------------------------------------------------
        double pTerm = kP * error;

        // --- integral, with an I-zone and a clamped contribution -----------------
        if (kI == 0 || Math.abs(error) > I_ZONE_RPM) {
            // Outside the band the feedforward and P term own the response, so
            // integrating there only builds windup during spin-up.
            integral = 0;
        } else {
            integral += error * dt;
        }
        double iTerm = kI * integral;
        double clampedITerm = Range.clip(iTerm, -Math.abs(MAX_I_POWER), Math.abs(MAX_I_POWER));
        if (kI != 0 && clampedITerm != iTerm) {
            integral = clampedITerm / kI; // back-calculate so the accumulator cannot run away
            iTerm = clampedITerm;
        }

        // --- derivative on measurement, so a setpoint change gives no kick --------
        double rawDerivative = (measuredRpm - lastMeasuredRpm) / dt;
        lastMeasuredRpm = measuredRpm;
        double dAlpha = Range.clip(DERIVATIVE_FILTER, 0, 0.95);
        derivativeRpm = dAlpha * derivativeRpm + (1 - dAlpha) * rawDerivative;
        double dTerm = -kD * derivativeRpm;

        // --- output -------------------------------------------------------------
        outputPower = Range.clip(feedforward + pTerm + iTerm + dTerm,
                Range.clip(MIN_POWER, -1, 1), Range.clip(MAX_POWER, -1, 1));
        write(outputPower);

        if (Math.abs(targetRpm - measuredRpm) > TOLERANCE_RPM) {
            inToleranceTimer.reset();
        }
    }

    private void write(double power) {
        lead.setPower(power);
        if (follower != null) {
            follower.setPower(power);
        }
    }

    private double ticksPerSecToRpm(double ticksPerSec) {
        double motorRpm = ticksPerSec / TICKS_PER_MOTOR_REV * 60.0;
        return motorRpm * FLYWHEEL_PER_MOTOR;
    }

    // ---------------------------------------------------------------------
    // Queries
    // ---------------------------------------------------------------------

    /** True once the flywheel has held the commanded speed for {@link #AT_SPEED_HOLD_SEC}. */
    public boolean atSpeed() {
        return targetRpm > 0
                && Math.abs(targetRpm - measuredRpm) <= TOLERANCE_RPM
                && inToleranceTimer.seconds() >= AT_SPEED_HOLD_SEC;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    /** The slew-limited setpoint the loop is currently chasing. */
    public double getSetpointRpm() {
        return setpointRpm;
    }

    /** Filtered flywheel speed, in RPM. */
    public double getRpm() {
        return measuredRpm;
    }

    /** Unfiltered flywheel speed, in RPM, straight off the encoder. */
    public double getRawRpm() {
        return rawRpm;
    }

    public double getErrorRpm() {
        return targetRpm - measuredRpm;
    }

    public double getPower() {
        return outputPower;
    }

    /** Seconds spent in the last control iteration. Watch it for loop-time regressions. */
    public double getLoopSeconds() {
        return dt;
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
        telemetry.addData("setpointRpm", setpointRpm);
        telemetry.addData("measuredRpm", measuredRpm);
        telemetry.addData("rawRpm", rawRpm);
        telemetry.addData("errorRpm", getErrorRpm());
        telemetry.addData("power", outputPower);
        telemetry.addData("atSpeed", atSpeed());
        telemetry.addData("loopMs", dt * 1000);
    }

    /** The same values, for code that builds its own dashboard packets. */
    public void addTelemetry(TelemetryPacket packet) {
        packet.put("targetRpm", targetRpm);
        packet.put("setpointRpm", setpointRpm);
        packet.put("measuredRpm", measuredRpm);
        packet.put("rawRpm", rawRpm);
        packet.put("errorRpm", getErrorRpm());
        packet.put("power", outputPower);
        packet.put("atSpeed", atSpeed() ? 1 : 0);
        packet.put("loopMs", dt * 1000);
    }
}
