package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Every Pedro Pathing constant for this robot, in one place.
 *
 * <p>Build a {@link Follower} with {@link #createFollower(HardwareMap)} rather than with
 * {@code new Follower(...)}, so every OpMode shares these values.
 *
 * <p>The numbers below are <em>starting points</em>, not tuned values. Each one is tagged
 * with where its real value comes from:
 * <ul>
 *   <li><b>CONFIG</b> - must match your robot configuration file on the Driver Hub.</li>
 *   <li><b>MEASURE</b> - get it with a ruler or a scale.</li>
 *   <li><b>TUNE</b> - comes out of the named OpMode inside {@link Tuning}.</li>
 * </ul>
 *
 * <p>Tuning order: run the "Tuning" TeleOp and work down its menu.
 * <ol>
 *   <li><b>Localization</b>: Localization Test, Offsets Tuner, then Forward / Lateral /
 *       Turn Tuner, until the reported pose matches reality when you push the robot by hand.</li>
 *   <li><b>Automatic</b>: Forward and Lateral Velocity Tuner, then the two Zero Power
 *       Acceleration Tuners.</li>
 *   <li><b>Manual</b>: Translational, Heading, Drive, then Centripetal, each checked against
 *       the matching path in the <b>Tests</b> folder.</li>
 *   <li><b>Tests</b>: Line, Triangle and Circle should all track cleanly when you are done.</li>
 * </ol>
 *
 * @see Tuning
 */
public class Constants {

    // =================================================================================
    // 1. Robot physical properties
    // =================================================================================

    /** MEASURE: robot mass in kilograms. Used for centripetal force correction. */
    public static final double MASS = 10.65;

    // =================================================================================
    // 2. Drivetrain - mecanum
    // =================================================================================

    /** CONFIG: motor names, exactly as they appear in the robot configuration. */
    public static final String LEFT_FRONT_MOTOR = "fl";
    public static final String RIGHT_FRONT_MOTOR = "fr";
    public static final String LEFT_REAR_MOTOR = "bl";
    public static final String RIGHT_REAR_MOTOR = "br";

    /**
     * TUNE (Localization Test): flip the direction of whichever side drives backwards.
     * Push the left stick forward; the robot must move forward, not spin or strafe.
     */
    public static final DcMotorSimple.Direction LEFT_FRONT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction LEFT_REAR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_REAR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    /** TUNE (Automatic &gt; Forward Velocity Tuner): top forward speed, in inches/second. */
    public static final double X_VELOCITY = 81.34056;

    /** TUNE (Automatic &gt; Lateral Velocity Tuner): top strafing speed, in inches/second. */
    public static final double Y_VELOCITY = 65.43028;

    // =================================================================================
    // 3. Localizer - goBILDA Pinpoint
    // =================================================================================

    /** CONFIG: the I2C device name of the Pinpoint in the robot configuration. */
    public static final String PINPOINT_NAME = "pinpoint";

    /**
     * MEASURE, then TUNE (Localization &gt; Offsets Tuner): pod positions relative to the
     * center of rotation, in inches. +X is forward, +Y is left.
     */
    public static final double FORWARD_POD_Y = 1;
    public static final double STRAFE_POD_X = -2.5;

    /**
     * TUNE (Localization Test): reverse whichever axis counts backwards when you push the
     * robot by hand along it.
     */
    public static final GoBildaPinpointDriver.EncoderDirection FORWARD_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection STRAFE_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // =================================================================================
    // 4. Follower - PIDFs and feedforward
    // =================================================================================

    /**
     * TUNE (Automatic &gt; Forward / Lateral Zero Power Acceleration Tuner): how hard the
     * robot coasts to a stop, in inches per second squared. Both are negative.
     */
    public static final double FORWARD_ZERO_POWER_ACCELERATION = -34.62719;
    public static final double LATERAL_ZERO_POWER_ACCELERATION = -78.15554;

    /**
     * TUNE (Manual &gt; Translational Tuner, checked against Tests &gt; Line): corrects error
     * perpendicular to the path.
     */
    public static final PIDFCoefficients TRANSLATIONAL_PIDF =
            new PIDFCoefficients(0.1, 0, 0, 0.015);

    /** TUNE (Manual &gt; Heading Tuner): corrects heading error. */
    public static final PIDFCoefficients HEADING_PIDF =
            new PIDFCoefficients(1, 0, 0, 0.01);

    /**
     * TUNE (Manual &gt; Drive Tuner): controls speed along the path. The fourth value is the
     * low-pass filter strength; leave it near 0.6 unless the drive output is noisy.
     */
    public static final FilteredPIDFCoefficients DRIVE_PIDF =
            new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01);

    /**
     * TUNE (Manual &gt; Centripetal Tuner, checked against Tests &gt; Circle): pushes the robot
     * into curves so it does not drift wide. Raise it if the robot swings outside the arc.
     */
    public static final double CENTRIPETAL_SCALING = 0.0005;

    // =================================================================================
    // 5. Path constraints - when the follower calls a path finished
    // =================================================================================

    /** t-value past which the path counts as parametrically complete. */
    public static final double T_VALUE_CONSTRAINT = 0.995;
    /** Velocity below which the robot counts as stopped, in inches/second. */
    public static final double VELOCITY_CONSTRAINT = 0.1;
    /** Translational error tolerance at the end of a path, in inches. */
    public static final double TRANSLATIONAL_CONSTRAINT = 0.1;
    /** Heading error tolerance at the end of a path, in radians. */
    public static final double HEADING_CONSTRAINT = 0.007;
    /** Extra time to correct after the path is parametrically done, in milliseconds. */
    public static final double TIMEOUT_CONSTRAINT = 100;
    /** Deceleration multiplier. Higher brakes harder, at the risk of overshoot. */
    public static final double BRAKING_STRENGTH = 1;
    /** Multiplier for how early braking begins. */
    public static final double BRAKING_START = 1;
    /** Search resolution for the closest point on a Bezier curve. */
    public static final int BEZIER_CURVE_SEARCH_LIMIT = 10;

    // =================================================================================
    // Wiring - you should not need to edit below this line
    // =================================================================================

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(MASS)
            .forwardZeroPowerAcceleration(FORWARD_ZERO_POWER_ACCELERATION)
            .lateralZeroPowerAcceleration(LATERAL_ZERO_POWER_ACCELERATION)
            .translationalPIDFCoefficients(TRANSLATIONAL_PIDF)
            .headingPIDFCoefficients(HEADING_PIDF)
            .drivePIDFCoefficients(DRIVE_PIDF)
            .centripetalScaling(CENTRIPETAL_SCALING);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(LEFT_FRONT_MOTOR)
            .rightFrontMotorName(RIGHT_FRONT_MOTOR)
            .leftRearMotorName(LEFT_REAR_MOTOR)
            .rightRearMotorName(RIGHT_REAR_MOTOR)
            .leftFrontMotorDirection(LEFT_FRONT_DIRECTION)
            .leftRearMotorDirection(LEFT_REAR_DIRECTION)
            .rightFrontMotorDirection(RIGHT_FRONT_DIRECTION)
            .rightRearMotorDirection(RIGHT_REAR_DIRECTION)
            .xVelocity(X_VELOCITY)
            .yVelocity(Y_VELOCITY);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName(PINPOINT_NAME)
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardPodY(FORWARD_POD_Y)
            .strafePodX(STRAFE_POD_X)
            .forwardEncoderDirection(FORWARD_ENCODER_DIRECTION)
            .strafeEncoderDirection(STRAFE_ENCODER_DIRECTION);

    // TUNE (optional): uncomment and set this from Localization > Turn Tuner if the reported
    // heading drifts against reality over many rotations. Leaving it unset uses the Pinpoint
    // factory calibration, which is usually correct.
    // static { localizerConstants.yawScalar(1.0); }

    public static PathConstraints pathConstraints = new PathConstraints(
            T_VALUE_CONSTRAINT,
            VELOCITY_CONSTRAINT,
            TRANSLATIONAL_CONSTRAINT,
            HEADING_CONSTRAINT,
            TIMEOUT_CONSTRAINT,
            BRAKING_STRENGTH,
            BEZIER_CURVE_SEARCH_LIMIT,
            BRAKING_START);

    /**
     * Builds a {@link Follower} wired to the constants above. Call this once per OpMode,
     * in {@code init}.
     *
     * @param hardwareMap the hardware map of the OpMode
     * @return a Follower for this robot
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }

    private Constants() {
    }
}
