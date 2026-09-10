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
 * Every Pedro Pathing constant for this robot, in one place: a goBILDA Pinpoint for
 * localization and a mecanum drive on motors {@code fl}, {@code fr}, {@code bl}, {@code br}.
 *
 * <p>Build a {@link Follower} with {@link #createFollower(HardwareMap)} rather than with
 * {@code new Follower(...)}, so every OpMode shares these values.
 *
 * <p>The numbers below are <em>starting points</em>, not tuned values. Anything marked
 * MEASURE is specific to this robot and is wrong until you set it; anything marked TUNE
 * comes out of an OpMode in {@link Tuning}. Work top to bottom:
 * <ol>
 *   <li>Measure the pod offsets and {@link #MASS} with a ruler and a scale.</li>
 *   <li>Run <b>Localization &gt; Yaw Scalar</b>, then <b>Localization Test</b> until the
 *       reported pose matches reality when you push the robot by hand.</li>
 *   <li>Run the four <b>Drive</b> tuners for velocity and zero-power acceleration.</li>
 *   <li>Tune the PIDFs against <b>Straight Back And Forth</b> and <b>Curved Back And Forth</b>.</li>
 * </ol>
 *
 * @see Tuning
 */
public class Constants {

    // ---------------------------------------------------------------------------------
    // Robot physical properties
    // ---------------------------------------------------------------------------------

    /** MEASURE: robot mass in kilograms, used for centripetal force correction. */
    public static final double MASS = 10.65;

    // ---------------------------------------------------------------------------------
    // Follower: path following behaviour and PIDF gains
    // ---------------------------------------------------------------------------------

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(MASS);

    // ---------------------------------------------------------------------------------
    // Drivetrain: mecanum on fl / fr / bl / br
    // ---------------------------------------------------------------------------------

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("fl")
            .rightFrontMotorName("fr")
            .leftRearMotorName("bl")
            .rightRearMotorName("br")

            // Localization Test, flip whichever side is wrong here.
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);


    // ---------------------------------------------------------------------------------
    // Localizer: goBILDA Pinpoint
    // ---------------------------------------------------------------------------------

    public static PinpointConstants localizerConstants = new PinpointConstants()
            // The name of the I2C port the Pinpoint is plugged into, in the robot config.
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.INCH)

            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            // Flip either of these if Localization Test reports motion backwards along
            // that axis when you push the robot by hand.
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // TUNE (optional): set this from Localization > Yaw Scalar if the reported heading
    // drifts against reality over many rotations. Leaving it unset uses the Pinpoint
    // factory calibration, which is usually correct.
    // static { localizerConstants.yawScalar(1.0); }

    // ---------------------------------------------------------------------------------
    // Path constraints: when the follower considers a path finished
    // ---------------------------------------------------------------------------------

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,  // t-value at which the path counts as complete
            0.1,    // velocity below which the robot counts as stopped (in/s)
            0.1,    // translational error tolerance (in)
            0.007,  // heading error tolerance (rad)
            100,    // timeout after reaching the end, before giving up (ms)
            1,      // braking strength
            10,     // Bezier curve search limit
            1       // braking start
    );

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
