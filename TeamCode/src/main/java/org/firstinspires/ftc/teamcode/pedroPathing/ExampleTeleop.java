package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * A field-centric mecanum TeleOp driven through the Pedro Pathing follower.
 *
 * <p>Because the follower owns the drivetrain, localization keeps running the whole match,
 * which is what makes the pose readout on FTC Dashboard useful during driver practice.
 *
 * <p>Controls:
 * <ul>
 *   <li>Left stick: translate</li>
 *   <li>Right stick X: turn</li>
 *   <li>Right bumper: hold for slow mode</li>
 *   <li>Options: reset field-centric zero to the current heading</li>
 * </ul>
 */
@TeleOp(name = "Example TeleOp", group = "Examples")
public class ExampleTeleop extends OpMode {

    /** How much of full power slow mode gives you. */
    public static double SLOW_MODE_SCALE = 0.35;

    private Follower follower;

    /**
     * Where the robot starts the match. If your autonomous just ran, carry its final pose
     * over instead so field-centric drive and the dashboard stay aligned.
     */
    private final Pose startingPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Initialized. Press play to drive.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;

        // Field-centric: the last argument false means "not robot-centric".
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * scale,
                -gamepad1.left_stick_x * scale,
                -gamepad1.right_stick_x * scale,
                false);

        if (gamepad1.options) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }

        follower.update();

        Pose pose = follower.getPose();
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetry.addData("slow mode", gamepad1.right_bumper);
        telemetry.update();
    }
}
