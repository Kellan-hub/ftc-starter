package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * A minimal Pedro Pathing autonomous: drive out, curve to a second point, then come back.
 *
 * <p>This is a template to copy, not a competition routine. The shape of it &mdash; a state
 * number, a {@code switch} that starts the next path once the follower is no longer busy, and
 * a single {@code follower.update()} per loop &mdash; is the pattern to keep.
 *
 * <p>Run this only after the Tuning OpMode has given you real values in {@link Constants};
 * with the stock numbers the robot will not track these paths accurately.
 */
@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose scorePose = new Pose(24, 0, Math.toRadians(0));
    private final Pose parkControl = new Pose(24, 24, Math.toRadians(0));
    private final Pose parkPose = new Pose(0, 24, Math.toRadians(90));

    private PathChain driveOut;
    private PathChain curveToPark;
    private PathChain returnHome;

    private void buildPaths() {
        driveOut = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        curveToPark = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, parkControl, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

        returnHome = follower.pathBuilder()
                .addPath(new BezierLine(parkPose, startPose))
                .setLinearHeadingInterpolation(parkPose.getHeading(), startPose.getHeading())
                .build();
    }

    /** Advances the state machine. Each case starts one path and moves on. */
    private void updatePathState() {
        switch (pathState) {
            case 0:
                follower.followPath(driveOut);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(curveToPark, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(returnHome, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            default:
                // -1 means done; hold position.
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Paths built. Press play to run.");
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        updatePathState();

        Pose pose = follower.getPose();
        telemetry.addData("path state", pathState);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }
}
