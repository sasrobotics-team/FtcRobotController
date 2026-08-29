package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "PedroAuto", group = "Auto")
public class PedroAuto extends LinearOpMode {

    private Follower follower;
    private Paths paths;

    private static final double MAX_SPEED = 0.5;
    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(MAX_SPEED);

        paths = new Paths(follower);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        runPath(paths.MainChain);            // segments 1–5, no rotation
        turnInPlace(paths.finalHeading);     // rotate, no translation
        runPath(paths.FinalSegment);         // segment 6, no rotation
    }

    public static class Paths {
        public PathChain MainChain;
        public PathChain FinalSegment;
        public final double finalHeading;

        public Paths(Follower follower) {
            finalHeading = Math.atan2(31.633 - 44.465, 83.105 - 69.628);  // ≈ −43.6°

            MainChain = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000,  8.000),  new Pose(55.822,  56.667)))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .addPath(new BezierLine(new Pose(55.822,  56.667), new Pose(105.195, 55.588)))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .addPath(new BezierLine(new Pose(105.195, 55.588), new Pose(105.412, 12.754)))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .addPath(new BezierLine(new Pose(105.412, 12.754), new Pose(70.150,  13.048)))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .addPath(new BezierLine(new Pose(70.150,  13.048), new Pose(69.628,  44.465)))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .setTimeoutConstraint(700)
                    .build();

            FinalSegment = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(69.628, 44.465), new Pose(83.105, 31.633)))
                    .setConstantHeadingInterpolation(finalHeading)
                    .setTimeoutConstraint(700)
                    .build();
        }
    }

    private void runPath(PathChain chain) {
        follower.followPath(chain, MAX_SPEED, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    private void turnInPlace(double targetHeadingRad) {
        follower.turnTo(targetHeadingRad);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }
}
