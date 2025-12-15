package org.firstinspires.ftc.teamcode.robot.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robot.subsystem.LauncherOuttakeFuckingThing;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.PedroComponent;

public class SortedAutoPaths {

    public static final double FIRST_SCORING_HEADING_BLUE = Math.toRadians(135);
    public static final Pose FIRST_SCORING_POSE_BLUE =
            new Pose(48.000, 96.000, FIRST_SCORING_HEADING_BLUE);

    public static final Command GPP = new LambdaCommand().setStart(() -> {
        Follower follower = PedroComponent.follower();
        follower.followPath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(FIRST_SCORING_POSE_BLUE, new Pose(44.658, 36.000))
                        )
                        .setLinearHeadingInterpolation(FIRST_SCORING_HEADING_BLUE, Math.toRadians(180))
                        .addPath(
                                new BezierLine(new Pose(44.658, 36.000), new Pose(14.127, 35.544))
                        )
                        .setTangentHeadingInterpolation()
                        .addPath(
                                new BezierLine(new Pose(14.127, 35.544), new Pose(36.000, 107.089))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }).setStop(bool -> LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM));

    // PGP
    public static final Command PGP = new LambdaCommand().setStart(() -> {
        Follower follower = PedroComponent.follower();
        follower.followPath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(FIRST_SCORING_POSE_BLUE, new Pose(44.886, 59.696))
                        )
                        .setLinearHeadingInterpolation(FIRST_SCORING_HEADING_BLUE, Math.toRadians(180))
                        .addPath(
                                new BezierLine(new Pose(44.886, 59.696), new Pose(14.354, 59.468))
                        )
                        .setTangentHeadingInterpolation()
                        .addPath(
                                new BezierLine(new Pose(14.354, 59.468), new Pose(22.329, 59.468))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addPath(
                                new BezierLine(new Pose(22.329, 59.468), new Pose(35.772, 106.633))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }).setStop(bool -> LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM));

    // PPG
    public static final Command PPG = new LambdaCommand().setStart(() -> {
        Follower follower = PedroComponent.follower();

        // Build ONE PathChain that runs all your segments in order:
     follower.pathBuilder()

                // startingppg
                .addPath(
                        new BezierCurve(
                                FIRST_SCORING_POSE_BLUE,
                                new Pose(63.760, 83.736),
                                new Pose(16.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))

                // scoringfirst
                .addPath(
                        new BezierLine(
                                new Pose(16.000, 84.000),
                                new Pose(48.000, 96.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(240))

                // Path3
                .addPath(
                        new BezierLine(
                                new Pose(48.000, 96.000),
                                new Pose(52.772, 75.080)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(290))

                // tograbgpp
                .addPath(
                        new BezierCurve(
                                new Pose(52.772, 75.080),
                                new Pose(61.928, 31.464),
                                new Pose(15.000, 36.000)
                        )
                )
                .setTangentHeadingInterpolation()

                // scoregpp
                .addPath(
                        new BezierCurve(
                                new Pose(15.000, 36.000),
                                new Pose(69.586, 45.947),
                                new Pose(37.290, 88.231),
                                new Pose(48.000, 96.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(250))
                .setReversed()  // if this overload complains, try .setReversed()

                // Path6
                .addPath(
                        new BezierCurve(
                                new Pose(48.000, 96.000),
                                new Pose(51.274, 76.245),
                                new Pose(41.785, 59.931)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(180))

                // grabthelastone
                .addPath(
                        new BezierLine(
                                new Pose(41.785, 59.931),
                                new Pose(16.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                // scorelastone
                .addPath(
                        new BezierLine(
                                new Pose(16.000, 60.000),
                                new Pose(48.000, 96.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-130))

                .build();
    }).setStop(bool ->
            LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM)
    );
}
