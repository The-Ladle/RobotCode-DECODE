package org.firstinspires.ftc.teamcode.opmode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.rollers.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.VisionDistanceHelper;

import java.util.List;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "blueCornerSpam")
public class BlueCornerSpam extends NextFTCOpMode {


    public BlueCornerSpam() {


        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(LauncherOuttakeFuckingThing.INSTANCE),
                BindingsComponent.INSTANCE

        );
    }

    // ======================
    // PATH / POSE COMMANDS
    // ======================

    // 1) go grab first corner stack


    public static final Command movefirst = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(48.000, 7.000), new Pose(59.000, 22.000))
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    public static final Command grabfirst_1 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(59.000, 22.000),
                                                new Pose(34.627, 13.151),
                                                new Pose(11.000, 15.000)
                                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // grabfirst
    public static final Command grabfirst_2 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(   new BezierCurve(
                                                new Pose(11.000, 15.000),
                                                new Pose(35.126, 11.320),
                                                new Pose(11.000, 8.000)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build(),
                        1, false

                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

 /*   public static final Command grabfirst_3 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(9.000, 25.000),
                                        new Pose(9.000, 14.000)))
                                .setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(240))
                                .build(),
                        1, false

                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());
    public static final Command grabfirst_4 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(9.000, 14.000),
                                                new Pose(9.000, 10.000)))
                                .setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(270))
                                .build(),
                        1, false

                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy()); */

    // 3) grab from corner #1 again
    public static final Command scoreFirst = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(9.000, 10.000),
                                                new Pose(59.000, 22.000))
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 4) score from corner #1
    public static final Command grabsecond = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(59.000, 22.000),
                                                new Pose(10.000, 15.000))
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 5) transition path toward second corner
    public static final Command grabsecondwiggle = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(10.000, 15.000),
                                                new Pose(25.138, 19.477),
                                                new Pose(9.822, 15.803)
                                        ))

                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 6) grab from second corner
    public static final Command scoresecond = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(9.822, 25.803),
                                                new Pose(59.000, 22.000))
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());



    // 7) final score from second corner
    public static final Command grabthird = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(59.000, 22.000),
                                                new Pose(10.000, 30.000))

                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabthirdwiggle = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(10.000, 30.000),
                                                new Pose(25.138, 19.477),
                                                new Pose(9.822, 15.803)
                                        ))

                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scorethird = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(9.822, 15.803),
                                                new Pose(59.000, 22.000))
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command move = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(59, 22),
                                                new Pose(30.000, 22.000))
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());




    @Override
    public void onInit() {
        // Disable relocalization for this auto (same as artifact autos)
        Turret.INSTANCE.limelight.pipelineSwitch(1);
        VisionDistanceHelper.RELOCALIZATION_ENABLED = false;

        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed);
        Turret.INSTANCE.updateLimelightAim(.02);
        Turret.INSTANCE.enableAutoAim(true);
        Turret.INSTANCE.LIMELIGHT_X_OFFSET_DEG = 2;

        VisionDistanceHelper.GOAL_TAG_X_IN =  144 - 127.64;



        // Kick the Limelight once (matches your other autos)
        LLResult result = Turret.INSTANCE.runLimelight();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
    }

    @Override
    public void onStartButtonPressed() {
        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
        LauncherOuttakeFuckingThing.autoCalculate = true;


        LLResult result = Turret.INSTANCE.runLimelight();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        Follower follower = PedroComponent.follower();

        // Start pose for this corner auto: first path's start pose + heading 180
        follower.setPose(new Pose(48.0, 7.0, Math.toRadians(180)));


        // Skeleton auto sequence.
        // You can now drop in the exact Intake/Turret/Launcher logic you use
        // in blueArtifactSpamSTATIC around these commands.
        Command auto = new SequentialGroup(
                Intake.INSTANCE.indexerIn,

                // Example: preload setup if you want it; copy your artifact spam here
                new LambdaCommand().setStart(() -> {
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                    Turret.INSTANCE.enableAutoAim(true);
                    // Example shooter/intake prep – customize as needed
                    LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed);
                //    LauncherOuttakeFuckingThing.INSTANCE.setFallback(3600, 41.5);
                    Turret.INSTANCE.enableAutoAim(true);
                }),
                Intake.INSTANCE.intakeOneZero,
                Intake.INSTANCE.intakeTwoZero,
                movefirst,


                Intake.INSTANCE.intakeOnePowerFull,
                Intake.INSTANCE.intakeTwoPowerFull,

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                new Delay(2),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                grabfirst_1,
                grabfirst_2,

                new Delay(.2),
                Intake.INSTANCE.intakeTwoZero,
                scoreFirst,


                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new LambdaCommand().setStart(() ->
                        Turret.INSTANCE.snapToRememberedGoalAndEnable()
                ),
                new Delay(1),

                Intake.INSTANCE.intakeTwoPowerFull,

                new Delay(2),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                grabsecond,
                grabsecondwiggle,
                new Delay(.75),
                Intake.INSTANCE.intakeTwoZero,

                new LambdaCommand()
                        .setStart(() -> {
                            Turret.INSTANCE.enableAutoAim(false);
                            Turret.INSTANCE.setManualAngle(-60);
                        }),

                scoresecond,
                new LambdaCommand().setStart(() ->
                        Turret.INSTANCE.snapToRememberedGoalAndEnable()
                ),
                new Delay(.5),
                Intake.INSTANCE.intakeTwoPowerFull,

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(2),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                grabthird,
                grabthirdwiggle,
                Intake.INSTANCE.intakeTwoZero,
                scorethird,

                new LambdaCommand().setStart(() ->
                        Turret.INSTANCE.snapToRememberedGoalAndEnable()
                ),
                new Delay(.5),
                Intake.INSTANCE.intakeTwoPowerFull,

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(2),
                move




        //Intake.INSTANCE.intakeTwoPowerFull

                //   Path5,
                // grabcorener2,
                //scorecorner3,


        );


        auto.schedule();
    }

    @Override
    public void onUpdate () {
        BindingManager.update();

        Pose pedroPose = follower().getPose();

        // Use local instance 'turret'
        LLResult result = Turret.INSTANCE.limelight.getLatestResult();

        double turretAngleDeg = Turret.INSTANCE.getMeasuredAngleDeg();

        double distLL = VisionDistanceHelper.distanceToGoalFromLimelight(result, turretAngleDeg);


        // --- DASHBOARD FIELD MAP DRAWING ---
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        double x = pedroPose.getX();       // assumed inches in Pedro frame
        double y = pedroPose.getY();
        double h = pedroPose.getHeading(); // radians

        double robotRadius = 9.0; // ~9in radius for visualization

        // Draw a circle for the robot
        field.strokeCircle(x, y, robotRadius);

        // Draw a heading line
        double lineLen = robotRadius * 1.2;
        double hx = x + lineLen * Math.cos(h);
        double hy = y + lineLen * Math.sin(h);
        field.strokeLine(x, y, hx, hy);

        telemetry.addData("LL distance to goal (in)", distLL);
        telemetry.addData("target RPM", LauncherOuttakeFuckingThing.INSTANCE.getTargetRpm());
        telemetry.addData("motor rpm", LauncherOuttakeFuckingThing.INSTANCE.getCurrentRpm());
        telemetry.addData("turret_angle_deg", Turret.INSTANCE.getMeasuredAngleDeg());
        telemetry.addData("turret_volts", Turret.INSTANCE.turretFeedback.getVoltage());
        telemetry.addData("turret_state", Turret.INSTANCE.turretStateString());
        telemetry.addData("imu", Turret.INSTANCE.getRobotHeadingDeg());
        telemetry.addData("X", pedroPose.getX());
        telemetry.addData("Y", pedroPose.getY());

        telemetry.update();

    }
}

