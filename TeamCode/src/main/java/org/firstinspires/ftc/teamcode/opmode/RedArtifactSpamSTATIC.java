package org.firstinspires.ftc.teamcode.opmode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "redArtifactSpamSTATIC")
public class RedArtifactSpamSTATIC extends NextFTCOpMode {

    public RedArtifactSpamSTATIC() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(LauncherOuttakeFuckingThing.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    public static final Command scoreFirst1 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(17.8, 118).mirror(),
                                                new Pose(48.000, 96.000).mirror()
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 2) go grab first stack
    public static final Command grabfirst2 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(48.000, 96.000).mirror(),
                                                new Pose(69.753, 81.739).mirror(),
                                                new Pose(15.000, 84.000).mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        .6,
                        false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 3) score first stack
    public static final Command scorefirst3 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(15.000, 84.000).mirror(),
                                                new Pose(55.000, 84.000).mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                // .setReversed()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 4) grab second stack
    public static final Command grabsecond4 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(55.000, 84.000).mirror(),
                                                new Pose(59.598, 52.772).mirror(),
                                                new Pose(40.786, 60.264).mirror(),
                                                new Pose(21.309, 60.097).mirror(),
                                                new Pose(43.616, 58.765).mirror(),
                                                new Pose(14.150, 59.931).mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        .6,
                        false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabsecondgate5 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(19.150, 56.931).mirror(),
                                                new Pose(26.969, 54.437).mirror(),
                                                new Pose(31.464, 62.594).mirror(),
                                                new Pose(15.000, 69.420).mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        .6,
                        false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 5) score second stack
    public static final Command scoresecond5 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(15.000, 69.420).mirror(),
                                                new Pose(55.000, 84.000).mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                // .setReversed()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 6) grab from gate
    public static final Command grabthird1_7 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(55.000, 84.000).mirror(),
                                                new Pose(63.760, 36.791).mirror(),
                                                new Pose(44.615, 35.292).mirror()
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 7) score from gate
    public static final Command grabthird2_8 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(44.615, 35.292).mirror(),
                                                new Pose(11.487, 35.292).mirror())
                                )
                                .setTangentHeadingInterpolation()

                                .build(),
                        .6, false);

            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scorethird = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(11.487, 35.292).mirror(),
                                                new Pose(55.000, 84.000).mirror())

                                )
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    @Override
    public void onInit() {
        //Turret.INSTANCE.updateLimelightAim(.02);
        Turret.INSTANCE.enableAutoAim(true);
        // 1. DISABLE Relocalization globally
        VisionDistanceHelper.RELOCALIZATION_ENABLED = false;
        VisionDistanceHelper.GOAL_TAG_X_IN =  127.64;


        // 2. ENABLE Auto Calculation for RPM/Angle and fallback (only for this auto)
       // LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
        Turret.INSTANCE.limelight.pipelineSwitch(0);
        Turret.INSTANCE.LIMELIGHT_X_OFFSET_DEG = -3;
        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed);

        LLResult result = Turret.INSTANCE.runLimelight();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
    }

    @Override
    public void onStartButtonPressed() {
        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
        LauncherOuttakeFuckingThing.autoCalculate = true;

        // Tell Pedro where we actually are at the start (artifact pile)
        Follower follower = PedroComponent.follower();
        follower.setPose(new Pose(17.8, 118, Math.toRadians(144)).mirror());

        Command auto = new SequentialGroup(
                Intake.INSTANCE.intakeOneZero,
                Intake.INSTANCE.intakeTwoZero,
                // Spin up + clamp for first shot
                Intake.INSTANCE.indexerIn,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                // For the first scoring position: if vision is dead, use 2500 / 25
                new LambdaCommand().setStart(() -> {
                    LauncherOuttakeFuckingThing.INSTANCE.setFallback(2600, 27);
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                }),
                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableAutoAim(true)),

                Intake.INSTANCE.intakeOnePowerFull,

                // Drive out to score first artifact
                scoreFirst1,

                // Make sure we're still in auto-calc mode as we approach
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation()
                ),
                Intake.INSTANCE.intakeTwoPowerFull,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),

                new Delay(1.5),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                // Go grab first stack
                grabfirst2,
                Intake.INSTANCE.intakeTwoZero,

                // Drive back to score first stack
                scorefirst3,

                // For this shot, use a tuned fallback if vision is missing
                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setFallback(2600, 27)
                ),
                new LambdaCommand().setStart(() ->
                        Turret.INSTANCE.snapToRememberedGoalAndEnable()
                ),

                new Delay(.75),

                Intake.INSTANCE.intakeTwoPowerFull,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                new Delay(1.5),
                Intake.INSTANCE.intakeTwoZero,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                Intake.INSTANCE.intakeTwoPowerFull,

                // Go grab second stack
                grabsecond4,
                grabsecondgate5,
                new Delay(1.5),
                Intake.INSTANCE.intakeTwoZero,

                // Drive back to score second stack
                scoresecond5,

                // Second stack shot fallback
                    new LambdaCommand().setStart(() ->
                            LauncherOuttakeFuckingThing.INSTANCE.setFallback(2600, 27)
                ),
                new LambdaCommand().setStart(() ->
                        Turret.INSTANCE.snapToRememberedGoalAndEnable()
                ),

                new Delay(.75),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                Intake.INSTANCE.intakeTwoPowerFull,
                new Delay(1.5),
                Intake.INSTANCE.intakeTwoZero,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                // Grab from gate, then score
                grabthird1_7,
                Intake.INSTANCE.intakeTwoPowerFull,
                grabthird2_8,
                new Delay(.75),
                Intake.INSTANCE.intakeTwoZero,
                Intake.INSTANCE.intakeOneZero,

                scorethird,

                // Third shot fallback
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setFallback(2600, 27)
                ),
                new LambdaCommand().setStart(() ->
                        Turret.INSTANCE.snapToRememberedGoalAndEnable()
                ),
                new Delay(1),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                Intake.INSTANCE.intakeOnePowerFull,
                Intake.INSTANCE.intakeTwoPowerFull
        );

        auto.schedule();
    }
    public void onUpdate() {

        BindingManager.update();

        Pose pedroPose = follower().getPose();

        // Use local instance 'turret'
        LLResult result = Turret.INSTANCE.limelight.getLatestResult();

        double turretAngleDeg = Turret.INSTANCE.getMeasuredAngleDeg();

        double distLL = VisionDistanceHelper.filteredDistanceToGoalFromLimelight(result, turretAngleDeg);

        double x = pedroPose.getX();       // assumed inches in Pedro frame
        double y = pedroPose.getY();
        double h = pedroPose.getHeading(); // radians

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
