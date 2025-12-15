package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.rollers.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.VisionDistanceHelper;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;


@Config
@TeleOp(name = "red TeleOp")
public class redGregTeleOp extends NextFTCOpMode {
    // Reverted: Define turret as an instance variable so we can create a fresh one


    private DriverControlledCommand driveCmd;
    public static double turretAngle = 0;

    private FtcDashboard dashboard;

    public redGregTeleOp() {

        VisionDistanceHelper.GOAL_TAG_X_IN =  127.64;

        addComponents(
                new PedroComponent(Constants::createFollower),
                // Pass the specific instance we just created
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(LauncherOuttakeFuckingThing.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        follower().setPose(new Pose(0, 0, 0));
        follower().update();

        dashboard = FtcDashboard.getInstance();
        Turret.INSTANCE.limelight.pipelineSwitch(0);
        Turret.INSTANCE.LIMELIGHT_X_OFFSET_DEG = -2;



        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Turret.INSTANCE.limelight.pipelineSwitch(0);


        LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(0.0);


    }

    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.enableLimelightAim();
        Turret.INSTANCE.enableAutoAim(true);



        LauncherOuttakeFuckingThing.autoCalculate = true;
        follower().startTeleopDrive();
        // Use local instance 'turret' instead of static 'Turret.INSTANCE'

        driveCmd = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()
        );

        driveCmd.schedule();

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Intake.INSTANCE.intakeOnePowerFull)
                .whenBecomesFalse(Intake.INSTANCE.intakeOneZero);
        Gamepads.gamepad1().leftTrigger().greaterThan(.5)
                .whenBecomesTrue(Intake.INSTANCE.intakeTwoPowerFull)
                .whenBecomesFalse(Intake.INSTANCE.intakeTwoZero)
                .whenBecomesFalse(new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)));

        Gamepads.gamepad1().x()
                .whenBecomesTrue(Intake.INSTANCE.indexerIn);

        Gamepads.gamepad1().y()
                .whenBecomesTrue(Intake.INSTANCE.indexerOut);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.5)
                .whenBecomesTrue(new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM)))
                .whenBecomesFalse(new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.SLOW_RPM)));

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(
                        // Use local instance method reference
                        new LambdaCommand().setStart(Turret.INSTANCE::snapToRememberedGoalAndEnable))
                .whenBecomesTrue(new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)));


        Gamepads.gamepad2().triangle()
                .whenBecomesTrue(new SequentialGroup(
                        // Step 1: Set Shooter Fallback & Start moving Turret to 0

                        new LambdaCommand()
                                .setStart(() -> {
                                    Turret.INSTANCE.enableAutoAim(false);

                                    // Set Shooter to static fallback
                                    LauncherOuttakeFuckingThing.INSTANCE.setManualShooter(
                                            LauncherOuttakeFuckingThing.FALLBACK_RPM,
                                            LauncherOuttakeFuckingThing.FALLBACK_ANGLE
                                    );
                                    // Command Turret to center
                                    Turret.INSTANCE.setManualAngle(0.0);
                                })
                                // This command counts as "finished" only when the turret is close to 0
                                .setIsDone(() -> Math.abs(Turret.INSTANCE.getMeasuredAngleDeg()) < 5.0),

                        // Step 2: Once the turret is centered, cut the power
                        new LambdaCommand().setStart(() -> Turret.INSTANCE.off())
                ));

        Gamepads.gamepad2().square()
                .whenBecomesTrue(new LambdaCommand().setStart(() -> {
                    // 1. Turn Shooter Vision Calculation back ON
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();

                    // 2. Turn Turret Tracking back ON
                    // This wakes it up from "OFF" and sets state to "LIMELIGHT"
                    Turret.INSTANCE.enableAutoAim(true);
                }));

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(Intake.INSTANCE.extendClimb1)
                .whenBecomesTrue(Intake.INSTANCE.extendClimb2);

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(Intake.INSTANCE.retractClimb1)
                .whenBecomesTrue(Intake.INSTANCE.retractClimb2);
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
     //   telemetry.addData("turret_volts", Turret.INSTANCE.turretFeedback.getVoltage());
        telemetry.addData("turret_state", Turret.INSTANCE.turretStateString());
        telemetry.addData("imu", Turret.INSTANCE.getRobotHeadingDeg());
        telemetry.addData("X", pedroPose.getX());
        telemetry.addData("Y", pedroPose.getY());

        telemetry.update();

        dashboard.sendTelemetryPacket(packet);
    }
}