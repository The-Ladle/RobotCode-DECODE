package org.firstinspires.ftc.teamcode.robot.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.robot.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystem.rollers.intake.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

public class SortedAutoRed extends NextFTCOpMode {
    private Pattern pattern = Pattern.GPP;
    private int swapStep = 0;
    private final Command swap;
    private final Command grab;
    private final Command launchOn;
    private final Command launchOff;
    public SortedAutoRed() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(LauncherOuttakeFuckingThing.INSTANCE),
                BindingsComponent.INSTANCE
        );

        Follower follower = PedroComponent.follower();
        launchOn = new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM));
        launchOff = new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.SLOW_RPM));

        grab = new SequentialGroup(Intake.INSTANCE.intakeOn)
                .then(new LambdaCommand().setStart(() ->
                follower.followPath(new Path(
                        new BezierLine(follower.getPose(), follower.getPose().plus(
                                new Pose(18, 0)
                        ))
                ))).setIsDone(() -> !follower.isBusy()))
                .afterTime(2)
                .then(Intake.INSTANCE.intakeOff);

        swap = new SequentialGroup(
                Intake.INSTANCE.intakeOn,
                grab
        )
                .then(Intake.INSTANCE.indexerIn)
                .then(launchOn)
                .then(Intake.INSTANCE.indexerOut)
                .afterTime(1)
                .then(Intake.INSTANCE.intakeOff, launchOff);
                // turn on intake, go forward, turn on indexer, launch, turn of indexer, launch

    }

    @Override
    public void onInit() {
        Turret.INSTANCE.manual();
        Turret.INSTANCE.setManualAngle(90);
    }

    /*      Field
     -Blue-       -Red-
    GPP --         -- PPG | 0
    PGP --         -- PGP | 1
    PPG --         -- GPP | 2
     */
    @Override
    public void onStartButtonPressed() {
        LLResult result = Turret.INSTANCE.runLimelight();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : tags) {
            int tagID = tag.getFiducialId();
            if (tagID == 21) { // GPP
                swapStep = 1;
            } else if (tagID == 22) { // PGP
                swapStep = 0;
            } else if (tagID == 23) { // PPG
                swapStep = 2;
            }
        }

        Turret.INSTANCE.snapToRememberedGoalAndEnable();

        new SequentialGroup(
                // path to 1
        )
                .then(testPattern(0))
                .then() // path to 2
                .then(testPattern(1))
                .then() // path to 3
                .then(testPattern(2))
                .schedule();

/*
        if (pattern == Pattern.GPP) {
            // path to 1
            grab.schedule();
            // launch
            // path to 2
            swap.schedule();
            // path to 3
            grab.schedule();
            // launch
        } else if (pattern == Pattern.PGP) {

        }
*/
    }

    private Command testPattern(int pattern) {
        if (swapStep == pattern) {
            return swap;
        }

        return new SequentialGroup(grab)
                .then(launchOn)
                .afterTime(1)
                .then(launchOff);
    }

    private enum Pattern {
        GPP,
        PGP,
        PPG
    }
}
