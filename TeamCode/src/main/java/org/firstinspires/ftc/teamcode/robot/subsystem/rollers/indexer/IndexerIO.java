package org.firstinspires.ftc.teamcode.robot.subsystem.rollers.indexer;

import org.firstinspires.ftc.teamcode.robot.subsystem.rollers.intake.IntakeIO;
import org.firstinspires.ftc.teamcode.util.control.PIDConstants;
import org.firstinspires.ftc.teamcode.util.loggerUtil.inputs.LoggedMotor;
import org.firstinspires.ftc.teamcode.util.loggerUtil.inputs.LoggedServo;

import Ori.Coval.Logging.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        boolean indexerConnected = false;
        LoggedServo indexer = new LoggedServo();
    }
    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setPosition(double position) {}
}
