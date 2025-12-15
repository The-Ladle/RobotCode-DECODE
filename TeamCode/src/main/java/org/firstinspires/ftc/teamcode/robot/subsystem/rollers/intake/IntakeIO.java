package org.firstinspires.ftc.teamcode.robot.subsystem.rollers.intake;

import org.firstinspires.ftc.teamcode.util.loggerUtil.inputs.LoggedMotor;

import Ori.Coval.Logging.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        boolean intakeOneConnected = false;
        LoggedMotor intakeOne = new LoggedMotor();
        boolean intakeTwoConnected = false;
        LoggedMotor intakeTwo = new LoggedMotor();
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setRawSpeed(double speed) {}

    public default void setSurfaceSpeed(double speed) {}
}
