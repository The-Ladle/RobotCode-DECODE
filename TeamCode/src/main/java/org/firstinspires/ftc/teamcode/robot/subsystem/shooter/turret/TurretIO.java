package org.firstinspires.ftc.teamcode.robot.subsystem.shooter.turret;

import org.firstinspires.ftc.teamcode.util.loggerUtil.inputs.LoggedCRServo;
import org.firstinspires.ftc.teamcode.util.loggerUtil.inputs.LoggedServo;

import Ori.Coval.Logging.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        boolean servoOneConnected = false;
        LoggedCRServo servoOne = new LoggedCRServo();
        boolean servoTwoConnected = false;
        LoggedCRServo servoTwo = new LoggedCRServo();
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setTargetPosition(double radians) {}
}
