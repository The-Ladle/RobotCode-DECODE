package org.firstinspires.ftc.teamcode.subsystem.rollers.intake;

import org.firstinspires.ftc.teamcode.constants.HardwareDevices;
import org.firstinspires.ftc.teamcode.util.hardwareUtil.MotorExEx;

public class IntakeIOMotor implements IntakeIO {
    private final MotorExEx motorOne = HardwareDevices.intakeOneMotorID.motor();
    private final MotorExEx motorTwo = HardwareDevices.intakeTwoMotorID.motor();

    public IntakeIOMotor() {
        var config =
    }
}
