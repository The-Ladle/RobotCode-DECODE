package org.firstinspires.ftc.teamcode.robot.constants;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardwareUtil.HardwareDevice;

public class HardwareDevices {
    public static HardwareMap hardwareMap;

    public static final HardwareDevice intakeOneMotorID = new HardwareDevice("intake_one");
    public static final HardwareDevice intakeTwoMotorID = new HardwareDevice("intake_two");

    public static final HardwareDevice indexerServoID = new HardwareDevice("indexer");
}
