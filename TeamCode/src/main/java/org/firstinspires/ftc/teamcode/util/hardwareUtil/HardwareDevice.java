package org.firstinspires.ftc.teamcode.util.hardwareUtil;

import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;

public class HardwareDevice {
    private final String name;

    public HardwareDevice(String name) {
        this.name = name;
    }

    public MotorExEx motor() {
        return new MotorExEx(name);
    }

    public ServoEx servo() {
        return new ServoEx(name);
    }

    public CRServoEx crservo() {
        return new CRServoEx(name);
    }
}
