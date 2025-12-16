package org.firstinspires.ftc.teamcode.util.hardwareUtil;

import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.robot.constants.HardwareDevices;

public class HardwareDevice {
    private final String name;

    public HardwareDevice(String name) {
        this.name = name;
    }

    public MotorExEx motor() {
        return new MotorExEx(name);
    }

    public ServoEx servo() {
        return new ServoEx(HardwareDevices.hardwareMap, name);
    }

    public CRServoEx crservo() {
        return new CRServoEx(HardwareDevices.hardwareMap, name);
    }

    public DigitalEncoder digitalEncoder() {
        return new DigitalEncoder(name);
    }

    public AnalogEncoder analogEncoder() { return new AnalogEncoder(name); }
}
