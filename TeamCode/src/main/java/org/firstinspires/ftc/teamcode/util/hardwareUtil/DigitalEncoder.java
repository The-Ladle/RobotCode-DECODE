package org.firstinspires.ftc.teamcode.util.hardwareUtil;

import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.robot.constants.HardwareDevices;
import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;

public class DigitalEncoder implements Encoder {
    private final MotorEx port;
    private GearRatio ratio = new GearRatio(1);
    private double offset = 0;

    public DigitalEncoder(String name) {
        port = new MotorEx(HardwareDevices.hardwareMap, name);
    }

    public DigitalEncoder(String name, GearRatio gearRatio) {
        this.port = new MotorEx(HardwareDevices.hardwareMap, name);
        this.ratio = gearRatio;
    }

    @Override
    public double getPosition() {
        return ratio.applySigned(port.getCurrentPosition() + offset);
    }

    @Override
    public double getVelocity() {
        return ratio.applySigned(port.getVelocity());
    }

    @Override
    public double getAcceleration() { return ratio.applySigned(port.getAcceleration()); }

    @Override
    public void setPosition(double position) { offset = position - port.getCurrentPosition(); }
}
