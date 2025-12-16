package org.firstinspires.ftc.teamcode.util.hardwareUtil;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.robot.constants.HardwareDevices;
import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;

public class AnalogEncoder implements Encoder {
    private final AnalogInput port;
    private GearRatio ratio = new GearRatio(1);
    private double offset = 0;

    public AnalogEncoder(String name) {
        port = new MotorEx(HardwareDevices.hardwareMap, name);
    }

    public AnalogEncoder(String name, GearRatio gearRatio) {
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
    public void setPosition(double position) { offset = position - port.getCurrentPosition(); }
}
