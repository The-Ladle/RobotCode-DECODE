package org.firstinspires.ftc.teamcode.util.hardwareUtil;

import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;

import dev.nextftc.hardware.impl.MotorEx;

public class Encoder {
    private final MotorEx port;
    private GearRatio ratio = new GearRatio(1);

    public Encoder(String name) {
        port = new MotorEx(name);
    }

    public double getPosition() {
        return ratio.applySigned(port.getCurrentPosition());
    }

    public double getVelocity() {
        return ratio.applySigned(port.getVelocity());
    }

    public void setPosition(double position) {
        port.setCurrentPosition(ratio.inverse().applySigned(position));
    }
}
