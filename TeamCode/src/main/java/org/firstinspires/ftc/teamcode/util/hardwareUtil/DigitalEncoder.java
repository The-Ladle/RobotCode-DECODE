package org.firstinspires.ftc.teamcode.util.hardwareUtil;

import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;

import dev.nextftc.hardware.impl.MotorEx;

public class DigitalEncoder implements Encoder {
    private final MotorEx port;
    private GearRatio ratio = new GearRatio(1);

    public DigitalEncoder(String name) {
        port = new MotorEx(name);
    }

    public DigitalEncoder(String name, GearRatio gearRatio) {
        this.port = new MotorEx(name);
        this.ratio = gearRatio;
    }

    @Override
    public double getPosition() {
        return ratio.applySigned(port.getCurrentPosition());
    }

    @Override
    public double getVelocity() {
        return ratio.applySigned(port.getVelocity());
    }

    @Override
    public void setPosition(double position) {
        port.setCurrentPosition(ratio.inverse().applySigned(position));
    }
}
