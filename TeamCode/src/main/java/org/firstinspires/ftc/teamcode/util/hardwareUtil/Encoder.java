package org.firstinspires.ftc.teamcode.util.hardwareUtil;

import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;

import dev.nextftc.hardware.impl.MotorEx;

public interface Encoder {
    public default double getPosition() {}

    public default double getVelocity() {}

    public default void setPosition(double position) {}
}
