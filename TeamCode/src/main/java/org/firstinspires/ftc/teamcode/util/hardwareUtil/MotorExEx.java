package org.firstinspires.ftc.teamcode.util.hardwareUtil;

import org.firstinspires.ftc.teamcode.util.config.MotorConfiguration;
import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;

import java.util.Optional;

import dev.nextftc.hardware.impl.MotorEx;

public final class MotorExEx {
    private final MotorEx motor;
    private GearRatio ratio = new GearRatio(1);
    private Optional<MotorExEx> leader;
    private Optional<Encoder> externalEncoder;

    public MotorExEx(String name) {
        this.motor = new MotorEx(name);
    }

    /**
     * NEEDS to be called if using as a follower motor, using an external encoder, or running a "on-device" PID. Doesn't need to be called with otherwise.
     */
    public void periodic() {
        leader.ifPresent(motorExEx -> this.motor.setPower(motorExEx.getPower()));
    }

    public void applyConfig(MotorConfiguration configuration) {
        motor.setDirection(configuration.getInverted().ordinal());
        ratio = configuration.getRatio();
        leader = configuration.getLeader();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getCurrentPosition() {
        return ratio.applySigned(motor.getCurrentPosition());
    }

    public double getPower() {
        return motor.getPower();
    }
}
