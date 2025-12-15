package org.firstinspires.ftc.teamcode.util.hardwareUtil;

import org.firstinspires.ftc.teamcode.util.config.MotorConfiguration;
import org.firstinspires.ftc.teamcode.util.control.FF;
import org.firstinspires.ftc.teamcode.util.control.PID;
import org.firstinspires.ftc.teamcode.util.control.PIDConstants;
import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;

import java.util.Optional;

import dev.nextftc.hardware.impl.MotorEx;

public final class MotorExEx {
    private final MotorEx motor;
    private GearRatio ratio = new GearRatio(1);
    private Optional<PID> pid;
    private Optional<FF> ff;
    private Optional<MotionProfile> motionProfile;
    private Optional<MotorExEx> leader;
    private Optional<Encoder> externalEncoder;

    public MotorExEx(String name) {
        this.motor = new MotorEx(name);
    }

    /**
     * NEEDS to be called if using as a follower motor, using an external encoder, or running a "on-device" control loop. Doesn't need to be called with otherwise.
     */
    public void periodic() {
        if (pid.isPresent() && ff.isEmpty()) {
            double pidCalc = pid.get().calculate();
            this.setPower(pidCalc);
        } else if (pid.isPresent()) {
            double ffCalc = ff.get().calculate();
            double pidCalc = pid.get().calculate();
        } else if (ff.isPresent()) {
            double ffCalc = ff.get().calculate();
            this.setPower(ffCalc);
        }
        leader.ifPresent(motorExEx -> this.motor.setPower(motorExEx.getPower()));
    }

    /**
     * To use a gear ratio in own logic, set the ratio in the config to 1 using new GearRatio(1)
     */
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

    public double getVelocity() {
        return ratio.applySigned(motor.getVelocity());
    }

    public void updatePID(PIDConstants pidConstants) {
        if(this.pid.isPresent()) {
            this.pid.get().setConstants(pidConstants);
        } else {
            this.pid = Optional.of(new PID());
        }
    }

    public void updateTarget(double target) {
        this.pid.ifPresent((loop) -> loop.setSetPoint(target));
    }
}
