package org.firstinspires.ftc.teamcode.util.config;

import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.util.hardwareUtil.Encoder;
import org.firstinspires.ftc.teamcode.util.hardwareUtil.MotorExEx;
import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;

import java.util.Optional;

public class MotorConfiguration {
    private InvertType inverted;
    private GearRatio ratio;
    private ControlType controlType;
    private Motor.ZeroPowerBehavior zeroPowerBehavior;
    private Optional<MotorExEx> leader = Optional.empty();
    private Optional<Encoder> encoder = Optional.empty();

    public MotorConfiguration() {}

    public void setInverted(InvertType type) {
        this.inverted = type;
    }

    public void setRatio(GearRatio ratio) {
        this.ratio = ratio;
    }

    public void setRawRatio(double reduction) {
        this.ratio = new GearRatio(reduction);
    }

    public void setControlType(ControlType controlType) {
        this.controlType = controlType;
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
    }
    /**
     * Must call MotorExEx.periodic() to function properly
     */
    public void setLeader(MotorExEx leader) {
        this.leader = Optional.of(leader);
    }

    public void setExternalEncoder(Encoder encoder) {
        this.encoder = Optional.of(encoder);
    }

    public InvertType getInverted() {
        return inverted;
    }

    public GearRatio getRatio() {
        return ratio;
    }

    public ControlType getControlType() {
        return controlType;
    }

    public Motor.ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    public Optional<MotorExEx> getLeader() {
        return this.leader;
    }

    public Optional<Encoder> getEncoder() { return this.encoder; }
}
