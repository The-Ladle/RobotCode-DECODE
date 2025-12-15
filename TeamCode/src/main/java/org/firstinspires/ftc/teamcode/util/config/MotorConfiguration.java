package org.firstinspires.ftc.teamcode.util.config;

import org.firstinspires.ftc.teamcode.util.hardwareUtil.Encoder;
import org.firstinspires.ftc.teamcode.util.hardwareUtil.MotorExEx;
import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;

import java.util.Optional;

public class MotorConfiguration {
    private InvertType inverted;
    private GearRatio ratio;
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

    public Optional<MotorExEx> getLeader() {
        return this.leader;
    }
}
