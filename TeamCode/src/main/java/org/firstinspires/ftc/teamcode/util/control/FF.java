package org.firstinspires.ftc.teamcode.util.control;

public class FF {

    private FFConstants ffConstants;

    private double velocity;
    private double acceleration;

    public FF() {}

    public FF(FFConstants ffConstants) {
        this.ffConstants = ffConstants;
    }

    public void setConstants(double kS, double kV, double kA, double kG) {
        this.ffConstants.setValues(kS, kV, kA, kG);
    }

    public void setConstants(FFConstants constants) {
        this.ffConstants = constants;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    public FFConstants getConstants() {
        return ffConstants;
    }

    /**
     * NOTE: If using something like an arm, manually update kG in your code and use setConstants() before calculation
     */
    public double calculate() {
        double sign = Math.signum(velocity);

        double kSTerm = ffConstants.getkS() * sign;
        double kVTerm = ffConstants.getkV() * velocity;
        double kATerm = ffConstants.getkA() * acceleration;

        double kGTerm = ffConstants.getkG();

        return kSTerm + kVTerm + kATerm + kGTerm;
    }
}