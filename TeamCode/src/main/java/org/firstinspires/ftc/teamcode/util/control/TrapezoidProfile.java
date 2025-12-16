package org.firstinspires.ftc.teamcode.util.control;

public class TrapezoidProfile {

    private final double maxVelocity;
    private final double maxAcceleration;

    public TrapezoidProfile(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    /**
     * Calculate the state at a given time tSecs for a trapezoidal profile
     * from start to end.
     */
    public MotionProfile.State calculate(double tSecs, MotionProfile.State start, MotionProfile.State end) {
        double distance = end.position - start.position;
        double direction = Math.signum(distance);
        double v0 = start.velocity * direction;
        double vf = end.velocity * direction;
        double d = Math.abs(distance);

        double tAccel = (maxVelocity - v0) / maxAcceleration;
        double dAccel = v0 * tAccel + 0.5 * maxAcceleration * tAccel * tAccel;

        double tDecel = (maxVelocity - vf) / maxAcceleration;
        double dDecel = vf * tDecel + 0.5 * maxAcceleration * tDecel * tDecel;

        double dFlat = d - (dAccel + dDecel);
        double tFlat = dFlat / maxVelocity;

        if (dFlat < 0) {
            double peakV = Math.sqrt((maxAcceleration * d) + (v0 * v0 + vf * vf) / 2);
            tAccel = (peakV - v0) / maxAcceleration;
            tDecel = (peakV - vf) / maxAcceleration;
            tFlat = 0;
        }

        double tTotal = tAccel + tFlat + tDecel;

        if (tSecs <= 0) return start;
        if (tSecs >= tTotal) return end;

        double pos, vel;

        if (tSecs < tAccel) {
            vel = v0 + maxAcceleration * tSecs;
            pos = start.position + v0 * tSecs + 0.5 * maxAcceleration * tSecs * tSecs;
        } else if (tSecs < tAccel + tFlat) {
            vel = maxVelocity;
            pos = start.position + dAccel + maxVelocity * (tSecs - tAccel);
        } else {
            double tDecelPhase = tSecs - tAccel - tFlat;
            vel = maxVelocity - maxAcceleration * tDecelPhase;
            pos = end.position - (vf * (tDecel - tDecelPhase) + 0.5 * maxAcceleration * (tDecel - tDecelPhase) * (tDecel - tDecelPhase));
        }

        return new MotionProfile.State(pos, vel * direction);
    }
}