package org.firstinspires.ftc.teamcode.util.control;

public interface MotionProfile {
    public default double sample(double tSecs) { return 0.0; }
}
