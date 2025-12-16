package org.firstinspires.ftc.teamcode.util.control;

public interface MotionProfile {
    public default State calculate(double tSecs, State start, State end) { return new State(0.0, 0.0); }

    public class State {
        public final double position;
        public final double velocity;

        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }
}
