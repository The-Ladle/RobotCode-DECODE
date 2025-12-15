package org.firstinspires.ftc.teamcode.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.qualcomm.robotcore.util.ElapsedTime;

import edu.wpi.first.units.measure.Time;

public class RobotState {
    public static RobotState getInstance() {
        return instance;
    }
    private static RobotState instance;

    private final ElapsedTime elapsedTime;

    public RobotState() {
        instance = this;
        elapsedTime = new ElapsedTime();
    }

    public Time getElapsedTime() {
        return Seconds.of(elapsedTime.seconds());
    }
}
