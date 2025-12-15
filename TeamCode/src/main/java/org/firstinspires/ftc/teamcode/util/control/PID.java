package org.firstinspires.ftc.teamcode.util.control;

import static edu.wpi.first.units.Units.Seconds;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.RobotState;

public class PID {
    private PIDConstants pidConstants;

    private double setPoint;
    private double currentPosition;

    private double allowedError;

    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private double dtSec = 0.0;

    public PID() {}

    public PID(PIDConstants pidConstants) {
        this.setConstants(pidConstants);
    }

    public void setConstants(double kP, double kI, double kD) {
        this.pidConstants.setValues(kP, kI, kD);
    }

    public void setConstants(PIDConstants constants) {
        this.pidConstants = constants;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void setCurrentPosition(double currentPosition) {
        this.currentPosition = currentPosition;
    }

    public void setAllowedError(double error) {
        this.allowedError = error;
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
    }

    public PIDConstants getConstants() {
        return pidConstants;
    }

    public double getSetPoint() {
        return this.setPoint;
    }

    public double getCurrentPosition() {
        return this.currentPosition;
    }

    public double getAllowedError() {
        return this.allowedError;
    }

    public double calculate(double error) {
        double absError = Math.abs(error);

        if (absError < allowedError) {
            integral = 0.0;
            lastError = 0.0;
            return 0.0;
        }
        double time = RobotState.getInstance().getElapsedTime().in(Seconds);
        dtSec = time - lastTime;
        lastTime = time;

        integral += error * dtSec;

        double deriv = (error - lastError) / dtSec;
        lastError = error;

        return pidConstants.getkP() * error + pidConstants.getkI() * integral + pidConstants.getkD() * deriv;
    }

    public double calculate() {
        double error = setPoint - currentPosition;
        double absError = Math.abs(error);

        if (absError < allowedError) {
            integral = 0.0;
            lastError = 0.0;
            return 0.0;
        }
        double time = RobotState.getInstance().getElapsedTime().in(Seconds);
        dtSec = time - lastTime;
        lastTime = time;

        integral += error * dtSec;

        double deriv = (error - lastError) / dtSec;
        lastError = error;

        return pidConstants.getkP() * error + pidConstants.getkI() * integral + pidConstants.getkD() * deriv;
    }
}
