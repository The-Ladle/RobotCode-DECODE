package org.firstinspires.ftc.teamcode.util.control;

public class PID {
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    private double setPoint;
    private double currentPosition;

    public PID() {}

    public void setConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setConstants(PIDConstants constants) {
        this.kP = constants.getP();
        this.kI = constants.getI();
        this.kD = constants.getD();
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void setCurrentPosition(double currentPosition) {
        this.currentPosition = currentPosition;
    }

    public PIDConstants getConstants() {
        return new PIDConstants(
            kP,
            kI,
            kD
        );
    }

    public double calculate(double error) {
        
    }

    public double calculate() {

    }
}
