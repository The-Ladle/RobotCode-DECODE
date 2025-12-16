package org.firstinspires.ftc.teamcode.robot.subsystem.rollers.intake;

import static org.firstinspires.ftc.teamcode.robot.subsystem.rollers.intake.IntakeConstants.motorToMechanism;
import static org.firstinspires.ftc.teamcode.robot.subsystem.rollers.intake.IntakeConstants.wheel;

import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.robot.constants.HardwareDevices;
import org.firstinspires.ftc.teamcode.util.config.ControlType;
import org.firstinspires.ftc.teamcode.util.config.InvertType;
import org.firstinspires.ftc.teamcode.util.config.MotorConfiguration;
import org.firstinspires.ftc.teamcode.util.control.MotionProfile;
import org.firstinspires.ftc.teamcode.util.control.PIDConstants;
import org.firstinspires.ftc.teamcode.util.hardwareUtil.MotorExEx;

public class IntakeIOMotor implements IntakeIO {
    private final MotorExEx motorOne = HardwareDevices.intakeOneMotorID.motor();
    private final MotorExEx motorTwo = HardwareDevices.intakeTwoMotorID.motor();

    public IntakeIOMotor() {
        var config = new MotorConfiguration();
        config.setInverted(InvertType.COUNTERCLOCKWISE_POSITIVE);
        config.setRatio(motorToMechanism);
        config.setControlType(ControlType.VELOCITY);
        config.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorOne.applyConfig(config);
        config.setLeader(motorOne);
        motorTwo.applyConfig(config);
    }

    @Override
    public void periodic() {
        motorOne.periodic();
        motorTwo.periodic();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeOneConnected = true;
        inputs.intakeOne.updateFrom(motorOne);
        inputs.intakeTwoConnected = true;
        inputs.intakeTwo.updateFrom(motorTwo);
    }

    @Override
    public void setRawSpeed(double speed) {
        motorOne.setPower(speed);
    }

    @Override
    public void setSurfaceSpeed(double mps) {
        motorOne.setTargetState(new MotionProfile.State(wheel.metersToRotations(mps),0));
    }

    @Override
    public void updatePID(PIDConstants constants) {
        motorOne.updatePID(constants);
    }
}
