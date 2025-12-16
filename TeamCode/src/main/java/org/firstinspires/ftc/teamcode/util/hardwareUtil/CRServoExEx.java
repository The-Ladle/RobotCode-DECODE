package org.firstinspires.ftc.teamcode.util.hardwareUtil;

import static edu.wpi.first.units.Units.Seconds;

import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.firstinspires.ftc.teamcode.robot.constants.HardwareDevices;
import org.firstinspires.ftc.teamcode.util.config.ControlType;
import org.firstinspires.ftc.teamcode.util.config.InvertType;
import org.firstinspires.ftc.teamcode.util.config.MotorConfiguration;
import org.firstinspires.ftc.teamcode.util.control.FF;
import org.firstinspires.ftc.teamcode.util.control.FFConstants;
import org.firstinspires.ftc.teamcode.util.control.MotionProfile;
import org.firstinspires.ftc.teamcode.util.control.PID;
import org.firstinspires.ftc.teamcode.util.control.PIDConstants;
import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;

import java.util.Optional;

public class CRServoExEx {
    private final CRServoEx motor;
    private InvertType invertType;
    private ControlType controlType;
    private GearRatio ratio = new GearRatio(1);
    private MotionProfile.State targetState = new MotionProfile.State(0, 0);
    private Optional<PID> pid;
    private Optional<FF> ff;
    private Optional<MotionProfile> motionProfile;
    private Optional<MotorExEx> leader;
    private Optional<Encoder> externalEncoder;
    private double profileStartTime = -1;
    private MotionProfile.State startState = new MotionProfile.State(0, 0);
    private double lastTime = 0;
    private double lastVelocity = 0;
    private double acceleration = 0;

    public CRServoExEx(String name) {
        this.motor = new CRServoEx(HardwareDevices.hardwareMap, name);
    }

    /**
     * NEEDS to be called if using as a follower motor, using an external encoder, running a "on-device" control loop, or getting acceleration. Doesn't need to be called with otherwise.
     */
    public void periodic() {
        var time = RobotState.getInstance().getElapsedTime().in(Seconds);
        var velo = motor.getCorrectedVelocity();
        acceleration = (velo - lastVelocity) / (time - lastTime);
        lastVelocity = velo;
        lastTime = time;

        MotionProfile.State measuredState;
        if (controlType == ControlType.POSITION) {
            measuredState = new MotionProfile.State(this.getCurrentPosition(), this.getVelocity());
        } else {
            measuredState = new MotionProfile.State(this.getVelocity(), this.getAcceleration());
        }

        double power = 0.0;

        MotionProfile.State setpoint = targetState;
        if (motionProfile.isPresent()) {
            if (profileStartTime < 0) {
                profileStartTime = RobotState.getInstance().getElapsedTime().in(Seconds);
                startState = measuredState;
            }

            double tSecs = (RobotState.getInstance().getElapsedTime().in(Seconds)) - profileStartTime;
            setpoint = motionProfile.get().calculate(tSecs, startState, targetState);
            targetState = setpoint;
        }

        if (pid.isPresent()) {
            var pidVar = pid.get();
            pidVar.setCurrentPosition(measuredState.position);
            pidVar.setSetPoint(setpoint.position);
            power += pidVar.calculate();
        }

        if (ff.isPresent()) {
            var ffVar = ff.get();
            ffVar.setVelocity(setpoint.velocity);
            ffVar.setAcceleration(0);
            double ffOutput = ff.get().calculate();
            power += ffOutput;
        }

        if (leader.isPresent()) {
            this.setPower(leader.get().getPower());
        } else {
            motor.set(power);
        }
    }


    /**
     * To use a gear ratio in own logic, set the ratio in the config to 1 using new GearRatio(1)
     */
    public void applyConfig(MotorConfiguration configuration) {
        motor.setInverted(configuration.getInverted() == InvertType.CLOCKWISE_POSITIVE);
        motor.setZeroPowerBehavior(configuration.getZeroPowerBehavior());
        ratio = configuration.getRatio();
        leader = configuration.getLeader();
        externalEncoder = configuration.getEncoder();
        invertType = configuration.getInverted();
        controlType = configuration.getControlType();
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public double getCurrentPosition() {
        return externalEncoder.map(Encoder::getPosition).orElseGet(() -> ratio.applySigned(motor.getCurrentPosition()));
    }

    public double getPower() {
        return motor.get();
    }

    public InvertType getInverted() {
        return invertType;
    }

    public int getDirection() {
        return motor.getInverted() ? 0 : 1;
    }

    public double getVelocity() {
        return externalEncoder.map(Encoder::getVelocity).orElseGet(() -> ratio.applySigned(motor.getCorrectedVelocity()));
    }

    public double getAcceleration() {
        return externalEncoder.map(Encoder::getAcceleration).orElseGet(() -> ratio.applySigned(acceleration));
    }

    public void updatePID(PIDConstants pidConstants) {
        if(this.pid.isPresent()) {
            this.pid.get().setConstants(pidConstants);
        } else {
            this.pid = Optional.of(new PID(pidConstants));
        }
    }

    public void updateFF(FFConstants ffConstants) {
        if(this.ff.isPresent()) {
            this.ff.get().setConstants(ffConstants);
        } else {
            this.ff = Optional.of(new FF(ffConstants));
        }
    }

    public void addMotionProfile(MotionProfile profile) {
        motionProfile = Optional.of(profile);
    }

    public void setTargetState(MotionProfile.State state) {
        targetState = state;
    }
}
