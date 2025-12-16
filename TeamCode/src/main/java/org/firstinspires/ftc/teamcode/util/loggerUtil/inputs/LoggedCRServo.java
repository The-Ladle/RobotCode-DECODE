package org.firstinspires.ftc.teamcode.util.loggerUtil.inputs;

import org.firstinspires.ftc.teamcode.util.hardwareUtil.CRServoExEx;
import org.firstinspires.ftc.teamcode.util.hardwareUtil.MotorExEx;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;

public class LoggedCRServo {
    private double power;
    private double acceleration;
    private double direction;
    private double currentPosition;
    private double velocity;

    public LoggedCRServo() {}

    private LoggedCRServo(double power, double acceleration, double direction, double currentPosition, double velocity) {
        this.power = power;
        this.acceleration = acceleration;
        this.direction = direction;
        this.currentPosition = currentPosition;
        this.velocity = velocity;
    }

    public double getPower() {
        return this.power;
    }

    public double getAcceleration() {
        return this.acceleration;
    }

    public int getDirection() {
        return (int) this.direction;
    }

    public double getCurrentPosition() {
        return this.currentPosition;
    }

    public double getVelocity() {
        return this.velocity;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    public void setDirection(int direction) {
        this.direction = direction;
    }

    public void setCurrentPosition(double currentPosition) {
        this.currentPosition = currentPosition;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public void updateFrom(CRServoExEx motorEx) {
        this.setPower(motorEx.getPower());
        this.setAcceleration(motorEx.getAcceleration());
        this.setDirection(motorEx.getDirection());
        this.setCurrentPosition(motorEx.getCurrentPosition());
        this.setVelocity(motorEx.getVelocity());
    }

    public static final LoggedCRServo.LoggedCRServoStruct struct = new LoggedCRServo.LoggedCRServoStruct();
    public static class LoggedCRServoStruct implements Struct<LoggedCRServo> {

        @Override
        public Class<LoggedCRServo> getTypeClass() {
            return LoggedCRServo.class;
        }

        @Override
        public String getTypeName() {
            return "Motor";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 5;
        }

        @Override
        public String getSchema() {
            return "double power;double acceleration;double direction;double currentPosition;double velocity;";
        }

        @Override
        public LoggedCRServo unpack(ByteBuffer bb) {
            var power = bb.getDouble();
            var acceleration = bb.getDouble();
            var direction = bb.getDouble();
            var currentPosition = bb.getDouble();
            var velocity = bb.getDouble();
            return new LoggedCRServo(power, acceleration, direction, currentPosition, velocity);
        }

        @Override
        public void unpackInto(LoggedCRServo out, ByteBuffer bb) {
            out.setPower(bb.getDouble());
            out.setAcceleration(bb.getDouble());
            out.setDirection((int) bb.getDouble());
            out.setCurrentPosition(bb.getDouble());
            out.setVelocity(bb.getDouble());
        }

        @Override
        public void pack(ByteBuffer bb, LoggedCRServo value) {
            bb.putDouble(value.getPower());
            bb.putDouble(value.getAcceleration());
            bb.putDouble(value.getDirection());
            bb.putDouble(value.getCurrentPosition());
            bb.putDouble(value.getVelocity());
        }
    }
}
