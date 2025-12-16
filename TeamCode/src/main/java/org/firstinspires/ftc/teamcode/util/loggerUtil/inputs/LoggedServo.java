package org.firstinspires.ftc.teamcode.util.loggerUtil.inputs;

import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.util.hardwareUtil.MotorExEx;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class LoggedServo implements StructSerializable {
    private double currentPosition;
    private double rawPosition;
    private double direction;

    public LoggedServo() {}

    private LoggedServo(double rawPosition, double direction, double currentPosition) {
        this.rawPosition = rawPosition;
        this.direction = direction;
        this.currentPosition = currentPosition;
    }

    public double getRawPosition() {
        return this.rawPosition;
    }

    public int getDirection() {
        return (int) this.direction;
    }

    public double getCurrentPosition() {
        return this.currentPosition;
    }

    public void setDirection(int direction) {
        this.direction = direction;
    }

    public void setCurrentPosition(double currentPosition) {
        this.currentPosition = currentPosition;
    }

    public void setRawPosition(double rawPosition) {
        this.rawPosition = rawPosition;
    }

    public void updateFrom(ServoEx servoEx) {
        this.setRawPosition(servoEx.getRawPosition());
        this.setDirection(servoEx.getInverted() ? 0 : 1);
        this.setCurrentPosition(servoEx.get());
    }

    public static final LoggedServoStruct struct = new LoggedServoStruct();
    public static class LoggedServoStruct implements Struct<LoggedServo> {

        @Override
        public Class<LoggedServo> getTypeClass() {
            return LoggedServo.class;
        }

        @Override
        public String getTypeName() {
            return "Servo";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 3;
        }

        @Override
        public String getSchema() {
            return "double direction;double rawPosition;double currentPosition;";
        }

        @Override
        public LoggedServo unpack(ByteBuffer bb) {
            var direction = bb.getDouble();
            var rawPosition = bb.getDouble();
            var currentPosition = bb.getDouble();
            return new LoggedServo(rawPosition, direction, currentPosition);
        }

        @Override
        public void unpackInto(LoggedServo out, ByteBuffer bb) {
            out.setDirection((int) bb.getDouble());
            out.setRawPosition(bb.getDouble());
            out.setCurrentPosition(bb.getDouble());
        }

        @Override
        public void pack(ByteBuffer bb, LoggedServo value) {
            bb.putDouble(value.getDirection());
            bb.putDouble(value.getRawPosition());
            bb.putDouble(value.getCurrentPosition());
        }
    }
}
