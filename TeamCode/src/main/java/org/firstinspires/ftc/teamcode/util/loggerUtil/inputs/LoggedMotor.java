package org.firstinspires.ftc.teamcode.util.loggerUtil.inputs;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.ByteBuffer;

import Ori.Coval.Logging.Logged;
import dev.nextftc.hardware.impl.MotorEx;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class LoggedMotor implements StructSerializable {
    private double power;
    private double rawTicks;
    private double direction;
    private double currentPosition;
    private double velocity;

    public LoggedMotor() {}

    private LoggedMotor(double power, double rawTicks, double direction, double currentPosition, double velocity) {
        this.power = power;
        this.rawTicks = rawTicks;
        this.direction = direction;
        this.currentPosition = currentPosition;
        this.velocity = velocity;
    }

    public double getPower() {
        return this.power;
    }

    public double getRawTicks() {
        return this.rawTicks;
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

    public void setRawTicks(double rawTicks) {
        this.rawTicks = rawTicks;
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

    public void updateFrom(MotorEx motorEx) {
        this.setPower(motorEx.getPower());
        this.setRawTicks(motorEx.getRawTicks());
        this.setDirection(motorEx.getDirection());
        this.setCurrentPosition(motorEx.getCurrentPosition());
        this.setVelocity(motorEx.getVelocity());
    }

    public static final LoggedMotorStruct struct = new LoggedMotorStruct();
    public static class LoggedMotorStruct implements Struct<LoggedMotor> {

        @Override
        public Class<LoggedMotor> getTypeClass() {
            return LoggedMotor.class;
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
            return "double power;double rawTicks;double direction;double currentPosition;double velocity;";
        }

        @Override
        public LoggedMotor unpack(ByteBuffer bb) {
            var power = bb.getDouble();
            var rawTicks = bb.getDouble();
            var direction = bb.getDouble();
            var currentPosition = bb.getDouble();
            var velocity = bb.getDouble();
            return new LoggedMotor(power, rawTicks, direction, currentPosition, velocity);
        }

        @Override
        public void unpackInto(LoggedMotor out, ByteBuffer bb) {
            out.setPower(bb.getDouble());
            out.setRawTicks(bb.getDouble());
            out.setDirection((int) bb.getDouble());
            out.setCurrentPosition(bb.getDouble());
            out.setVelocity(bb.getDouble());
        }

        @Override
        public void pack(ByteBuffer bb, LoggedMotor value) {
            bb.putDouble(value.getPower());
            bb.putDouble(value.getRawTicks());
            bb.putDouble(value.getDirection());
            bb.putDouble(value.getCurrentPosition());
            bb.putDouble(value.getVelocity());
        }
    }
}
