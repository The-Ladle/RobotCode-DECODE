package org.firstinspires.ftc.teamcode.util.control;


import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class PIDConstants implements StructSerializable {
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    public PIDConstants() {}

    public PIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setValues(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    public static final PIDConstantsStruct struct = new PIDConstantsStruct();
    public static class PIDConstantsStruct implements Struct<PIDConstants> {

        @Override
        public Class<PIDConstants> getTypeClass() {
            return PIDConstants.class;
        }

        @Override
        public String getTypeName() {
            return "PIDConstants";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 3;
        }

        @Override
        public String getSchema() {
            return "double kP;double kI;double kD;";
        }

        @Override
        public PIDConstants unpack(ByteBuffer bb) {
            var kP = bb.getDouble();
            var kI = bb.getDouble();
            var kD = bb.getDouble();
            return new PIDConstants(kP, kI, kD);
        }

        @Override
        public void unpackInto(PIDConstants out, ByteBuffer bb) {
            out.setValues(bb.getDouble(), bb.getDouble(), bb.getDouble());
        }

        @Override
        public void pack(ByteBuffer bb, PIDConstants value) {
            bb.putDouble(value.getkP());
            bb.putDouble(value.getkI());
            bb.putDouble(value.getkD());
        }
    }
}
