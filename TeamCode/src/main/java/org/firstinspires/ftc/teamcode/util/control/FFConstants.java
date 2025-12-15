package org.firstinspires.ftc.teamcode.util.control;


import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class FFConstants implements StructSerializable {
    private double kS = 0;
    private double kV = 0;
    private double kA = 0;
    private double kG = 0;

    public FFConstants() {}

    public FFConstants(double kS, double kV, double kA, double kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    public void setValues(double kS, double kV, double kA, double kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }

    public double getkS() {
        return kS;
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }

    public double getkG() {
        return kG;
    }

    public static final FFConstantsStruct struct = new FFConstantsStruct();
    public static class FFConstantsStruct implements Struct<FFConstants> {

        @Override
        public Class<FFConstants> getTypeClass() {
            return FFConstants.class;
        }

        @Override
        public String getTypeName() {
            return "FFConstants";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 4;
        }

        @Override
        public String getSchema() {
            return "double kS;double kV;double kA;double kG;";
        }

        @Override
        public FFConstants unpack(ByteBuffer bb) {
            var kS = bb.getDouble();
            var kV = bb.getDouble();
            var kA = bb.getDouble();
            var kG = bb.getDouble();
            return new FFConstants(kS, kV, kA, kG);
        }

        @Override
        public void unpackInto(FFConstants out, ByteBuffer bb) {
            out.setValues(bb.getDouble(), bb.getDouble(), bb.getDouble(), bb.getDouble());
        }

        @Override
        public void pack(ByteBuffer bb, FFConstants value) {
            bb.putDouble(value.getkS());
            bb.putDouble(value.getkV());
            bb.putDouble(value.getkA());
            bb.putDouble(value.getkG());
        }
    }
}