package org.firstinspires.ftc.teamcode.robot.subsystem.rollers.indexer;

import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.robot.constants.HardwareDevices;

public class IndexerIOServo implements IndexerIO {
    private final CRServoEx servo = HardwareDevices.indexerServoID.servo();

    public IndexerIOServo() {

    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerConnected = true;
        inputs.indexer.updateFrom(servo);

    }

    @Override
    public void setPosition(double position) {
        servo.set(position);
    }
}
