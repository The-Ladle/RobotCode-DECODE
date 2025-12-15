package org.firstinspires.ftc.teamcode.subsystem.rollers.intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    public final MotorEx intakeOne = new MotorEx("intake_one");
    public final MotorEx intakeTwo = new MotorEx("intake_two");
    public final ServoEx indexer = new ServoEx("indexer");

    public final ServoEx climbServo1 = new ServoEx("climb1");

    public final ServoEx climbServo2 = new ServoEx("climb2");

    public final Command extendClimb1 = new SetPosition(climbServo1, 1).requires(this);
    public final Command extendClimb2 = new SetPosition(climbServo2, 1).requires(this);

    public final Command retractClimb2 = new SetPosition(climbServo2, 0.1).requires(this);
    public final Command retractClimb1 = new SetPosition(climbServo1, 0.1).requires(this);







    public final Command intakeOnePowerFull = new SetPower(intakeOne, 1).requires(this);
    public final Command intakeTwoPower8 = new SetPower(intakeTwo, .8).requires(this);
    public final Command intakeTwoPowerFull = new SetPower(intakeTwo, 1).requires(this);

    public final Command intakeTwoZero = new SetPower(intakeTwo, 0).requires(this);
    public final Command intakeOneZero = new SetPower(intakeOne, 0).requires(this);

    public final Command intakeOn = new SequentialGroup(intakeOnePowerFull, intakeTwoPowerFull);
    public final Command intakeOff = new SequentialGroup(intakeOneZero, intakeTwoZero);

    public final Command indexerIn = new SetPosition(indexer, .03).requires(this);
    public final Command indexerOut = new SetPosition(indexer, .77).requires(this);

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {

    }
}


