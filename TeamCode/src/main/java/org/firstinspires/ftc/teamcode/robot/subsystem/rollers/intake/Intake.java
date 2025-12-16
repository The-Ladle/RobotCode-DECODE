package org.firstinspires.ftc.teamcode.robot.subsystem.rollers.intake;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.control.PIDConstants;

import java.util.Collections;
import java.util.Set;
import java.util.function.DoubleSupplier;

@Config
public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    private final PIDConstants constants = new PIDConstants(kP, kI, kD);

    public Intake(IntakeIO io) {
        this.io = io;
        io.updatePID(constants);
    }

//    public final MotorEx intakeOne = new MotorEx("intake_one");
//    public final MotorEx intakeTwo = new MotorEx("intake_two");
//    public final ServoEx indexer = new ServoEx("indexer");
//
//    public final ServoEx climbServo1 = new ServoEx("climb1");
//
//    public final ServoEx climbServo2 = new ServoEx("climb2");
//
//    public final Command extendClimb1 = new SetPosition(climbServo1, 1).requires(this);
//    public final Command extendClimb2 = new SetPosition(climbServo2, 1).requires(this);
//
//    public final Command retractClimb2 = new SetPosition(climbServo2, 0.1).requires(this);
//    public final Command retractClimb1 = new SetPosition(climbServo1, 0.1).requires(this);
//
//
//
//
//
//
//
//    public final Command intakeOnePowerFull = new SetPower(intakeOne, 1).requires(this);
//    public final Command intakeTwoPower8 = new SetPower(intakeTwo, .8).requires(this);
//    public final Command intakeTwoPowerFull = new SetPower(intakeTwo, 1).requires(this);
//
//    public final Command intakeTwoZero = new SetPower(intakeTwo, 0).requires(this);
//    public final Command intakeOneZero = new SetPower(intakeOne, 0).requires(this);
//
//    public final Command intakeOn = new SequentialGroup(intakeOnePowerFull, intakeTwoPowerFull);
//    public final Command intakeOff = new SequentialGroup(intakeOneZero, intakeTwoZero);
//
//    public final Command indexerIn = new SetPosition(indexer, .03).requires(this);
//    public final Command indexerOut = new SetPosition(indexer, .77).requires(this);
    @Override
    public void periodic() {
        io.periodic();
        io.updateInputs(inputs);
        constants.setValues(kP, kI, kD);
        io.updatePID(constants);
    }

    public Command intake(DoubleSupplier mps) {
        var subsystem = this;
        return new Command() {

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                io.setSurfaceSpeed(mps.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                io.setRawSpeed(0);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(subsystem);
            }
        };
    }

    public Command stop() {
        var subsystem = this;
        return new Command() {

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                io.setRawSpeed(0);
            }

            @Override
            public void end(boolean interrupted) {
                io.setRawSpeed(0);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(subsystem);
            }
        };
    }
}


