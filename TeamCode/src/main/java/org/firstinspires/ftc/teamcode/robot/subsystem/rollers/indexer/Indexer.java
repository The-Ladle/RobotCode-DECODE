package org.firstinspires.ftc.teamcode.robot.subsystem.rollers.indexer;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.Set;

@Config
public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIO.IndexerIOInputs inputs = new IndexerIO.IndexerIOInputs();

    public static double in = 0.03;
    public static double out = 0.77;

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command in() {
        var subsystem = this;
        return new Command() {

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                io.setPosition(in);
            }

            @Override
            public void end(boolean interrupted) {

            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(subsystem);
            }
        };
    }

    public Command out() {
        var subsystem = this;
        return new Command() {
            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                io.setPosition(out);
            }

            @Override
            public void end(boolean interrupted) {

            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(subsystem);
            }
        };
    }
}
