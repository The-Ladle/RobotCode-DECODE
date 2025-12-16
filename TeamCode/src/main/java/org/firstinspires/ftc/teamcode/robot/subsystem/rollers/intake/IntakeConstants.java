package org.firstinspires.ftc.teamcode.robot.subsystem.rollers.intake;

import static edu.wpi.first.units.Units.Inches;

import org.firstinspires.ftc.teamcode.util.mechanismUtil.GearRatio;
import org.firstinspires.ftc.teamcode.util.mechanismUtil.LinearRelation;

public class IntakeConstants {
    public static final GearRatio motorToMechanism = new GearRatio()
            .sprocket(28)
            .sprocket(1)
            .planetary(1)
            ;

    public static final LinearRelation wheel = LinearRelation.wheelDiameter(Inches.of(4));
}
