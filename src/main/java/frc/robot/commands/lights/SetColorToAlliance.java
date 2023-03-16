// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.LightSubsystem;

public class SetColorToAlliance extends CommandBase {

    private final LightSubsystem lights;

    public SetColorToAlliance(LightSubsystem lightSubsystem) {
        lights = lightSubsystem;
        addRequirements(lightSubsystem);
    }

    @Override
    public void execute() {
        Alliance alliance = DriverStation.getAlliance();
        Color allianceColor = Color.kBlack;

        if (alliance == Alliance.Red) {
            allianceColor = Color.kRed;
        } else if (alliance == Alliance.Blue) {
            allianceColor = Color.kBlue;
        } 

        lights.set(allianceColor);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
