// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;

public class SetColor extends CommandBase {
    
    private final LightSubsystem lights;
    private final Color color;

    public SetColor(LightSubsystem lightSubsystem, Color colour) {
        lights = lightSubsystem;
        color = colour;
    }

    @Override
    public void execute() {
        lights.set(color);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
