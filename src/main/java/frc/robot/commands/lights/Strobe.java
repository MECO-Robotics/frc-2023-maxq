// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightSubsystem;

public class Strobe extends CommandBase {
    private final LightSubsystem lights;

    public Strobe(LightSubsystem lightSubsystem) {
        lights = lightSubsystem;
    }

    @Override
    public void execute() {
        // TODO Write the Strobe.execute() method
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
