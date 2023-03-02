// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.brakes;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.BrakesSubsystem;

public class LowerBrakes extends CommandBase {

    private final BrakesSubsystem brakes;

    public LowerBrakes(BrakesSubsystem brakesIn) {
        brakes = brakesIn;
        addRequirements(brakesIn);
        // brakesobama
    }


    @Override
    public void execute() {
        brakes.lowerBrakes();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
}