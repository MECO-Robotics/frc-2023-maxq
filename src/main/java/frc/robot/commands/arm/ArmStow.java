// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElbowPosition;
import frc.robot.Constants.ShoulderPosition;

public class ArmStow extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmStow(ArmSubsystem arm) {
        armSubsystem = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        armSubsystem.move(ElbowPosition.allIn);
        armSubsystem.move(ShoulderPosition.allBackStow);
    }

    @Override
    public boolean isFinished() {
        return true;

    }

}