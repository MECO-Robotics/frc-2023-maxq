// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class Extend extends CommandBase {

    private final IntakeSubsystem intake;

    public Extend(IntakeSubsystem intakeIn) {
        intake = intakeIn;
        addRequirements(intakeIn);
    }

    @Override
    public void execute() {
        intake.intakeExtend();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}