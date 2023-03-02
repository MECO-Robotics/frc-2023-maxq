// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Spin extends CommandBase {

    private final IntakeSubsystem intake;
    private final ControllerSubsystem controller;

    public Spin(IntakeSubsystem intakeIn, ControllerSubsystem controllerIn){
        intake = intakeIn;
        controller = controllerIn;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intakeSpin(controller.getCopilotController().getLeftTriggerAxis());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}