// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimbingArmSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopClimbingArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbingArmSubsystem climbingArmSubsystem;
  private final ControllerSubsystem controllerSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param climbingArm The subsystem used by this command.
   */
  public TeleopClimbingArm(ClimbingArmSubsystem climbingArm, ControllerSubsystem controllers) {
    climbingArmSubsystem = climbingArm;
    controllerSubsystem = controllers;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbingArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbingArmSubsystem.moveArm(controllerSubsystem.getLiftExtenderSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
