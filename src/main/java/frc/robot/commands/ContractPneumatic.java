// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**  */
public class ContractPneumatic extends CommandBase {

  private final ClimbingSubsystem climbingSubsystem;

  /**
   * Creates a new Command.
   *
   * @param 
   */
  public ContractPneumatic(ClimbingSubsystem climbingSubsystem) {
    this.climbingSubsystem = climbingSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbingSubsystem.set2(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.set2(false);
  }

  // Returns true when the command should end. (this command never finishes)
  @Override
  public boolean isFinished() {
    return false;
  }
}
