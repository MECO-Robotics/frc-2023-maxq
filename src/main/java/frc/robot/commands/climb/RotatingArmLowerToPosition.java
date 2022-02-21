// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 *  The code will allow you to set the telescoping arm.
 */
public class RotatingArmLowerToPosition extends CommandBase {

  private final ClimbingSubsystem climb;
  private final double position;
  /**
   *
   * @param ClimbingSubsystem The subsystem used by this command.
   */
  public RotatingArmLowerToPosition(ClimbingSubsystem subsystem, double position) {
    climb = subsystem;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.rotatingArmSetWinch(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  /**
   * 
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
