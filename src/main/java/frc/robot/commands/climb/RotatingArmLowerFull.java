// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * Fully raise the rotating arm.
 */
public class RotatingArmLowerFull extends CommandBase {

  private final ClimbingSubsystem climb;
  
  /**
   *
   * @param ClimbingSubsystem The subsystem used by this command.
   */
  public RotatingArmLowerFull(ClimbingSubsystem subsystem) {
    climb = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    climb.rotatingArmSetWinch(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted) {
      climb.rotatingArmPneumaticIn();
    }
  }

  /**
   * Returns true when the position is at least 95% there.
   */
  @Override
  public boolean isFinished() {
    return climb.getRotatingArmWinchPosition() < 0.05;
  }
}
