// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * With the arm raised (1.0), grip the bar by pulling in the winch a little 
 */
public class RotatingArmRaisedGrip extends CommandBase {

  private final ClimbingSubsystem climb;
  
  /**
   *
   * @param ClimbingSubsystem The subsystem used by this command.
   */
  public RotatingArmRaisedGrip(ClimbingSubsystem subsystem) {
    climb = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    climb.rotatingArmSetWinch(.95);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  /**
   * Returns true when the position is close to the set position
   */
  @Override
  public boolean isFinished() {
    return climb.getRotatingArmWinchPosition() < .97;
  }
}
