// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * With the arm raised (1.0), grip the bar by pulling in the winch a little 
 */
public class RotatingArmGrabBar extends CommandBase {

  private final ClimbingSubsystem climb;
  
  /**
   *
   * @param ClimbingSubsystem The subsystem used by this command.
   */
  public RotatingArmGrabBar(ClimbingSubsystem subsystem) {
    climb = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    climb.rotatingArmWinchMove(-.5, -.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.rotatingArmWinchMove(0, 0);
  }

  /**
   * Returns true when the position is close to the set position
   */
  @Override
  public boolean isFinished() {
    return climb.getRotatingArmLeftWinchPosition() < .95 && climb.getRotatingArmRightWinchPosition() < .95;
  }
}
