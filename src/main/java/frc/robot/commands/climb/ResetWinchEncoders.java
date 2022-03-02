// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** With ALL the winches all the way in, set all encoders to zero. */
public class ResetWinchEncoders extends CommandBase {

  private final ClimbingSubsystem climb;
  
  /**
   *
   * @param ClimbingSubsystem The subsystem used by this command.
   */
  public ResetWinchEncoders(ClimbingSubsystem subsystem) {
    climb = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    climb.winchResetZero();
  }

  /**
   * Returns true when the position is at least 95% there.
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}
