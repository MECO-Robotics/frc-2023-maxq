// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * This command interrupts any command that requires the climbing subsystem.
 * It does nothing itself, because the command that are interrupted should
 * do what they need to do in thier end() calls.
 */
public class CancelClimb extends CommandBase {

  /**
   *
   * @param ClimbingSubsystem The subsystem used by this command.
   */
  public CancelClimb(ClimbingSubsystem subsystem) {

    // By adding the subsystem, just like the Climb command, when this command is invoked,
    // it interrupts the Climb command
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  /**
   * Returns true when the position is at least 95% there.
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}
