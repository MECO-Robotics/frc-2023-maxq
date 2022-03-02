// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * Fully lower the rotating arm for parking / end of match.
 */
public class TeleopRotatingArmPneumaticOff extends CommandBase {

  private final ClimbingSubsystem climb;
  
  /**
   *
   * @param ClimbingSubsystem The subsystem used by this command.
   */
  public TeleopRotatingArmPneumaticOff(ClimbingSubsystem subsystem) {
    climb = subsystem;

    // We don't want to call addRequirements. If we did, we're saying this is a
    // command that causes other commands currently running on the 
    // ClimbingSubystem to be inerrupted.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.rotatingArmPneumaticOff();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  /**
   * Returns true when the position is at least 95% there.
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}
