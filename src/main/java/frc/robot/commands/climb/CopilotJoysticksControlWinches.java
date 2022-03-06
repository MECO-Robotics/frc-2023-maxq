// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * Manual control of the climbing subsystem winch motors.
 */
public class CopilotJoysticksControlWinches extends CommandBase {

  private final ClimbingSubsystem climb;
  private final ControllerSubsystem controllers;
  
  /**
   *
   * @param ClimbingSubsystem The subsystem used by this command.
   */
  public CopilotJoysticksControlWinches(ClimbingSubsystem subsystem, ControllerSubsystem control) {
    climb = subsystem;
    controllers = control;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.rotatingArmWinchMove(controllers.getCopilotController().getLeftY(), controllers.getCopilotController().getLeftY());
    climb.telescopingArmWinchMove(controllers.getCopilotController().getRightY(), controllers.getCopilotController().getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.rotatingArmWinchMove(0, 0);
    climb.telescopingArmWinchMove(0, 0);
  }

  /**
   * Returns true when the position is at least 95% there.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
