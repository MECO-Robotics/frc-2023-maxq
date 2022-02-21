// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * Move the rotating arm in response to user input. This is an interuptable command.
 * Once the end() is called, it finishes. Otherwise, it never finishes.
 */
public class RotatingArmMove extends CommandBase {

  private final ClimbingSubsystem climb;
  private final double speed;
  
  /**
   * Move the rotating arm up. This command is intended to be a hold-to-activate button 
   * available on Shuffleboard, primarily for testing purposes. The speed should be set
   * on the RobotContainer.
   * 
   * @param ClimbingSubsystem The subsystem used by this command.
   */
  public RotatingArmMove(ClimbingSubsystem subsystem, double speed) {
    climb = subsystem;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.rotatingArmWinchMove(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.rotatingArmWinchMove(0);
  }

  /**
   * 
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
