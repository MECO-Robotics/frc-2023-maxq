// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Turn right for an angle. */
public class SpinRightAngle extends CommandBase {
  
  private final DriveSubsystem driveTrain;
  private double angleDesired;

  /**
   * Creates a new command.
   *
   * @param driveSubsystem The subsystem used by this command.
   * @param turnDegrees How far to go forward, in inches.
   */
  public SpinRightAngle(DriveSubsystem driveSubsystem, double turnDegrees) {
    driveTrain = driveSubsystem;
    angleDesired = turnDegrees;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Update the desired angle here because we need the current heading 
    // when the command is scheduled, not during construction.
    angleDesired = driveTrain.getHeadingDegrees() + angleDesired;

    // Start the turn
    driveTrain.tankDrive(Constants.AUTO_SPEED, -Constants.AUTO_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // nothing to do here. The isFinished will determine when we've gone far enough.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.getHeadingDegrees() >= angleDesired;
  }
}
