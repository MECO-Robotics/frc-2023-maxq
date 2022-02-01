// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Drive a requested distance, forward. */
public class DriveStraight extends CommandBase {
  
  private final DriveSubsystem driveTrain;
  private double distanceDesired;
  private final boolean forward;

  /**
   * Creates a new DriveStraight command.
   *
   * @param driveSubsystem The subsystem used by this command.
   * @param distanceInches How far to go forward (negative backwards), in inches.
   */
  public DriveStraight(DriveSubsystem driveSubsystem, double distanceInches) {
    driveTrain = driveSubsystem;
    distanceDesired = Units.inchesToMeters(distanceInches);
    forward = distanceDesired > 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceDesired = driveTrain.getLeftDistance() + distanceDesired;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle;
    if(forward) {
      throttle = Constants.AUTO_SPEED;
    } else {
      throttle = -Constants.AUTO_SPEED;
    }
    // Call repeatedly, even though with the same value because if only called once during 
    // initialize(), the commmand to the CAN bus could get dropped
    driveTrain.arcadeDrive(throttle, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(forward) {
      return driveTrain.getLeftDistance() >= distanceDesired;
    } else {
      return driveTrain.getLeftDistance() <= distanceDesired;
    }
  }
}
