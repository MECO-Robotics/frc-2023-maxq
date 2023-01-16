// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Spin right for a desired distance using the left wheel to measure. 
 * Useful if a gyro is not installed.
 */
public class SpinRightDistance extends CommandBase {
  
  private final DriveSubsystem driveTrain;
  private double distanceDesired;

  /**
   * Creates a new DriveForward command.
   *
   * @param driveSubsystem The subsystem used by this command.
   * @param leftWheelDistanceInches How far to go forward, in inches.
   */
  public SpinRightDistance(DriveSubsystem driveSubsystem, double leftWheelDistanceInches) {
    driveTrain = driveSubsystem;
    distanceDesired = Units.inchesToMeters(leftWheelDistanceInches);

    // If this command is called, we want to interupt any other commands running
    // on the driving subystem
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //distanceDesired = driveTrain.getLeftDistance() + distanceDesired;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turn at half the auto speed so that ramp and inertia has less of an effect
  //  driveTrain.arcadeDrive(0, Constants.AUTO_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return driveTrain.getLeftDistance() >= distanceDesired;
  }
}
