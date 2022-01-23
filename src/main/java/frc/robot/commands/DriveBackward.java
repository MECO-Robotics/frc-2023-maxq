// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Drive a requested distance, backwards. */
public class DriveBackward extends CommandBase {
  
  private final DriveSubsystem driveTrain;
  private double distanceDesired;

  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem The subsystem used by this command.
   */
  public DriveBackward(DriveSubsystem driveSubsystem, double distanceInches) {
    driveTrain = driveSubsystem;
    distanceDesired = Units.inchesToMeters(distanceInches);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Our destination is actually whever we started, minus how far we need to go
    distanceDesired = driveTrain.getLeftDistance() - distanceDesired;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.arcadeDrive(-Constants.AUTO_SPEED, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.printf("DriveBackward: ENCODER: %f  DESIRED: %f\n", driveTrain.getLeftDistance(), distanceDesired);
    return driveTrain.getLeftDistance() <= distanceDesired;
  }
}
