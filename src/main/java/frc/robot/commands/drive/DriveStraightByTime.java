// Copyright (c) MECO Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Drive a requested distance, forward. */
public class DriveStraightByTime extends CommandBase {
  
  private final DriveSubsystem driveTrain;
  private double initialHeading;
  private double speed;
  private double drivingTime;
  /**
   * Creates a new DriveStraight command.
   *
   * @param driveSubsystem The subsystem used by this command.
   * @param distanceInches How far to go forward (negative backwards), in inches.
   */
  

  public DriveStraightByTime(DriveSubsystem driveSubsystem, double desiredTime, double desiredSpeed) {
    driveTrain = driveSubsystem;
    drivingTime = desiredTime;
    speed = desiredSpeed;
    // If this command is called, we want to interupt any other commands running
    // on the driving subystem
    addRequirements(driveSubsystem);
  }

  double startTime;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    initialHeading = driveTrain.getHeadingDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.robotDrive(-speed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("CMD: DriveStraight: end");

    // Use tank drive, which doesn't have speed ramping because we need the motors to
    // stop immediately.
    driveTrain.robotDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = Timer.getFPGATimestamp();
    if((currentTime - startTime) > drivingTime) {
      return true;
    } else {
      return false;
    }

  }
}
